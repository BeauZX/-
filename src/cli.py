"""CLI：算前方路面縱向坡度。兩種來源——離線影片 或 即時鏡頭。

離線（讀錄好的一段）：
    python3 main.py SEG_DIR [--calib calib.npz] [--out road_angle.csv]
    python3 main.py --cam0 a.mp4 --cam1 b.mp4 --calib calib.npz

即時（接兩顆 Picamera2 鏡頭，Ctrl+C 停止）：
    python3 main.py --live [--out road_angle.csv]

SEG_DIR 若含 cam*_pts.txt / start_time.json 會自動用時間戳配對，否則序號配對。
所有演算法參數固定在 config.py。
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path
from typing import Iterator, TextIO

from .calib_loader import load_calibration
from .config import DEFAULT
from .pipeline import FrameResult, process_live, process_segment

_CSV_FIELDS = [
    "index",
    "pitch_deg",
    "roll_deg",
    "cam_height_m",
    "n_inliers",
    "n_road_points",
    "rms_m",
    "time_diff_ms",
]


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="road-angle",
        description="純雙目立體視覺預測前方路面縱向坡度 (pitch)。",
    )
    p.add_argument(
        "seg_dir",
        nargs="?",
        help="錄影段落資料夾（含 cam0.mp4/cam1.mp4，可選 pts/start_time.json）",
    )
    # 演算法參數全部固定在 config.py，這裡只留「每次跑會變」的路徑/來源選項。
    p.add_argument("--live", action="store_true", help="接兩顆即時鏡頭直接算 (Picamera2)，Ctrl+C 停止")
    ui = p.add_mutually_exclusive_group()
    ui.add_argument("--ui", dest="ui", action="store_true", default=None, help="開 PyQt 視覺介面（即時模式）")
    ui.add_argument("--no-ui", dest="ui", action="store_false", help="強制純終端機、不開介面")
    # 錄影一律開啟（寫死在 config.record=True），即時模式自動存到 output/segment_NNN/。
    p.add_argument(
        "--calib", default=DEFAULT.calib_path, help=f"calib.npz 路徑（預設 {DEFAULT.calib_path}）"
    )
    p.add_argument("--cam0", help="離線：覆寫 cam0(左) 影片路徑")
    p.add_argument("--cam1", help="離線：覆寫 cam1(右) 影片路徑")
    p.add_argument("--out", help="輸出 CSV 路徑（離線預設 SEG_DIR/road_angle.csv；即時預設不寫）")
    p.add_argument("--limit", type=int, help="只處理前 N 幀（快速測試/限制即時幀數）")
    p.add_argument("--quiet", action="store_true", help="不要逐幀印出")
    return p


def _row(r: FrameResult) -> dict:
    return {
        "index": r.index,
        "pitch_deg": _fmt(r.pitch_deg),
        "roll_deg": _fmt(r.roll_deg),
        "cam_height_m": _fmt(r.cam_height_m, 3),
        "n_inliers": r.n_inliers,
        "n_road_points": r.n_road_points,
        "rms_m": _fmt(r.rms_m, 4),
        "time_diff_ms": _fmt(r.time_diff_ms, 2),
    }


def _consume(
    results: Iterator[FrameResult],
    *,
    writer: csv.DictWriter | None,
    quiet: bool,
    limit: int | None,
    show_fps: bool,
) -> list[float]:
    """走訪結果：寫 CSV(可選)、逐幀印出、回傳成功的 pitch 列表。"""
    pitches: list[float] = []
    n_ok = n_fail = 0
    t0 = time.monotonic()
    for r in results:
        if limit is not None and r.index >= limit:
            break
        if writer is not None:
            writer.writerow(_row(r))
        if r.pitch_deg is None:
            n_fail += 1
            status = "  --  (路面擬合失敗)"
        else:
            n_ok += 1
            pitches.append(r.pitch_deg)
            status = (
                f"pitch={r.pitch_deg:+6.2f}° roll={r.roll_deg:+5.2f}° "
                f"h={r.cam_height_m:.2f}m inl={r.n_inliers}"
            )
        if not quiet:
            fps = ""
            if show_fps:
                el = time.monotonic() - t0
                fps = f" [{(r.index + 1) / el:4.1f}fps]" if el > 0 else ""
            print(f"frame {r.index:5d}{fps}  {status}")
    print(f"\n完成：{n_ok} 幀成功、{n_fail} 幀失敗")
    return pitches


def _print_summary(pitches: list[float]) -> None:
    if not pitches:
        return
    print(
        f"縱向坡度(相機相對) 中位={statistics.median(pitches):+.2f}° "
        f"平均={statistics.fmean(pitches):+.2f}° "
        f"範圍=[{min(pitches):+.2f}°, {max(pitches):+.2f}°]"
    )
    print("註：這是相對相機光軸的坡度。要相對水平面請用 IMU pitch 修正 (見 roadplane.to_gravity_referenced)。")


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    calib = load_calibration(args.calib, process_scale=DEFAULT.process_scale)
    print(
        f"calib: baseline={calib.baseline_mm:.1f}mm, "
        f"reproj_error={calib.reproj_error:.3f}px, "
        f"native={calib.native_size[0]}x{calib.native_size[1]} "
        f"→ process={calib.process_size[0]}x{calib.process_size[1]}"
    )

    # 介面開關：CLI --ui/--no-ui 優先，否則用 config.show_ui
    show_ui = DEFAULT.show_ui if args.ui is None else args.ui
    record = DEFAULT.record  # 錄影一律開啟（config.record）

    if args.live:
        if show_ui:
            from .ui import run_ui

            print("開啟 PyQt 視覺介面。")
            return run_ui(calib, DEFAULT, record=record)
        return _run_live(calib, args, record=record)
    return _run_segment(calib, args)


def _run_segment(calib, args) -> int:
    if not args.seg_dir and not (args.cam0 and args.cam1):
        print("錯誤：需要 SEG_DIR，或同時給 --cam0 與 --cam1（或用 --live 接鏡頭）。", file=sys.stderr)
        return 2

    seg_dir = args.seg_dir or "."
    out_path = Path(args.out) if args.out else Path(seg_dir) / "road_angle.csv"
    results = process_segment(
        calib, seg_dir, config=DEFAULT, cam0_mp4=args.cam0, cam1_mp4=args.cam1
    )
    with _open_csv(out_path) as (f, writer):
        pitches = _consume(
            results, writer=writer, quiet=args.quiet, limit=args.limit, show_fps=False
        )
    print(f"CSV → {out_path}")
    _print_summary(pitches)
    return 0


def _run_live(calib, args, *, record: bool = False) -> int:
    print("即時模式：接兩顆鏡頭，Ctrl+C 停止。")
    recorder = None
    if record:
        from .recorder import SessionRecorder

        recorder = SessionRecorder(
            DEFAULT.output_dir, calib.native_size, DEFAULT.record_fps, DEFAULT.segment_seconds
        )
        print(f"錄影中 → {recorder.dir}（每 {DEFAULT.segment_seconds:.0f} 秒收一段）")
    from .roi_store import load_roi  # headless 沿用 UI 記住的 ROI（roi.json）

    roi = load_roi(DEFAULT.roi_path, calib.process_size)
    if roi is not None:
        print(f"套用記住的 ROI {roi}（來自 {DEFAULT.roi_path}；UI 拉框設定/雙擊清除）")
    results = process_live(
        calib, config=DEFAULT, max_frames=args.limit, recorder=recorder, roi=roi
    )
    writer_ctx = _open_csv(Path(args.out)) if args.out else _null_csv()
    pitches: list[float] = []
    try:
        with writer_ctx as (f, writer):
            pitches = _consume(
                results, writer=writer, quiet=args.quiet, limit=None, show_fps=True
            )
    except KeyboardInterrupt:
        print("\n已停止 (Ctrl+C)。")
    finally:
        if recorder is not None:
            recorder.close()
            n = len(recorder.segments)
            print(f"已存 {n} 段 → {DEFAULT.output_dir}/（每段含 cam0/cam1.mp4, road_angle.csv, road_angle_trend.png）")
    if args.out:
        print(f"CSV → {args.out}")
    _print_summary(pitches)
    return 0


# --- CSV context helpers ---
class _open_csv:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._f: TextIO | None = None

    def __enter__(self):
        self._f = open(self.path, "w", newline="")
        writer = csv.DictWriter(self._f, fieldnames=_CSV_FIELDS)
        writer.writeheader()
        return self._f, writer

    def __exit__(self, *exc) -> None:
        if self._f:
            self._f.close()


class _null_csv:
    def __enter__(self):
        return None, None

    def __exit__(self, *exc) -> None:
        pass


def _fmt(v: float | None, ndigits: int = 2) -> str:
    return "" if v is None else f"{v:.{ndigits}f}"


if __name__ == "__main__":
    raise SystemExit(main())
