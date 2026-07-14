"""即時錄影 + 逐幀角度的 session 輸出。

每次啟動在 output/ 底下開一個「遞增編號、不覆蓋」的資料夾 segment_NNN，內含：
    cam0.mp4 / cam1.mp4   原生解析度的左右影像（可再拿去離線重跑）
    road_angle.csv        每幀角度 + 時間戳
    road_angle_trend.png  pitch/roll 隨時間的趨勢圖

影片用固定名義 fps 寫入（實際擷取是變動 fps），播放速度為近似值；要精準時間對齊
可用 CSV 裡的 time_s 欄。
"""

from __future__ import annotations

import csv
import time
from pathlib import Path

import cv2
import numpy as np

from .pipeline import FrameResult
from .plot import save_angle_trend

_CSV_FIELDS = [
    "index", "time_s", "pitch_deg", "roll_deg", "cam_height_m",
    "n_inliers", "n_road_points", "rms_m",
    "imu_pitch_deg", "pitch_gravity_deg",  # 只有 use_imu 開啟時才有值，否則空
]


def next_segment_dir(output_dir: str | Path) -> Path:
    """在 output_dir 底下找下一個未使用的 segment_NNN 並建立它（不覆蓋既有）。"""
    base = Path(output_dir)
    base.mkdir(parents=True, exist_ok=True)
    n = 0
    while (base / f"segment_{n:03d}").exists():
        n += 1
    seg = base / f"segment_{n:03d}"
    seg.mkdir()
    return seg


class SessionRecorder:
    """錄 cam0/cam1 影片、累積每幀結果，輸出 CSV + 趨勢圖。

    segment_seconds > 0 時每滿這麼多秒就把當前 segment 收好、換下一段遞增編號
    （output/segment_NNN/），跑越久也不怕中途掛掉。設 0 = 不輪替、只在 close()
    收尾（舊行為）。每段的 time_s 都從該段開始 0 重新計。
    """

    def __init__(
        self,
        output_dir: str | Path,
        size: tuple[int, int],
        fps: float,
        segment_seconds: float = 0.0,
    ) -> None:
        self.output_dir = output_dir
        self.size = size
        self.fps = fps
        self.segment_seconds = segment_seconds
        self.segments: list[Path] = []
        self._open_segment()

    def _open_segment(self) -> None:
        self.dir = next_segment_dir(self.output_dir)
        self.segments.append(self.dir)
        w, h = self.size
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.vw0 = cv2.VideoWriter(str(self.dir / "cam0.mp4"), fourcc, self.fps, (w, h))
        self.vw1 = cv2.VideoWriter(str(self.dir / "cam1.mp4"), fourcc, self.fps, (w, h))
        self._results: list[FrameResult] = []
        self._times: list[float] = []
        self._t0 = time.monotonic()

    def _finalize_segment(self) -> None:
        self.vw0.release()
        self.vw1.release()
        self._write_csv()
        self._write_trend()

    def add(self, img0: np.ndarray, img1: np.ndarray, fr: FrameResult) -> None:
        # 當前 segment 滿了就先收好、再開新的一段（下面這幀寫進新段）。
        if self.segment_seconds > 0 and (time.monotonic() - self._t0) >= self.segment_seconds:
            self._finalize_segment()
            self._open_segment()
        self.vw0.write(img0)
        self.vw1.write(img1)
        self._results.append(fr)
        self._times.append(time.monotonic() - self._t0)

    def close(self) -> Path:
        self._finalize_segment()
        return self.dir

    def _write_csv(self) -> None:
        with open(self.dir / "road_angle.csv", "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
            wr.writeheader()
            for t, r in zip(self._times, self._results):
                wr.writerow({
                    "index": r.index,
                    "time_s": f"{t:.3f}",
                    "pitch_deg": _fmt(r.pitch_deg),
                    "roll_deg": _fmt(r.roll_deg),
                    "cam_height_m": _fmt(r.cam_height_m, 3),
                    "n_inliers": r.n_inliers,
                    "n_road_points": r.n_road_points,
                    "rms_m": _fmt(r.rms_m, 4),
                    "imu_pitch_deg": _fmt(r.imu_pitch_deg),
                    "pitch_gravity_deg": _fmt(r.pitch_gravity_deg),
                })

    def _write_trend(self) -> None:
        idx = [r.index for r in self._results]
        pit = [r.pitch_deg for r in self._results]
        rol = [r.roll_deg for r in self._results]
        save_angle_trend(str(self.dir / "road_angle_trend.png"), idx, pit, rol)


def _fmt(v: float | None, ndigits: int = 2) -> str:
    return "" if v is None else f"{v:.{ndigits}f}"
