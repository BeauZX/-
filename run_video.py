"""用「錄好的影片」跑路面坡度偵測——UI 跟即時鏡頭模式一模一樣，只是來源換成影片。

流程（跟即時 UI 相同的視窗與操作，差別在用空白鍵切出想要的路段）：
    1. 開視窗，影片循環播放 → 滑鼠拉框設定路面 ROI（雙擊清除，會記住）
    2. 按 Enter → 開始偵測（疊綠色路面 + 顯示前方坡度）
    3. 影片裡有回轉／不想要的片段，看著畫面；覺得「這段就是我要的路段」時按空白鍵
    4. 空白鍵 → 自動結束偵測，把 Enter 到空白鍵這段輸出成一個 segment：
         cam0.mp4 / cam1.mp4     原影片（這段的原始左右畫面）
         detect.mp4              偵測影片（疊了綠色路面 + 坡度文字的畫面）
         road_angle.csv          每幀角度
         road_angle_trend.png    坡度趨勢圖

用法：
    python3 run_video.py                       # 用下方寫死的預設影片，直接開視窗
    python3 run_video.py cam0.mp4 cam1.mp4      # 臨時指定別的影片（覆寫預設）
    python3 run_video.py --calib calib.npz

要換平常固定跑的影片，改下面 DEFAULT_CAM0 / DEFAULT_CAM1 兩個常數就好。
演算法參數一律固定在 src/config.py。輸出寫到 output_videos/segment_NNN/（跟即時錄影的
output/ 分開，不混淆）。
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

from src.calib_loader import load_calibration
from src.config import DEFAULT

# === 不給參數時預設要跑的兩支影片（改這裡就好）===
DEFAULT_CAM0 = "/home/beau/Desktop/output/20260713_162209/segment_005/cam0.mp4"
DEFAULT_CAM1 = "/home/beau/Desktop/output/20260713_162209/segment_005/cam1.mp4"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="run-video",
        description="用錄好的影片跑路面坡度偵測（UI 同即時模式，空白鍵切出想要的路段）。",
    )
    p.add_argument("cam0", nargs="?", help=f"cam0（左）影片路徑（不給用預設 {DEFAULT_CAM0}）")
    p.add_argument("cam1", nargs="?", help=f"cam1（右）影片路徑（不給用預設 {DEFAULT_CAM1}）")
    p.add_argument("--calib", default=DEFAULT.calib_path, help=f"calib.npz 路徑（預設 {DEFAULT.calib_path}）")
    args = p.parse_args(argv)

    if bool(args.cam0) != bool(args.cam1):
        print("錯誤：cam0 與 cam1 要嘛都給、要嘛都不給（不給就用預設路徑）。", file=sys.stderr)
        return 2
    cam0 = Path(args.cam0) if args.cam0 else Path(DEFAULT_CAM0)
    cam1 = Path(args.cam1) if args.cam1 else Path(DEFAULT_CAM1)
    for pth in (cam0, cam1):
        if not pth.is_file():
            print(f"錯誤：找不到影片 {pth}", file=sys.stderr)
            return 2

    calib = load_calibration(args.calib, process_scale=DEFAULT.process_scale)
    print(
        f"calib: baseline={calib.baseline_mm:.1f}mm, reproj_error={calib.reproj_error:.3f}px, "
        f"native={calib.native_size[0]}x{calib.native_size[1]} "
        f"→ process={calib.process_size[0]}x{calib.process_size[1]}"
    )

    # 影片模式的 config：不切段（segment_seconds=0，Enter→空白鍵整段當一個 segment、
    # 一張趨勢圖）、輸出另開 output_videos/ 跟即時錄影分開；use_imu=True 讓 video_ui
    # 去讀影片旁的 imu_raw.csv（若有）補算相對水平面坡度，沒有就自動退回純雙目。
    config = dataclasses.replace(
        DEFAULT, segment_seconds=0.0, output_dir="output_videos", use_imu=True
    )

    from src.video_ui import run_video_ui  # 影片專用 UI（跟即時 ui.py 分開）

    print("開啟視窗：拉框設 ROI → Enter 開始偵測 → 看到想要的路段按空白鍵結束並輸出。")
    return run_video_ui(calib, config, cam0, cam1)


if __name__ == "__main__":
    raise SystemExit(main())
