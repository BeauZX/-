"""讀錄影時同步記錄的 imu_raw.csv，對齊到每個影片幀，供離線影片補算相對水平坡度。

給 run_video.py 的離線影片 UI 用：影片旁若有 imu_raw.csv（上層錄影程式錄影時
同步產生），就能把純雙目的「相機相對坡度」用當時記錄的相機 pitch 補成「相對
水平面」的真實坡度（pitch_gravity_deg）。旁邊沒有這個檔就回 None，管線退回純
雙目、行為完全不變。

跟即時的 src/imu.py（讀現在插著的實體 ICM20948）不同：離線影片是「別的時間錄
的」，當下的感測器讀值用不上，必須讀當初錄影一起存下來的那份 CSV。

pitch 值直接取自 imu_raw.csv 的 `pitch_deg` 欄——那是上層錄影程式已經互補濾波
＋套過零點的「車輛俯仰」（非生值），正負號與即時 imu.py 一致（都 atan2(ay,az)
+gx），故這裡**不再套** config 的 imu_invert_pitch / imu_mount_pitch_offset_deg
（那組偏移是給「生 atan2」歸零用的，重複套會雙重扣）。錄影程式零點與本專案零點
約差 1~2°，這個殘餘的安裝俯角之後由「平地校零」一併吸收（見 CLAUDE.md 零點偏移
校正）。
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

from .config import Config
from .pairing import _load_sorted_pts, _nearest_idx


def _read_imu_csv(path: Path) -> tuple[list[float], list[float]]:
    """讀 imu_raw.csv 的 (imu_timestamp, pitch_deg)，按時間排序回傳。

    沒有 pitch_deg 欄（例如欄位還沒改名）時回傳兩個空 list。
    """
    ts: list[float] = []
    pitch: list[float] = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "pitch_deg" not in reader.fieldnames:
            return [], []
        for row in reader:
            try:
                t = float(row["imu_timestamp"])
                p = float(row["pitch_deg"])
            except (TypeError, ValueError):
                continue
            ts.append(t)
            pitch.append(p)
    # imu_timestamp 本就遞增寫入，保險起見仍與 pitch 綁在一起按時間排序。
    order = sorted(range(len(ts)), key=lambda i: ts[i])
    return [ts[i] for i in order], [pitch[i] for i in order]


def _frame_count(mp4: Path) -> int:
    """cam0.mp4 的總幀數（只在沒有 pts 時的退化備援用）；讀不到回 0。"""
    import cv2

    cap = cv2.VideoCapture(str(mp4))
    try:
        return int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    finally:
        cap.release()


class ImuTrack:
    """錄影期間記錄的逐幀相機 pitch（相對水平，抬頭為正），對齊到 cam0 幀序。"""

    def __init__(self, per_frame_pitch: list[float]) -> None:
        self._pitch = per_frame_pitch  # index = cam0 幀序（0-based）

    def __len__(self) -> int:
        return len(self._pitch)

    def pitch_for_frame(self, index: int) -> float | None:
        """第 index 幀（cam0 幀序）對應的相機 pitch；loop 播放時取模循環對應。"""
        if not self._pitch:
            return None
        return self._pitch[index % len(self._pitch)]

    @classmethod
    def load(cls, cam0_path, config: Config) -> "ImuTrack | None":
        """影片旁若有 imu_raw.csv 就載入並對齊；沒有/讀不到回 None（退回純雙目）。

        對齊優先用 pts + start_time.json 對絕對時間（跟 pairing.py 同一套）；缺這
        兩個檔時退化成「IMU 與影片等時間跨度」的序號等比例對應（精度較差）。
        """
        cam0 = Path(cam0_path)
        csv_path = cam0.parent / "imu_raw.csv"
        if not csv_path.is_file():
            return None
        ts, pitch = _read_imu_csv(csv_path)
        if not ts:
            print(f"[imu_track] {csv_path} 沒有可用的 pitch_deg 欄，跳過 IMU 修正。")
            return None

        pts_path = cam0.parent / f"{cam0.stem}_pts.txt"
        stj = cam0.parent / "start_time.json"
        if pts_path.is_file() and stj.is_file():
            info = json.loads(stj.read_text())
            key = "cam1_start_unix" if "cam1" in cam0.stem else "cam0_start_unix"
            start_unix = info.get(key, info.get("cam0_start_unix"))
            frame_times = _load_sorted_pts(pts_path, start_unix)
            per_frame = [pitch[_nearest_idx(ts, t)] for t in frame_times]
            print(
                f"[imu_track] 已載入 {csv_path.name}：{len(ts)} 筆 IMU → "
                f"時間戳對齊 {len(per_frame)} 幀。"
            )
        else:
            n_frames = _frame_count(cam0)
            if n_frames <= 0:
                per_frame = list(pitch)
            else:
                per_frame = [
                    pitch[round((i / max(1, n_frames - 1)) * (len(pitch) - 1))]
                    for i in range(n_frames)
                ]
            print(
                f"[imu_track] {csv_path.name} 無 pts/start_time，改用序號等比例對應 "
                f"{len(per_frame)} 幀（精度較差）。"
            )
        return cls(per_frame)
