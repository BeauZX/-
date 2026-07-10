"""把 cam0.mp4 / cam1.mp4 配成「同一瞬間」的左右幀對，供立體幾何使用。

要算深度/路面角度，每個時間點都需要同一瞬間的左右兩張圖。這支負責產生那些
幀對，並自動選精度：

  - 資料夾裡有 cam0_pts.txt / cam1_pts.txt / start_time.json → 用「時間戳配對」：
    把兩顆鏡頭各自的幀時間換算成絕對 Unix 時間，用「時間最近」配對。掉幀時仍能
    自動對回正確的左右對應（硬體幀同步下中位時間差約 3-4ms）。
  - 只有兩支 mp4 → 退回「序號配對」（第 i 幀對第 i 幀）並警告：只要有一邊掉幀，
    之後會永遠錯開一幀而不自知，僅適合快速看結果。
"""

from __future__ import annotations

import bisect
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import av
import numpy as np


@dataclass
class FramePair:
    index: int  # cam0 幀序（0-based）
    img0: np.ndarray  # BGR，左（cam0）
    img1: np.ndarray  # BGR，右（cam1）
    time_diff_ms: float | None  # cam1 相對 cam0 的時間差；序號配對時為 None


def _load_sorted_pts(pts_path: Path, start_unix: float) -> list[float]:
    times: list[float] = []
    with open(pts_path) as f:
        for line in f:
            line = line.strip()
            try:
                ms = float(line)  # 第一行 "# timecode format v2" 會在這被跳過
            except ValueError:
                continue
            times.append(start_unix + ms / 1000.0)
    times.sort()  # pts 行序是編碼輸出序(可能被 B-frame 重排)，一定要排成時間序
    return times


def _nearest_idx(sorted_times: list[float], t: float) -> int:
    i = bisect.bisect_left(sorted_times, t)
    candidates = [j for j in (i - 1, i) if 0 <= j < len(sorted_times)]
    return min(candidates, key=lambda j: abs(sorted_times[j] - t))


def _decode_all(mp4: Path) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    with av.open(str(mp4)) as c:
        for frame in c.decode(video=0):
            frames.append(frame.to_ndarray(format="bgr24"))
    return frames


def iter_pairs(
    seg_dir: str | Path,
    *,
    cam0_mp4: str | Path | None = None,
    cam1_mp4: str | Path | None = None,
) -> Iterator[FramePair]:
    """走訪一段錄影的左右幀對，自動選時間戳配對或序號配對。

    seg_dir：段落資料夾（預設從這裡找 camN.mp4 與時間戳檔）。
    cam0_mp4/cam1_mp4：可覆寫影片路徑（例如兩支 mp4 不在同一資料夾）。
    """
    seg = Path(seg_dir)
    p0 = Path(cam0_mp4) if cam0_mp4 else seg / "cam0.mp4"
    p1 = Path(cam1_mp4) if cam1_mp4 else seg / "cam1.mp4"

    frames0 = _decode_all(p0)
    frames1 = _decode_all(p1)

    pts0 = seg / "cam0_pts.txt"
    pts1 = seg / "cam1_pts.txt"
    stj = seg / "start_time.json"

    if pts0.exists() and pts1.exists() and stj.exists():
        info = json.loads(stj.read_text())
        # 用實際可解碼幀數截斷 pts（pts 行數偶爾比可解碼幀數多 1）
        t0 = _load_sorted_pts(pts0, info["cam0_start_unix"])[: len(frames0)]
        t1 = _load_sorted_pts(pts1, info["cam1_start_unix"])[: len(frames1)]
        for i0, tt in enumerate(t0):
            i1 = _nearest_idx(t1, tt)
            diff = (t1[i1] - tt) * 1000.0
            yield FramePair(i0, frames0[i0], frames1[i1], diff)
    else:
        print(
            "[pairing] 找不到 pts/start_time.json，退回序號配對(第 i 幀對第 i 幀)；"
            "掉幀不對稱時會錯位。",
            file=sys.stderr,
        )
        n = min(len(frames0), len(frames1))
        for i in range(n):
            yield FramePair(i, frames0[i], frames1[i], None)
