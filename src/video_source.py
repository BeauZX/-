"""離線雙目來源：把 cam0.mp4 + cam1.mp4 當成 UI 的來源，介面跟 LiveStereo 一樣。

給 UI 用（run_video.py）。跟 live.py 的 LiveStereo 一樣是 context manager，`frames()`
吐 native 尺寸的 (cam0_bgr, cam1_bgr) 幀對，所以 StereoWorker 只要換這個來源、其他
（rectify → 視差 → 擬平面 → 塗綠 → 算角度、滑鼠圈 ROI、Enter 閘門）完全不變。

差別：
  - 幀對用「序號配對」（第 i 幀對第 i 幀）——UI 互動預覽夠用；要掉幀也對得回的精準
    時間戳配對走離線 CSV 路徑（pairing.iter_pairs）。
  - 預設 `loop=True` 循環播放：影片放完自動從頭再來，這樣「框選階段」永遠有畫面在動、
    你有充裕時間圈 ROI，跟鏡頭「無限吐幀」的行為一致。
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator

import av
import cv2
import numpy as np


class VideoStereo:
    """context manager：把兩支 mp4 當來源，逐幀吐 native 尺寸的 (cam0, cam1) BGR 幀對。

    with VideoStereo(cam0_path, cam1_path, size=(1280,720)) as cams:
        for img0, img1 in cams.frames():
            ...
    """

    def __init__(
        self,
        cam0: str | Path,
        cam1: str | Path,
        size: tuple[int, int] = (1280, 720),
        loop: bool = True,
    ) -> None:
        self.cam0 = str(cam0)
        self.cam1 = str(cam1)
        self.size = size  # (width, height)，須等於 calib.native_size
        self.loop = loop

    def __enter__(self) -> "VideoStereo":
        print(f"[video] cam0={self.cam0}  cam1={self.cam1}  @ {self.size[0]}x{self.size[1]}"
              f"{'（循環播放）' if self.loop else ''}")
        return self

    def first_frame(self) -> tuple[np.ndarray, np.ndarray]:
        """只解一張：兩支影片各自的第一幀（native 尺寸）。給框選階段當定格底圖用。"""
        with av.open(self.cam0) as c0, av.open(self.cam1) as c1:
            f0 = next(c0.decode(video=0))
            f1 = next(c1.decode(video=0))
            return self._fit(f0.to_ndarray(format="bgr24")), self._fit(f1.to_ndarray(format="bgr24"))

    def frames(self) -> Iterator[tuple[np.ndarray, np.ndarray]]:
        """逐幀吐 (cam0_bgr, cam1_bgr)（序號配對）。loop=True 時放完自動從頭。

        用串流解碼（不一次載入整支影片），開視窗不會卡住、記憶體省。
        """
        while True:
            with av.open(self.cam0) as c0, av.open(self.cam1) as c1:
                for f0, f1 in zip(c0.decode(video=0), c1.decode(video=0)):
                    yield (
                        self._fit(f0.to_ndarray(format="bgr24")),
                        self._fit(f1.to_ndarray(format="bgr24")),
                    )
            if not self.loop:
                break

    def _fit(self, img: np.ndarray) -> np.ndarray:
        """保險：影片解析度若跟 native_size 不符就縮放（rectify 需要 native 尺寸）。"""
        if (img.shape[1], img.shape[0]) != self.size:
            img = cv2.resize(img, self.size, interpolation=cv2.INTER_AREA)
        return img

    def __exit__(self, *exc) -> None:
        pass
