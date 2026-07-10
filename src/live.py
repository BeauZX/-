"""即時雙目來源：用 Picamera2 同時開兩顆 IMX219，吐出 (cam0, cam1) BGR 幀對。

只有 `--live` 模式才會 import 到這支（picamera2 只在 Pi 上才有）。

與離線最大的差異——**沒有硬體幀同步**：兩顆鏡頭是各自獨立的 Picamera2 實例，
這裡只能背靠背 capture_array()，左右幀之間會有幾毫秒的時間差。車速快時這會讓
視差略有誤差；靜止或慢速影響不大。要真正硬體同步得走 rpicam-vid --sync（錄影
用），即時串流目前沒有等價做法。

解析度強制用 calib 綁定的 image_size；兩顆吃同一組固定曝光（config.live_*），
AWB 維持 auto——跟 calibrate_stereo.py / 錄影程式一致，避免各自收斂到不同亮度。
"""

from __future__ import annotations

from typing import Iterator

import cv2
import numpy as np

from .config import Config


class LiveStereo:
    """context manager：進入時開兩顆鏡頭，離開時關閉。

    with LiveStereo(config, size=(1280,720)) as cams:
        for img0, img1 in cams.frames():
            ...
    """

    def __init__(self, config: Config, size: tuple[int, int] = (1280, 720)) -> None:
        self.config = config
        self.size = size  # (width, height)，須等於 calib.image_size
        self._cams: list = []

    def __enter__(self) -> "LiveStereo":
        from picamera2 import Picamera2

        controls = {
            "AeEnable": False,  # 關自動曝光，兩顆吃同一組固定值
            "ExposureTime": int(self.config.live_shutter_us),
            "AnalogueGain": float(self.config.live_gain),
            "AwbEnable": True,  # AWB 維持 auto（勿改固定色溫）
        }
        for idx in (self.config.live_cam0_index, self.config.live_cam1_index):
            cam = Picamera2(idx)
            cfg = cam.create_video_configuration(
                main={"size": self.size, "format": "RGB888"},
                controls=controls,
            )
            cam.configure(cfg)
            cam.start()
            self._cams.append(cam)
        info = Picamera2.global_camera_info()
        print(
            f"[live] cam0=index{self.config.live_cam0_index}"
            f"({info[self.config.live_cam0_index].get('Id','?')}) "
            f"cam1=index{self.config.live_cam1_index}"
            f"({info[self.config.live_cam1_index].get('Id','?')}) "
            f"@ {self.size[0]}x{self.size[1]}"
        )
        return self

    def frames(self) -> Iterator[tuple[np.ndarray, np.ndarray]]:
        """無限吐出 (cam0_bgr, cam1_bgr)。呼叫端自行決定何時停(Ctrl+C/max_frames)。"""
        cam0, cam1 = self._cams
        while True:
            # 背靠背擷取，盡量壓低左右時間差（非硬體同步，見模組說明）
            a = cam0.capture_array()
            b = cam1.capture_array()
            yield _to_bgr(a), _to_bgr(b)

    def __exit__(self, *exc) -> None:
        for cam in self._cams:
            try:
                cam.stop()
                cam.close()
            except Exception:
                pass
        self._cams = []


def _to_bgr(arr: np.ndarray) -> np.ndarray:
    """Picamera2 "RGB888" 主串流回傳的是 BGR 排列的 3 通道陣列，直接給 OpenCV 用。
    若拿到 4 通道(XBGR) 則去掉 alpha。"""
    if arr.ndim == 3 and arr.shape[2] == 4:
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    return arr
