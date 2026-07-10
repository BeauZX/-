"""載入器：讀 calib.npz（由上層 calibrate_stereo.py 產生），建出立體校正用的
remap 表與 Q。這支只「讀取」既有校正結果，不做棋盤格校正。

calib.npz 的 key（由 calibrate_stereo.py 的 np.savez 決定）：
    mtx0, dist0, mtx1, dist1   兩顆鏡頭內參 / 畸變
    R, T                       cam0→cam1 外參（T 單位＝校正時的 SQUARE_SIZE_MM，通常 mm）
    R1, R2, P1, P2, Q          stereoRectify 的輸出
    image_size                 [width, height]
    reproj_error               立體校正 RMS reprojection error

cam0 是參考/左影像（對應 P1），cam1 是右影像（對應 P2，Tx 為負）。
視差用左影像(cam0) 為基準計算，reprojectImageTo3D(disp, Q) 得到 cam0 校正座標系
的 3D 點，單位同 T（mm）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class StereoCalibration:
    """一組立體校正參數 + 預先算好的 remap 表。

    支援降解析度處理：rectify 的 remap 表 dest 是 process_size（可小於原生），
    但 map 值索引的來源仍是原生 native_size 影像——所以餵進 rectify 的原圖必須是
    native_size（相機/影片的原生解析度），remap 一步完成「去畸變+校正+縮小」。
    Q 已對應 process_size 的視差尺度。
    """

    map0x: np.ndarray
    map0y: np.ndarray
    map1x: np.ndarray
    map1y: np.ndarray
    Q: np.ndarray
    native_size: tuple[int, int]  # (w, h) 相機/影片必須提供的原生解析度
    process_size: tuple[int, int]  # (w, h) rectify 後、實際拿去算視差的解析度
    baseline_mm: float
    reproj_error: float
    process_scale: float

    # 相容舊名：image_size 指原生解析度
    @property
    def image_size(self) -> tuple[int, int]:
        return self.native_size

    def rectify(self, img0: np.ndarray, img1: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """原生 cam0/cam1 影像 → 校正後(process_size)的平行雙目影像。"""
        r0 = cv2.remap(img0, self.map0x, self.map0y, cv2.INTER_LINEAR)
        r1 = cv2.remap(img1, self.map1x, self.map1y, cv2.INTER_LINEAR)
        return r0, r1


_REQUIRED = ("mtx0", "dist0", "mtx1", "dist1", "R1", "R2", "P1", "P2", "Q", "image_size")


def load_calibration(path: str | Path, process_scale: float = 1.0) -> StereoCalibration:
    """載入 calib.npz，回傳可直接 rectify 的 StereoCalibration。

    process_scale < 1.0 時，rectify 直接輸出縮小後的校正影像（加速視差計算），
    P1/P2 各縮放 s、Q 對應縮放，幾何維持正確。缺 key 會明確報錯。
    """
    if not (0.0 < process_scale <= 1.0):
        raise ValueError(f"process_scale 需在 (0,1]，得到 {process_scale}")

    data = np.load(str(path), allow_pickle=True)
    have = set(data.files)
    missing = [k for k in _REQUIRED if k not in have]
    if missing:
        raise KeyError(
            f"calib.npz 缺少必要 key {missing}；實際有的 key：{sorted(have)}。"
            " 這個載入器對應 calibrate_stereo.py 的輸出格式。"
        )

    native = tuple(int(v) for v in data["image_size"])  # (w, h)
    if len(native) != 2:
        raise ValueError(f"image_size 應為 [width, height]，得到 {data['image_size']}")

    s = float(process_scale)
    pw, ph = round(native[0] * s), round(native[1] * s)
    process_size = (pw, ph)

    # P1/P2 的前兩列(fx,fy,cx,cy,Tx項) 縮放 s → 校正輸出直接是 process_size。
    # 來源相機矩陣仍用原生 mtx（原圖是原生解析度）。
    Sd = np.diag([s, s, 1.0])
    P1s = Sd @ np.asarray(data["P1"], dtype=np.float64)
    P2s = Sd @ np.asarray(data["P2"], dtype=np.float64)
    # Q 對應縮放：視差在縮小影像裡也縮 s，Q @ diag(1/s,1/s,1/s,1) 抵消回正確 3D。
    Qs = np.asarray(data["Q"], dtype=np.float64) @ np.diag([1 / s, 1 / s, 1 / s, 1.0])

    map0x, map0y = cv2.initUndistortRectifyMap(
        data["mtx0"], data["dist0"], data["R1"], P1s, process_size, cv2.CV_32FC1
    )
    map1x, map1y = cv2.initUndistortRectifyMap(
        data["mtx1"], data["dist1"], data["R2"], P2s, process_size, cv2.CV_32FC1
    )

    # 基線：優先用 T，沒有就從 P2 的 Tx 反推 (P2[0,3] = -f * baseline)
    if "T" in have:
        baseline = float(np.linalg.norm(data["T"]))
    else:
        baseline = abs(float(data["P2"][0, 3]) / float(data["P2"][0, 0]))

    reproj = float(data["reproj_error"]) if "reproj_error" in have else float("nan")

    return StereoCalibration(
        map0x=map0x,
        map0y=map0y,
        map1x=map1x,
        map1y=map1y,
        Q=Qs,
        native_size=(native[0], native[1]),
        process_size=process_size,
        baseline_mm=baseline,
        reproj_error=reproj,
        process_scale=s,
    )


def _summary(path: str) -> None:
    calib = load_calibration(path)
    nw, nh = calib.native_size
    pw, ph = calib.process_size
    print(f"calib: {path}")
    print(f"  native_size   : {nw} x {nh}")
    print(f"  process_size  : {pw} x {ph}  (scale={calib.process_scale})")
    print(f"  baseline      : {calib.baseline_mm:.2f} mm")
    print(f"  reproj_error  : {calib.reproj_error:.4f} px")
    print(f"  focal (Q[2,3]): {calib.Q[2, 3]:.2f}")
    print(f"  remap shape   : {calib.map0x.shape}")


if __name__ == "__main__":
    import sys

    _summary(sys.argv[1] if len(sys.argv) > 1 else "calib.npz")
