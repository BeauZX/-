"""校正後左右影像 → 視差圖 → 相機座標系 3D 點雲（公尺）。

用 StereoSGBM 算視差（以 cam0 校正影像為左基準），再 reprojectImageTo3D(Q)
還原成 3D。calib 的 T 單位是 mm，所以 reproject 出來也是 mm，這裡統一 ÷1000
轉成公尺後交給下游 roadplane。
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .calib_loader import StereoCalibration

# reprojectImageTo3D 對無效視差會給出極大值，用這個門檻濾掉
_MISSING = 1e4  # m


@dataclass
class StereoMatcher:
    """包一顆 StereoSGBM，參數針對 1280x720、基線 ~62mm 的路面場景。

    num_disparities 必須是 16 的倍數。預設 128 對應最近可測約 0.7m
    (Z = f*baseline/disp = 1521*0.062/128 ≈ 0.74m)，涵蓋前方路面。
    """

    num_disparities: int = 128
    block_size: int = 5
    uniqueness_ratio: int = 10
    speckle_window_size: int = 100
    speckle_range: int = 2
    depth_scale: float = 1000.0  # mm → m

    roi_fraction: float = 1.0  # 只在校正影像下面這個比例的橫帶上算視差

    @classmethod
    def from_config(cls, config) -> "StereoMatcher":
        """用 config.Config 的固定參數建 matcher。

        num_disparities 以原生解析度定義，這裡依 process_scale 縮放並取 16 倍數
        （縮小影像裡視差也等比例變小）。
        """
        nd = int(round(config.num_disparities * config.process_scale / 16.0)) * 16
        nd = max(16, nd)
        return cls(
            num_disparities=nd,
            block_size=config.block_size,
            uniqueness_ratio=config.uniqueness_ratio,
            speckle_window_size=config.speckle_window_size,
            speckle_range=config.speckle_range,
            depth_scale=config.depth_scale,
            roi_fraction=config.disparity_roi_fraction,
        )

    def __post_init__(self) -> None:
        bs = self.block_size
        self._sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self.num_disparities,
            blockSize=bs,
            P1=8 * 3 * bs * bs,
            P2=32 * 3 * bs * bs,
            disp12MaxDiff=1,
            uniquenessRatio=self.uniqueness_ratio,
            speckleWindowSize=self.speckle_window_size,
            speckleRange=self.speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def disparity(self, rect0: np.ndarray, rect1: np.ndarray) -> np.ndarray:
        """回傳浮點視差圖（px）。無效處為負值（SGBM 慣例）。"""
        g0 = _to_gray(rect0)
        g1 = _to_gray(rect1)
        # SGBM 回傳 int16、放大 16 倍的定點數
        raw = self._sgbm.compute(g0, g1).astype(np.float32) / 16.0
        return raw

    def point_cloud(
        self, disp: np.ndarray, Q: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """視差 → (H,W,3) 3D 點雲（公尺）+ 有效遮罩。

        有效＝視差為正且深度落在合理範圍（0 < Z < _MISSING）。
        """
        pts = cv2.reprojectImageTo3D(disp, Q.astype(np.float32))
        pts = pts / self.depth_scale  # mm → m
        z = pts[..., 2]
        valid = (disp > 0.0) & np.isfinite(z) & (z > 0.0) & (z < _MISSING)
        return pts, valid


def _to_gray(img: np.ndarray) -> np.ndarray:
    if img.ndim == 3 and img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


@dataclass
class StereoResult:
    """一幀立體處理的中間+最終產物（供視覺化取用）。"""

    rect0_roi: np.ndarray  # 左影像的 ROI 區塊（process 解析度，BGR）
    disp: np.ndarray  # ROI 視差圖 (px)
    pts: np.ndarray  # (H_roi, W_roi, 3) 3D 點雲 (公尺)
    valid: np.ndarray  # 有效遮罩
    x0: int  # ROI 左緣在 process 影像的欄位置
    y0: int  # ROI 頂端在 process 影像的列位置


def compute_stereo(
    calib: StereoCalibration,
    matcher: StereoMatcher,
    img0: np.ndarray,
    img1: np.ndarray,
    roi: tuple[int, int, int, int] | None = None,
) -> StereoResult:
    """原始 cam0/cam1 → 校正 → (ROI)視差 → 3D 點雲，回傳含中間結果的 StereoResult。

    roi=(x0,y0,x1,y1)（process 影像座標）時只在該矩形內算視差（滑鼠拉框用）；
    None 時退回 matcher.roi_fraction 的「下方橫帶」。Q 的主點 cx/cy 依裁切位移，
    3D 幾何維持正確。
    """
    rect0, rect1 = calib.rectify(img0, img1)
    return stereo_from_rectified(rect0, rect1, calib.Q, matcher, roi)


def stereo_from_rectified(
    rect0: np.ndarray,
    rect1: np.ndarray,
    base_Q: np.ndarray,
    matcher: StereoMatcher,
    roi: tuple[int, int, int, int] | None = None,
) -> StereoResult:
    """已校正的左右影像 → (ROI)視差 → 3D 點雲。給 UI 用（整張只 rectify 一次）。

    base_Q 是對應整張 process 影像的 Q；裁 ROI 後這裡複製並位移 cx/cy。
    """
    ph, pw = rect0.shape[:2]
    if roi is not None:
        x0, y0, x1, y1 = roi
        x0 = max(0, min(int(x0), pw - 1))
        y0 = max(0, min(int(y0), ph - 1))
        x1 = max(x0 + 1, min(int(x1), pw))
        y1 = max(y0 + 1, min(int(y1), ph))
    else:
        x0, x1 = 0, pw
        y0 = int(ph * (1.0 - matcher.roi_fraction)) if matcher.roi_fraction < 1.0 else 0
        y1 = ph

    if (x0, y0, x1, y1) != (0, 0, pw, ph):
        rect0 = rect0[y0:y1, x0:x1]
        rect1 = rect1[y0:y1, x0:x1]
        Q = base_Q.copy()
        Q[0, 3] += x0  # 裁掉左邊 x0 欄 → Q[0,3]=-cx 補回 +x0
        Q[1, 3] += y0  # 裁掉頂端 y0 列 → Q[1,3]=-cy 補回 +y0
    else:
        Q = base_Q
    disp = matcher.disparity(rect0, rect1)
    pts, valid = matcher.point_cloud(disp, Q)
    return StereoResult(rect0_roi=rect0, disp=disp, pts=pts, valid=valid, x0=x0, y0=y0)


def stereo_to_points(
    calib: StereoCalibration,
    matcher: StereoMatcher,
    img0: np.ndarray,
    img1: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """完整一步：原始 cam0/cam1 → (ROI)視差 → 3D 點雲(公尺)+有效遮罩。"""
    res = compute_stereo(calib, matcher, img0, img1)
    return res.pts, res.valid
