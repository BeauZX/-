"""從雙目 3D 點雲估計前方路面的縱向坡度 (pitch)。

座標系採 OpenCV 相機慣例（reprojectImageTo3D 的輸出即此系）：
    X → 右, Y → 下, Z → 前（光軸方向，射向場景）。

路面在相機下方、往前延伸，Y 在路面上是單值函數，所以用線性平面模型
    Y = a*Z + b*X + c
擬合最穩定（不像一般三參數平面法向量在近似水平時病態）。

由此得到的坡度都是「相對相機光軸」的：
    縱向坡度 pitch = atan(-a)   # 路面每前進 1 單位上升多少；上坡為正
    橫向坡度 roll  = atan(-b)   # 右側較高為正
（Y 向下，所以「上升」= Y 變小，故取負號。）

要換成「相對水平面（重力）」的真實坡度，再用 IMU 的相機 pitch 修正，
見 to_gravity_referenced()。IMU 只是輔助，主結果是純雙目的相機相對坡度。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class RoadPlane:
    """一次路面平面擬合的結果（單位：公尺、度）。"""

    a: float  # dY/dZ
    b: float  # dY/dX
    c: float  # 截距 (m)：Z=0,X=0 處的路面高度（負值＝在相機下方）
    pitch_deg: float  # 縱向坡度（相機相對），上坡為正
    roll_deg: float  # 橫向坡度（相機相對），右高為正
    n_inliers: int
    n_points: int
    rms_m: float  # 內點殘差 RMS (m)
    cam_height_m: float  # 相機離路面的估計高度 (m)，= -c（僅 c<0 時有意義）

    @property
    def inlier_ratio(self) -> float:
        return self.n_inliers / self.n_points if self.n_points else 0.0


def select_road_points(
    points_xyz: np.ndarray,
    valid_mask: np.ndarray | None = None,
    *,
    z_min: float = 0.5,
    z_max: float = 30.0,
    y_min: float = -0.5,
    image_bottom_fraction: float = 0.45,
) -> np.ndarray:
    """從整張 (H,W,3) 的 3D 點圖挑出可能屬於路面的點。

    篩選條件（都很寬鬆，真正剔除非路面點交給 RANSAC）：
      - valid_mask 為 True（視差有效）
      - z_min <= Z <= z_max：合理的前方距離
      - Y >= y_min：路面在相機下方（Y 向下為正），排除天空/高處
      - 只取影像下半部（image_bottom_fraction）：地平線以下才可能是路面

    回傳 (N,3) 的點陣列（公尺）。
    """
    h, w = points_xyz.shape[:2]
    xs = points_xyz[..., 0]
    ys = points_xyz[..., 1]
    zs = points_xyz[..., 2]

    mask = np.isfinite(zs) & np.isfinite(xs) & np.isfinite(ys)
    if valid_mask is not None:
        mask &= valid_mask
    mask &= (zs >= z_min) & (zs <= z_max)
    mask &= ys >= y_min

    if image_bottom_fraction < 1.0:
        row_cut = int(h * (1.0 - image_bottom_fraction))
        row_mask = np.zeros((h, w), dtype=bool)
        row_mask[row_cut:, :] = True
        mask &= row_mask

    return points_xyz[mask].reshape(-1, 3)


def fit_road_plane(
    pts: np.ndarray,
    *,
    threshold_m: float = 0.05,
    iterations: int = 300,
    min_inliers: int = 200,
    rng: np.random.Generator | None = None,
) -> RoadPlane | None:
    """對路面點雲用 RANSAC 擬合 Y = a*Z + b*X + c，回傳坡度。

    threshold_m：判定內點的垂直（Y 方向）殘差門檻，預設 5cm。
    pts：(N,3) 公尺。點太少或找不到夠大的內點集合時回傳 None。
    """
    n = len(pts)
    if n < max(3, min_inliers):
        return None
    if rng is None:
        rng = np.random.default_rng(0)

    X = pts[:, 0]
    Y = pts[:, 1]
    Z = pts[:, 2]
    # 設計矩陣 [Z, X, 1] 對應參數 [a, b, c]
    A = np.column_stack((Z, X, np.ones(n)))

    best_inliers: np.ndarray | None = None
    best_count = 0
    for _ in range(iterations):
        idx = rng.choice(n, size=3, replace=False)
        try:
            coef = np.linalg.solve(A[idx], Y[idx])
        except np.linalg.LinAlgError:
            continue
        resid = np.abs(A @ coef - Y)
        inliers = resid < threshold_m
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < min_inliers:
        return None

    # 用全部內點做最小二乘精修
    Ai = A[best_inliers]
    Yi = Y[best_inliers]
    coef, *_ = np.linalg.lstsq(Ai, Yi, rcond=None)
    a, b, c = (float(v) for v in coef)

    resid = Ai @ coef - Yi
    rms = float(np.sqrt(np.mean(resid**2)))

    return RoadPlane(
        a=a,
        b=b,
        c=c,
        pitch_deg=math.degrees(math.atan(-a)),
        roll_deg=math.degrees(math.atan(-b)),
        n_inliers=best_count,
        n_points=n,
        rms_m=rms,
        cam_height_m=-c,
    )


def plane_inlier_mask(
    points_xyz: np.ndarray,
    valid_mask: np.ndarray,
    plane: RoadPlane,
    threshold_m: float,
) -> np.ndarray:
    """回傳 (H,W) 布林遮罩：哪些有效像素落在這片路面平面上（供視覺化塗色）。

    這就是「角度的依據」——被塗出來的像素正是拿去擬合坡度的那片路面。
    """
    X = points_xyz[..., 0]
    Y = points_xyz[..., 1]
    Z = points_xyz[..., 2]
    # 無效像素帶 inf，運算會冒 invalid-value warning；用 errstate 靜音，
    # 最後 valid_mask & isfinite 會濾掉這些點，結果不受影響。
    with np.errstate(invalid="ignore"):
        resid = np.abs(plane.a * Z + plane.b * X + plane.c - Y)
    return valid_mask & np.isfinite(resid) & (resid < threshold_m)


def to_gravity_referenced(plane: RoadPlane, imu_pitch_deg: float) -> float:
    """把相機相對的路面 pitch 換成相對水平面（重力）的真實縱向坡度。

    imu_pitch_deg：相機自身相對水平面的 pitch（低頭為負、抬頭為正，
    跟 build_sync_csv 互補濾波輸出的 pitch 同慣例）。

    真實路面坡度 = 相機相對坡度 + 相機自身抬頭角。
    （相機低頭看地面時，平坦路面在相機座標裡呈現為往前上升＝正的相機相對
    pitch；把相機低頭角補回去才是路面對地面的真實坡度。IMU 只是輔助修正。）
    """
    return plane.pitch_deg + imu_pitch_deg
