"""偵測疊圖：把一幀的校正影像 + 路面內點 + 坡度文字畫成一張並排 BGR 畫面。

**純 cv2/numpy，不依賴 PyQt** ——這是刻意的：即時 UI (`ui.py`)、headless 純 `--live`
(`pipeline.process_live`) 與測距工具都要用它，headless 不該為了畫個圖去 import Qt。
Qt 只在 `ui.py` 出現（`_to_qimage` 負責把這裡回傳的 BGR 轉成 QImage）。

這張圖同時是「螢幕上看到的畫面」與「detect.mp4 的一幀」——只畫一次、兩邊共用。
離線 `video_ui.py` 的偵測畫面文字刻意精簡（自己的 `_draw_slope`），不走這裡的
`draw_text`，但共用底下的 `as_bgr`/`compose_lr`/`panel_label`。

cv2 的 Hershey 字型畫不出中文：擬合失敗訊息會顯示成紅色 ??????（非當機，見 CLAUDE.md）。
"""

from __future__ import annotations

import cv2
import numpy as np

from .roadplane import plane_inlier_mask

DISPLAY_SCALE = 2  # 影像放大顯示倍率；滑鼠座標除以它換回 process 座標


def as_bgr(rect) -> np.ndarray:
    """校正影像轉可疊色的 BGR 副本（灰階→BGR，彩色則 copy 不動原圖）。"""
    img = rect.copy()
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def compose_lr(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """cam0(左)、cam1(右)水平並排，中間一條深灰分隔線。兩者同為 process_size。"""
    sep = np.full((left.shape[0], 4, 3), 60, np.uint8)
    return np.hstack([left, sep, right])


def panel_label(img: np.ndarray, text: str) -> None:
    """在單一 panel 左下角標 L/R 鏡頭來源（黑描邊 + 黃字）。"""
    y = img.shape[0] - 6
    cv2.putText(img, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
    cv2.putText(img, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)


def patch_depth_m(res, cx: int, cy: int, half: int = 8) -> float | None:
    """res 中心 (cx,cy) 附近小區塊的中位 Z（公尺，前向距離）；無有效點回 None。

    cx,cy 是 res 陣列的局部座標（整張算時＝process 座標；已裁 ROI 時傳中心 w//2,h//2）。
    取小區塊中位數避開單一像素的視差雜訊。
    """
    zs = res.pts[..., 2]
    valid = res.valid & np.isfinite(zs)
    h, w = zs.shape
    y0 = max(0, cy - half); y1 = min(h, cy + half + 1)
    x0 = max(0, cx - half); x1 = min(w, cx + half + 1)
    if y1 <= y0 or x1 <= x0:
        return None
    pz = zs[y0:y1, x0:x1][valid[y0:y1, x0:x1]]
    if pz.size == 0:
        return None
    return float(np.median(pz))


def draw_text(img, plane, fr, fps: float, z_center: float | None = None) -> None:
    """畫面只留主結果 + 一盞 sanity 燈，其餘（pitch/RMS/inliers/fps/imu）不上螢幕、
    全留在 road_angle.csv（跟離線 video_ui._draw_slope 一致）：

        slope <大字>        前方路面相對水平面的坡度（雙目+IMU；無 IMU 退回純雙目 pitch）
        h .. m   roll ..    相機估計高度 + 橫向坡度（確認擬到的是路面、不是牆）
        ROI ~ X.X m         ROI 中心的前向距離（黃字），驗證框住物件的距離用；不論有無平面都畫

    文字顏色沿用可信度：RMS 小且內點比例高→綠、否則橘（隱含 RMS/內點，不再列數字）。
    註：距離用 process_scale（預設 0.5）算，求快犧牲精度；要準的誤差驗證用 tool/measure_distance.py。"""
    # ROI 中心距離獨立於平面：框物件時擬不出路面平面，但距離讀數照樣要能驗證
    if z_center is not None:
        for w, c in ((4, (0, 0, 0)), (2, (0, 255, 255))):
            cv2.putText(img, f"ROI ~ {z_center:.1f} m", (10, 112),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, w)
    if plane is None:
        # 必須是英文：cv2 的 Hershey 字型沒有中文字形，中文會整串變成紅色 ??????
        for w, col in ((5, (0, 0, 0)), (2, (0, 0, 255))):
            cv2.putText(img, "NO ROAD PLANE", (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, w)
        return
    ratio = fr.n_inliers / fr.n_road_points if fr.n_road_points else 0.0
    good = plane.rms_m * 100 < 4.0 and ratio > 0.5
    col = (0, 255, 0) if good else (0, 165, 255)
    # 主結果：有 IMU → slope（相對水平）；沒有 → 退回純雙目 pitch（相機相對）
    head = (
        f"slope {fr.pitch_gravity_deg:+.1f}"
        if fr.pitch_gravity_deg is not None
        else f"pitch {fr.pitch_deg:+.1f}"
    )
    for w, c in ((6, (0, 0, 0)), (3, col)):  # 黑描邊 + 彩字，放大當標題
        cv2.putText(img, head, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 1.2, c, w)
    sub = f"h {fr.cam_height_m:.2f}m   roll {fr.roll_deg:+.1f}"
    for w, c in ((4, (0, 0, 0)), (2, col)):
        cv2.putText(img, sub, (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, w)


def annotate_frame(rect0, rect1, res, plane, fr, config, fps: float = 0.0) -> np.ndarray:
    """一幀 → 疊好圖的 BGR 畫面（cam0 左 / cam1 右並排、放大 DISPLAY_SCALE 倍）。

    綠色路面內點只疊在左圖(cam0)：mask 是 cam0 像素座標，右圖同像素被視差平移，
    塗上去會錯位。黃色 ROI 框兩顆都畫（強調視差是左右一起算的）。
    """
    left = as_bgr(rect0)
    right = as_bgr(rect1)
    x0, y0 = res.x0, res.y0
    h_roi, w_roi = res.pts.shape[:2]

    if plane is not None:  # 綠色：路面內點（＝角度的依據，越乾淨越可信）
        mask = plane_inlier_mask(res.pts, res.valid, plane, config.ransac_threshold_m)
        sub = left[y0 : y0 + h_roi, x0 : x0 + w_roi]
        sub[mask] = (0.4 * sub[mask] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)

    cv2.rectangle(left, (x0, y0), (x0 + w_roi, y0 + h_roi), (0, 255, 255), 1)
    cv2.rectangle(right, (x0, y0), (x0 + w_roi, y0 + h_roi), (0, 255, 255), 1)
    panel_label(left, "L - cam0 (ref)")
    panel_label(right, "R - cam1")

    # 並排合成後再放大（左圖仍起於 x=0，滑鼠 ROI 對映不變）
    combo = compose_lr(left, right)
    big = cv2.resize(
        combo, (combo.shape[1] * DISPLAY_SCALE, combo.shape[0] * DISPLAY_SCALE),
        interpolation=cv2.INTER_NEAREST,
    )
    # ROI 中心距離：重用已算好的 res（零額外成本）
    draw_text(big, plane, fr, fps, patch_depth_m(res, w_roi // 2, h_roi // 2))
    return big
