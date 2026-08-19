"""即時視覺介面 (PyQt5)。

兩階段：開視窗先進「框選階段」（只顯示預覽 + 黃色 ROI 框，跳過 SGBM/擬合/錄影，
預覽更順好瞄準），把框調到滿意後按 **Enter** 才進入「預測階段」（開始算角度並錄影）。
（headless 純 `--live` 不經過本 worker、不需 Enter，一律直接跑。）

預測階段畫面＝cam0(左)與 cam1(右)兩顆校正後影像**並排**（讓人一眼看到雙目
都在跑，深度是左右一起算的），左圖(cam0=參考影像)疊上：
  - 綠色：被判定為路面、拿去算坡度的那片點（＝角度的依據，越乾淨越可信）
  - 黃框：目前的 ROI（只在框內算視差）
  - 文字：主結果 slope(有 IMU)/pitch(無)大字 + 一行 h/roll sanity；RMS/內點/fps 只進 CSV
右圖(cam1)只疊黃色 ROI 框：綠色內點是 cam0 像素座標，右圖同像素被視差平移，
畫上去會錯位，所以右圖單純呈現「另一顆鏡頭的即時校正畫面」。滑鼠拉框設 ROI
只在左圖(cam0)座標系生效。

滑鼠操作（兩階段都可用）：
  - 在影像上「拖一個方框」＝設定 ROI（只算框內，聚焦到路面），會存進 roi.json 記住
  - 「雙擊」＝清除 ROI，回到預設下方橫帶

運算在背景 QThread，GUI 只負責畫。由 CLI `--ui`（或 config.show_ui=True）啟動。
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
from PyQt5.QtCore import QRect, QSize, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QRubberBand,
    QVBoxLayout,
    QWidget,
)

from .calib_loader import StereoCalibration
from .config import Config
from .disparity import StereoMatcher, stereo_from_rectified
from .pipeline import FrameResult
from .roadplane import fit_road_plane, plane_inlier_mask, select_road_points
from .roi_store import load_roi, save_roi

DISPLAY_SCALE = 2  # 影像放大顯示倍率；滑鼠座標除以它換回 process 座標


class StereoWorker(QThread):
    """背景執行緒：開鏡頭、逐幀算坡度、發出疊好圖的畫面 + 結果。"""

    frameReady = pyqtSignal(QImage, object)
    sessionSaved = pyqtSignal(str)

    def __init__(self, calib: StereoCalibration, config: Config, record: bool = False) -> None:
        super().__init__()
        self.calib = calib
        self.config = config
        self.record = record
        self._running = True
        self.roi: tuple[int, int, int, int] | None = None  # process 座標，主執行緒設定
        # False＝框選階段（只顯示預覽+ROI 框，不算角度/不錄影）；主執行緒按 Enter 設 True 開始預測。
        # 只有 UI 用這個閘門；headless 的 process_live 不經過本 worker，一律直接跑。
        self.active = False

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        from .live import LiveStereo  # 延後 import，只有 UI 才需要 picamera2

        matcher = StereoMatcher.from_config(self.config)
        rng = np.random.default_rng(self.config.seed)
        recorder = None  # 延後到「開始預測」才建，避免框選階段就開一個空 segment
        t_prev = time.monotonic()
        fps = 0.0
        i = 0
        roi_z: float | None = None  # 框選階段 ROI 中心的前向距離（公尺），節流量測後快取
        roi_z_t = 0.0
        try:
            with LiveStereo(self.config, size=self.calib.native_size) as cams:
                for img0, img1 in cams.frames():
                    if not self._running:
                        break
                    rect0, rect1 = self.calib.rectify(img0, img1)

                    # 尚未按 Enter：只顯示預覽 + ROI 框（跳過 SGBM/擬合/錄影，預覽更順好瞄準）
                    if not self.active:
                        pw, ph = self.calib.process_size
                        rect = self._roi_rect(pw, ph, matcher.roi_fraction)
                        # 節流量測 ROI 中心距離：框選階段刻意不每幀跑 SGBM（保持預覽順、好瞄準），
                        # 只約每 0.3 秒對「當前 ROI 框」算一次視差取中心中位 Z，其餘幀沿用快取值。
                        now = time.monotonic()
                        if now - roi_z_t > 0.3:
                            roi_z_t = now
                            roi_z = self._roi_center_depth(rect0, rect1, matcher, rect)
                        self.frameReady.emit(self._draw_preview(rect0, rect1, rect, roi_z), None)
                        continue

                    # 第一幀 active 才建立錄影器（此後每 segment_seconds 自動輪替）
                    if self.record and recorder is None:
                        from .recorder import SessionRecorder

                        recorder = SessionRecorder(
                            self.config.output_dir,
                            self.calib.native_size,
                            self.config.record_fps,
                            self.config.segment_seconds,
                        )

                    res = stereo_from_rectified(rect0, rect1, self.calib.Q, matcher, self.roi)

                    road = select_road_points(
                        res.pts, res.valid,
                        z_min=self.config.z_min_m, z_max=self.config.z_max_m,
                        y_min=self.config.y_min_m, image_bottom_fraction=1.0,
                    )
                    plane = fit_road_plane(
                        road,
                        threshold_m=self.config.ransac_threshold_m,
                        iterations=self.config.ransac_iterations,
                        min_inliers=self.config.min_road_points,
                        rng=rng,
                    )
                    fr = (
                        FrameResult.failed(i, len(road), None)
                        if plane is None
                        else FrameResult.from_plane(i, plane, None)
                    )
                    if recorder is not None:
                        recorder.add(img0, img1, fr)

                    now = time.monotonic()
                    fps = 0.9 * fps + 0.1 * (1.0 / max(1e-6, now - t_prev))
                    t_prev = now

                    qimg = self._draw(rect0, rect1, res, plane, fr, fps)
                    self.frameReady.emit(qimg, fr)
                    i += 1
        finally:
            if recorder is not None:
                seg = recorder.close()
                self.sessionSaved.emit(str(seg))

    def _draw(self, rect0, rect1, res, plane, fr: FrameResult, fps: float) -> QImage:
        left = _as_bgr(rect0)
        right = _as_bgr(rect1)
        x0, y0 = res.x0, res.y0
        h_roi, w_roi = res.pts.shape[:2]

        # 綠色：路面內點（角度的依據）。只畫在左(cam0)：mask 是 cam0 像素座標，
        # 右圖同像素被視差平移，塗上去會錯位。
        if plane is not None:
            mask = plane_inlier_mask(res.pts, res.valid, plane, self.config.ransac_threshold_m)
            sub = left[y0 : y0 + h_roi, x0 : x0 + w_roi]
            sub[mask] = (0.4 * sub[mask] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)

        # 黃框：ROI（兩顆都畫，強調左右一起算視差）
        cv2.rectangle(left, (x0, y0), (x0 + w_roi, y0 + h_roi), (0, 255, 255), 1)
        cv2.rectangle(right, (x0, y0), (x0 + w_roi, y0 + h_roi), (0, 255, 255), 1)
        _panel_label(left, "L - cam0 (ref)")
        _panel_label(right, "R - cam1")

        # 並排合成後再放大（左圖仍起於 x=0，滑鼠 ROI 對映不變）
        combo = _compose_lr(left, right)
        big = cv2.resize(
            combo, (combo.shape[1] * DISPLAY_SCALE, combo.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )
        # ROI 中心距離：重用已算好的 res（零額外成本），供偵測階段驗證框住物件的前向距離；
        # 物件不是路面故 plane 常為 None，但距離讀數不受影響照樣顯示。
        z_center = self._patch_depth_m(res, w_roi // 2, h_roi // 2)
        _draw_text(big, plane, fr, fps, z_center)
        rgb = cv2.cvtColor(big, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()

    def _roi_rect(self, pw: int, ph: int, roi_fraction: float) -> tuple[int, int, int, int]:
        """目前生效的 ROI 框（process 座標）。與 stereo_from_rectified 的裁切邏輯一致：
        有拉框用框（夾在畫面內），沒框則退回 roi_fraction 的下方橫帶。"""
        if self.roi is not None:
            x0, y0, x1, y1 = self.roi
            x0 = max(0, min(int(x0), pw - 1))
            y0 = max(0, min(int(y0), ph - 1))
            x1 = max(x0 + 1, min(int(x1), pw))
            y1 = max(y0 + 1, min(int(y1), ph))
            return x0, y0, x1, y1
        y0 = int(ph * (1.0 - roi_fraction)) if roi_fraction < 1.0 else 0
        return 0, y0, pw, ph

    def _roi_center_depth(self, rect0, rect1, matcher, rect) -> float | None:
        """框選階段量 ROI 框中心的前向距離 Z（公尺）：只對框內算一次視差、取中心小區塊
        中位 Z。框太窄（< num_disparities+block_size）SGBM 會算出負寬度爆記憶體，直接跳過。
        用途：一眼看出 ROI 框太遠/太近——落在 z_min_m~z_max_m 窗外的話每幀擬合會失敗。"""
        x0, y0, x1, y1 = rect
        if (x1 - x0) < matcher.num_disparities + matcher.block_size:
            return None
        res = stereo_from_rectified(rect0, rect1, self.calib.Q, matcher, rect)
        h_roi, w_roi = res.pts.shape[:2]
        return self._patch_depth_m(res, w_roi // 2, h_roi // 2)

    @staticmethod
    def _patch_depth_m(res, cx: int, cy: int, half: int = 8) -> float | None:
        """res 中心 (cx,cy) 附近小區塊的中位 Z（公尺，前向距離）；無有效點回 None。
        取小區塊中位數避開單一像素的視差雜訊。"""
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

    def _draw_preview(
        self, rect0, rect1, roi_rect: tuple[int, int, int, int], z_center: float | None = None
    ) -> QImage:
        """框選階段的畫面：cam0/cam1 校正後並排預覽 + 黃色 ROI 框 + 「按 Enter 開始」
        提示 + ROI 中心距離（英文，避免 cv2 Hershey 畫不出中文變 ??? ）。"""
        left = _as_bgr(rect0)
        right = _as_bgr(rect1)
        x0, y0, x1, y1 = roi_rect
        cv2.rectangle(left, (x0, y0), (x1, y1), (0, 255, 255), 1)
        cv2.rectangle(right, (x0, y0), (x1, y1), (0, 255, 255), 1)
        _panel_label(left, "L - cam0 (ref)")
        _panel_label(right, "R - cam1")
        combo = _compose_lr(left, right)
        big = cv2.resize(
            combo, (combo.shape[1] * DISPLAY_SCALE, combo.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )
        depth = f"ROI center ~ {z_center:.1f} m" if z_center is not None else "ROI center: no depth"
        for text, yy in (("Frame road ROI", 26), ("press ENTER to start", 56), (depth, 86)):
            cv2.putText(big, text, (10, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(big, text, (10, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        rgb = cv2.cvtColor(big, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()


def _as_bgr(rect) -> np.ndarray:
    """校正影像轉可疊色的 BGR 副本（灰階→BGR，彩色則 copy 不動原圖）。"""
    img = rect.copy()
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _compose_lr(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """cam0(左)、cam1(右)水平並排，中間一條深灰分隔線。兩者同為 process_size。"""
    sep = np.full((left.shape[0], 4, 3), 60, np.uint8)
    return np.hstack([left, sep, right])


def _panel_label(img: np.ndarray, text: str) -> None:
    """在單一 panel 左下角標 L/R 鏡頭來源（黑描邊 + 黃字）。"""
    y = img.shape[0] - 6
    cv2.putText(img, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
    cv2.putText(img, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)


def _draw_text(img, plane, fr: FrameResult, fps: float, z_center: float | None = None) -> None:
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
        for w, col in ((5, (0, 0, 0)), (2, (0, 0, 255))):
            cv2.putText(img, "路面擬合失敗", (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, w)
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


class VideoLabel(QLabel):
    """顯示畫面並支援滑鼠拉框設 ROI、雙擊清除。座標以 process 座標對外發出。"""

    roiSelected = pyqtSignal(object)  # (x0,y0,x1,y1) 或 None

    def __init__(self) -> None:
        super().__init__()
        self.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self._rubber = QRubberBand(QRubberBand.Rectangle, self)
        self._origin = None

    def mousePressEvent(self, e) -> None:
        self._origin = e.pos()
        self._rubber.setGeometry(QRect(self._origin, QSize()))
        self._rubber.show()

    def mouseMoveEvent(self, e) -> None:
        if self._origin is not None:
            self._rubber.setGeometry(QRect(self._origin, e.pos()).normalized())

    def mouseReleaseEvent(self, e) -> None:
        if self._origin is None:
            return
        r = QRect(self._origin, e.pos()).normalized()
        self._origin = None
        self._rubber.hide()
        if r.width() < 8 or r.height() < 8:
            return
        s = DISPLAY_SCALE
        self.roiSelected.emit((r.left() // s, r.top() // s, r.right() // s, r.bottom() // s))

    def mouseDoubleClickEvent(self, e) -> None:
        self.roiSelected.emit(None)  # 清除 ROI


class MainWindow(QWidget):
    def __init__(self, calib: StereoCalibration, config: Config, record: bool = False) -> None:
        super().__init__()
        self.calib = calib
        self.config = config
        self.setWindowTitle("前方路面坡度 — 即時")
        self._record = record
        self.setFocusPolicy(Qt.StrongFocus)  # 讓視窗收得到 Enter 鍵
        self.video = VideoLabel()
        self.hint = QLabel(
            "框選階段：滑鼠拉框設定路面 ROI（會記住）、雙擊清除；左上顯示框中心距離。"
            "按 s 存圖（check_dist/）、按 Enter 開始預測角度。"
        )
        layout = QVBoxLayout(self)
        layout.addWidget(self.video, 1)
        layout.addWidget(self.hint)

        self.worker = StereoWorker(calib, config, record=record)
        # 套用上次記住的 ROI（換解析度作廢時 load_roi 回 None＝退回預設橫帶）
        self.worker.roi = load_roi(config.roi_path, calib.process_size)
        self.worker.frameReady.connect(self._on_frame)
        self.worker.sessionSaved.connect(self._on_saved)
        self.video.roiSelected.connect(self._on_roi)
        self._last_qimg: QImage | None = None  # 最新一幀（含 ROI 框+距離文字），供 s 鍵存圖
        self._shot_dir = Path("check_dist")  # 跟 tool/measure_distance.py 的截圖放一起
        self.worker.start()  # 開視窗先進框選階段，按 Enter 才開始預測

    def keyPressEvent(self, e) -> None:
        # s：把當前畫面（含 ROI 框+距離讀數）存成圖片，供只看距離、不跑辨識時記錄。
        if e.key() == Qt.Key_S:
            self._save_shot()
            return
        # Enter：從框選階段進入角度預測（並開始錄影）。已在預測中則忽略。
        if e.key() in (Qt.Key_Return, Qt.Key_Enter) and not self.worker.active:
            self.worker.active = True
            msg = "● 預測角度中：滑鼠仍可重拉 ROI、雙擊清除。Ctrl+C / 關閉視窗結束。"
            if self._record:
                msg = "● 預測角度中（錄影中）：滑鼠仍可重拉 ROI、雙擊清除。關閉視窗結束。"
            self.hint.setText(msg)
        else:
            super().keyPressEvent(e)

    def _save_shot(self) -> None:
        """存最新一幀到 check_dist/sample_NNN.png（遞增、不覆蓋、跨執行接續編號，
        沿用 tool/measure_distance.py 的慣例）。框選階段就能按，不必進辨識/錄影。"""
        if self._last_qimg is None:
            self.hint.setText("還沒有畫面可存。")
            return
        self._shot_dir.mkdir(parents=True, exist_ok=True)
        existing = [int(m.stem[7:]) for m in self._shot_dir.glob("sample_*.png") if m.stem[7:].isdigit()]
        n = (max(existing) + 1) if existing else 0
        path = self._shot_dir / f"sample_{n:03d}.png"
        if self._last_qimg.save(str(path)):
            self.hint.setText(f"已存圖 → {path}")
        else:
            self.hint.setText(f"存圖失敗：{path}")

    def _on_roi(self, roi) -> None:
        self.worker.roi = roi
        save_roi(self.config.roi_path, roi, self.calib.process_size)  # 記住供下次套用

    def _on_frame(self, qimg: QImage, fr: FrameResult) -> None:
        self._last_qimg = qimg  # 供 s 鍵存圖
        self.video.setPixmap(QPixmap.fromImage(qimg))
        self.video.setFixedSize(qimg.width(), qimg.height())

    def _on_saved(self, seg: str) -> None:
        self.hint.setText(f"已存到 {seg}（cam0/cam1.mp4 + CSV + 趨勢圖）")

    def closeEvent(self, event) -> None:
        self.worker.stop()
        self.worker.wait(3000)
        super().closeEvent(event)


def run_ui(calib: StereoCalibration, config: Config, record: bool = False) -> int:
    app = QApplication.instance() or QApplication([])
    win = MainWindow(calib, config, record=record)
    win.show()
    return app.exec_()
