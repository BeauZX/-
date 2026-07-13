"""即時視覺介面 (PyQt5)。

兩階段：開視窗先進「框選階段」（只顯示預覽 + 黃色 ROI 框，跳過 SGBM/擬合/錄影，
預覽更順好瞄準），把框調到滿意後按 **Enter** 才進入「預測階段」（開始算角度並錄影）。
（headless 純 `--live` 不經過本 worker、不需 Enter，一律直接跑。）

預測階段畫面＝左鏡頭即時影像，疊上：
  - 綠色：被判定為路面、拿去算坡度的那片點（＝角度的依據，越乾淨越可信）
  - 黃框：目前的 ROI（只在框內算視差）
  - 文字：前方路面坡度、RMS(擬合殘差)、內點比例、fps

滑鼠操作（兩階段都可用）：
  - 在影像上「拖一個方框」＝設定 ROI（只算框內，聚焦到路面），會存進 roi.json 記住
  - 「雙擊」＝清除 ROI，回到預設下方橫帶

運算在背景 QThread，GUI 只負責畫。由 CLI `--ui`（或 config.show_ui=True）啟動。
"""

from __future__ import annotations

import time

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
                        self.frameReady.emit(self._draw_preview(rect0, rect), None)
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

                    qimg = self._draw(rect0, res, plane, fr, fps)
                    self.frameReady.emit(qimg, fr)
                    i += 1
        finally:
            if recorder is not None:
                seg = recorder.close()
                self.sessionSaved.emit(str(seg))

    def _draw(self, rect0, res, plane, fr: FrameResult, fps: float) -> QImage:
        base = rect0.copy()
        if base.ndim == 2:
            base = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
        x0, y0 = res.x0, res.y0
        h_roi, w_roi = res.pts.shape[:2]

        # 綠色：路面內點（角度的依據）
        if plane is not None:
            mask = plane_inlier_mask(res.pts, res.valid, plane, self.config.ransac_threshold_m)
            sub = base[y0 : y0 + h_roi, x0 : x0 + w_roi]
            sub[mask] = (0.4 * sub[mask] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)

        # 黃框：ROI
        cv2.rectangle(base, (x0, y0), (x0 + w_roi, y0 + h_roi), (0, 255, 255), 1)

        # 放大顯示
        big = cv2.resize(
            base, (base.shape[1] * DISPLAY_SCALE, base.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )
        _draw_text(big, plane, fr, fps)
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

    def _draw_preview(self, rect0, roi_rect: tuple[int, int, int, int]) -> QImage:
        """框選階段的畫面：校正後預覽 + 黃色 ROI 框 + 「按 Enter 開始」提示（英文，
        避免 cv2 Hershey 畫不出中文變 ??? ）。"""
        base = rect0.copy()
        if base.ndim == 2:
            base = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
        x0, y0, x1, y1 = roi_rect
        cv2.rectangle(base, (x0, y0), (x1, y1), (0, 255, 255), 1)
        big = cv2.resize(
            base, (base.shape[1] * DISPLAY_SCALE, base.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )
        for text, col in (("Frame road ROI", (0, 255, 255)),
                          ("press ENTER to start", (0, 255, 255))):
            cv2.putText(big, text, (10, 26 if "Frame" in text else 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(big, text, (10, 26 if "Frame" in text else 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        rgb = cv2.cvtColor(big, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()


def _draw_text(img, plane, fr: FrameResult, fps: float) -> None:
    if plane is None:
        lines = [("路面擬合失敗", (0, 0, 255))]
    else:
        ratio = fr.n_inliers / fr.n_road_points if fr.n_road_points else 0.0
        rms_cm = plane.rms_m * 100
        # 可信度：RMS 小且內點比例高 → 綠，否則橘
        good = rms_cm < 4.0 and ratio > 0.5
        col = (0, 255, 0) if good else (0, 165, 255)
        lines = [
            (f"pitch {fr.pitch_deg:+.1f}  roll {fr.roll_deg:+.1f}  h {fr.cam_height_m:.2f}m", col),
            (f"RMS {rms_cm:.1f}cm  inliers {ratio*100:.0f}%  fps {fps:.1f}", col),
        ]
    y = 26
    for text, col in lines:
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        y += 30


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
            "框選階段：滑鼠拉框設定路面 ROI（會記住）、雙擊清除；滿意後按 Enter 開始預測角度。"
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
        self.worker.start()  # 開視窗先進框選階段，按 Enter 才開始預測

    def keyPressEvent(self, e) -> None:
        # Enter：從框選階段進入角度預測（並開始錄影）。已在預測中則忽略。
        if e.key() in (Qt.Key_Return, Qt.Key_Enter) and not self.worker.active:
            self.worker.active = True
            msg = "● 預測角度中：滑鼠仍可重拉 ROI、雙擊清除。Ctrl+C / 關閉視窗結束。"
            if self._record:
                msg = "● 預測角度中（錄影中）：滑鼠仍可重拉 ROI、雙擊清除。關閉視窗結束。"
            self.hint.setText(msg)
        else:
            super().keyPressEvent(e)

    def _on_roi(self, roi) -> None:
        self.worker.roi = roi
        save_roi(self.config.roi_path, roi, self.calib.process_size)  # 記住供下次套用

    def _on_frame(self, qimg: QImage, fr: FrameResult) -> None:
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
