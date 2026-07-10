"""即時視覺介面 (PyQt5)。

一開視窗就自動開始（不用按鈕）。畫面＝左鏡頭即時影像，疊上：
  - 綠色：被判定為路面、拿去算坡度的那片點（＝角度的依據，越乾淨越可信）
  - 黃框：目前的 ROI（只在框內算視差）
  - 文字：前方路面坡度、RMS(擬合殘差)、內點比例、fps

滑鼠操作：
  - 在影像上「拖一個方框」＝設定 ROI（只算框內，聚焦到路面）
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

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        from .live import LiveStereo  # 延後 import，只有 UI 才需要 picamera2

        matcher = StereoMatcher.from_config(self.config)
        rng = np.random.default_rng(self.config.seed)
        recorder = None
        if self.record:
            from .recorder import SessionRecorder

            recorder = SessionRecorder(
                self.config.output_dir, self.calib.native_size, self.config.record_fps
            )
        t_prev = time.monotonic()
        fps = 0.0
        i = 0
        try:
            with LiveStereo(self.config, size=self.calib.native_size) as cams:
                for img0, img1 in cams.frames():
                    if not self._running:
                        break
                    rect0, rect1 = self.calib.rectify(img0, img1)
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
        self.setWindowTitle("前方路面坡度 — 即時")
        self.video = VideoLabel()
        hint = "滑鼠拉框＝設定路面 ROI；雙擊＝清除。綠色＝算角度用的路面點。"
        if record:
            hint += "  ●錄影中"
        self.hint = QLabel(hint)
        layout = QVBoxLayout(self)
        layout.addWidget(self.video, 1)
        layout.addWidget(self.hint)

        self.worker = StereoWorker(calib, config, record=record)
        self.worker.frameReady.connect(self._on_frame)
        self.worker.sessionSaved.connect(self._on_saved)
        self.video.roiSelected.connect(self._on_roi)
        self.worker.start()  # 一開視窗就自動跑，不用按鈕

    def _on_roi(self, roi) -> None:
        self.worker.roi = roi

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
