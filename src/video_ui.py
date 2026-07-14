"""影片專用視覺介面 (PyQt5)——跟即時的 ui.py 分開，操作流程也不同。

用「錄好的影片」跑，跟即時最大的差別是「用空白鍵切出想要的路段」：

    1. 開視窗，影片循環播放 → 滑鼠拉框設定路面 ROI（雙擊清除，會記住 roi.json）
    2. 按 Enter → 開始偵測（疊綠色路面 + 顯示前方坡度），同時開始把這段錄下來
    3. 影片裡有回轉／不想要的片段；看著畫面覺得「這段就是我要的路段」時按空白鍵
    4. 空白鍵 → 結束偵測，把 Enter 到空白鍵這段輸出成一個 segment：
         cam0.mp4 / cam1.mp4     原影片（這段的原始左右畫面，native 解析度）
         detect.mp4              偵測影片（cam0/cam1 並排：左疊綠色路面 + 坡度文字，右只黃框）
         road_angle.csv          每幀角度
         road_angle_trend.png    坡度趨勢圖

運算在背景 QThread。由 run_video.py 啟動。跟 ui.py 共用的只有通用顯示元件
（VideoLabel 滑鼠拉框、_as_bgr/_compose_lr/_panel_label 疊圖、DISPLAY_SCALE）；
偵測畫面的文字改由本檔自己的 _draw_slope 畫（精簡版，跟即時 ui.py 的 _draw_text
分開），其餘完全獨立、不影響即時模式。
"""

from __future__ import annotations

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

from .calib_loader import StereoCalibration
from .config import Config
from .disparity import StereoMatcher, stereo_from_rectified
from .imu_track import ImuTrack
from .pipeline import FrameResult
from .recorder import SessionRecorder
from .roadplane import fit_road_plane, plane_inlier_mask, select_road_points
from .roi_store import load_roi, save_roi
from .ui import (  # 通用顯示元件，沿用不重造
    DISPLAY_SCALE,
    VideoLabel,
    _as_bgr,
    _compose_lr,
    _panel_label,
)
from .video_source import VideoStereo


def _draw_slope(img: np.ndarray, plane, fr: FrameResult) -> None:
    """偵測畫面只留主結果 + 兩盞 sanity 燈，其餘（pitch/RMS/inliers/fps/imu/ROI 距離）
    不上螢幕、全留在 road_angle.csv：

        slope <大字>        前方路面相對水平面的坡度（雙目+IMU；無 IMU 退回純雙目 pitch）
        h .. m   roll ..    相機估計高度 + 橫向坡度（確認擬到的是路面、不是牆）

    文字顏色沿用可信度：RMS 小且內點比例高→綠、否則橘（隱含 RMS/內點，不再列數字）。
    cv2 Hershey 畫不出中文：擬合失敗訊息會顯示為紅色 ??????（非當機，見 CLAUDE.md）。"""
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


class VideoWorker(QThread):
    """背景執行緒：讀影片、逐幀算坡度、發出疊好圖的畫面 + 結果。

    兩階段（同即時）：active=False 框選（只預覽+ROI 框，不算不錄）；主執行緒按 Enter
    設 active=True 開始偵測並錄影。主執行緒按空白鍵呼叫 stop()，run() 收尾寫出這段。
    """

    frameReady = pyqtSignal(QImage, object)
    sessionSaved = pyqtSignal(str)

    def __init__(self, calib: StereoCalibration, config: Config, cam0, cam1) -> None:
        super().__init__()
        self.calib = calib
        self.config = config
        self.cam0 = cam0
        self.cam1 = cam1
        self._running = True
        self.roi: tuple[int, int, int, int] | None = None  # process 座標，主執行緒設定
        self.active = False  # False＝框選階段；Enter→True 開始偵測+錄影

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        matcher = StereoMatcher.from_config(self.config)
        rng = np.random.default_rng(self.config.seed)

        # === 框選階段：定格在影片第一幀（底圖不動，方便框 ROI）===
        # 只解第一幀當靜止底圖，按 Enter 前一直重畫它——影像固定不動，但拖曳中的黃色
        # ROI 框（讀 self.roi）每次重畫都會更新，所以框得到、又不會被移動的畫面干擾。
        first0, first1 = VideoStereo(
            self.cam0, self.cam1, size=self.calib.native_size
        ).first_frame()
        rect0_frozen, rect1_frozen = self.calib.rectify(first0, first1)
        pw, ph = self.calib.process_size
        # 對定格幀「整張」算一次視差/3D，之後拖 ROI 只要索引就能即時查框中心是幾公尺
        # （前向距離 Z），用來判斷 ROI 是不是框太遠（太遠→遠處視差稀疏→辨識差）。
        res_frozen = stereo_from_rectified(
            rect0_frozen, rect1_frozen, self.calib.Q, matcher, (0, 0, pw, ph)
        )
        while self._running and not self.active:
            rect = self._roi_rect(pw, ph, matcher.roi_fraction)
            cx, cy = (rect[0] + rect[2]) // 2, (rect[1] + rect[3]) // 2
            z = self._patch_depth_m(res_frozen, cx - res_frozen.x0, cy - res_frozen.y0)
            self.frameReady.emit(self._draw_preview(rect0_frozen, rect1_frozen, rect, z), None)
            self.msleep(50)  # 約 20fps 重畫，讓 ROI 框跟手；底圖同一張不動
        if not self._running:
            return

        # === 偵測階段：從頭播放影片、逐幀算角度並錄影，直到按空白鍵 stop() ===
        # IMU 輔助：影片旁有 imu_raw.csv 且 config.use_imu 開啟時，讀當初錄影記錄的
        # 逐幀相機 pitch，補算相對水平面的真實坡度（pitch_gravity）。沒有就 None＝純雙目。
        imu_track = ImuTrack.load(self.cam0, self.config) if self.config.use_imu else None
        recorder = None  # 進偵測才建，避免框選階段就開一個空 segment
        detect_vw = None  # 疊好圖的偵測影片 writer
        i = 0
        try:
            with VideoStereo(self.cam0, self.cam1, size=self.calib.native_size, loop=True) as cams:
                for img0, img1 in cams.frames():
                    if not self._running:
                        break
                    rect0, rect1 = self.calib.rectify(img0, img1)

                    if recorder is None:  # 第一幀才建錄影器（避免框選階段就開一個空 segment）
                        recorder = SessionRecorder(
                            self.config.output_dir,
                            self.calib.native_size,
                            self.config.record_fps,
                            self.config.segment_seconds,  # run_video 設 0＝這段不切、一張趨勢圖
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
                    if imu_track is not None:  # 補算相對水平面的真實坡度（pitch_gravity）
                        fr.with_imu(imu_track.pitch_for_frame(i))
                    recorder.add(img0, img1, fr)  # 原影片（native cam0/cam1）+ 累積角度

                    big = self._annotate(rect0, rect1, res, plane, fr)  # 並排 BGR 畫面
                    if detect_vw is None:  # 用實際並排畫面尺寸建 writer（左右並排比單圖寬）
                        detect_vw = cv2.VideoWriter(
                            str(recorder.dir / "detect.mp4"),
                            cv2.VideoWriter_fourcc(*"mp4v"),
                            self.config.record_fps,
                            (big.shape[1], big.shape[0]),
                        )
                    detect_vw.write(big)  # 偵測影片
                    self.frameReady.emit(self._to_qimage(big), fr)
                    i += 1
        finally:
            if detect_vw is not None:
                detect_vw.release()
            if recorder is not None:
                seg = recorder.close()  # 寫 CSV + 趨勢圖、封 mp4
                self.sessionSaved.emit(str(seg))

    def _roi_rect(self, pw: int, ph: int, roi_fraction: float) -> tuple[int, int, int, int]:
        """目前生效的 ROI 框（process 座標）：有拉框用框（夾在畫面內），否則預設下方橫帶。"""
        if self.roi is not None:
            x0, y0, x1, y1 = self.roi
            x0 = max(0, min(int(x0), pw - 1))
            y0 = max(0, min(int(y0), ph - 1))
            x1 = max(x0 + 1, min(int(x1), pw))
            y1 = max(y0 + 1, min(int(y1), ph))
            return x0, y0, x1, y1
        y0 = int(ph * (1.0 - roi_fraction)) if roi_fraction < 1.0 else 0
        return 0, y0, pw, ph

    def _draw_preview(
        self, rect0, rect1, roi_rect: tuple[int, int, int, int], z_center: float | None = None
    ) -> QImage:
        """框選階段畫面：cam0(左)/cam1(右) 校正後並排預覽 + 黃色 ROI 框（兩顆都畫，強調
        左右一起算視差）+ 英文提示 + ROI 中心距離（cv2 畫不出中文）。"""
        left = _as_bgr(rect0)
        right = _as_bgr(rect1)
        x0, y0, x1, y1 = roi_rect
        cv2.rectangle(left, (x0, y0), (x1, y1), (0, 255, 255), 1)
        cv2.rectangle(right, (x0, y0), (x1, y1), (0, 255, 255), 1)
        _panel_label(left, "L - cam0 (ref)")
        _panel_label(right, "R - cam1")
        combo = _compose_lr(left, right)  # 左圖仍起於 x=0，滑鼠 ROI 對映不變
        big = cv2.resize(
            combo, (combo.shape[1] * DISPLAY_SCALE, combo.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )
        depth = f"ROI center ~ {z_center:.1f} m" if z_center is not None else "ROI center: no depth"
        for text, yy in (("Frame road ROI", 26), ("press ENTER to start", 56), (depth, 86)):
            cv2.putText(big, text, (10, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(big, text, (10, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return self._to_qimage(big)

    def _annotate(self, rect0, rect1, res, plane, fr: FrameResult) -> np.ndarray:
        """cam0(左)/cam1(右) 校正後並排 + 精簡角度文字，回傳放大後的 BGR 畫面（顯示與偵測
        影片共用）。綠色路面內點只疊在左圖(cam0)：mask 是 cam0 像素座標，右圖同像素被視差
        平移、塗上去會錯位；黃色 ROI 框兩顆都畫。"""
        left = _as_bgr(rect0)
        right = _as_bgr(rect1)
        x0, y0 = res.x0, res.y0
        h_roi, w_roi = res.pts.shape[:2]
        if plane is not None:  # 綠色：路面內點（角度依據，只畫在左圖 cam0）
            mask = plane_inlier_mask(res.pts, res.valid, plane, self.config.ransac_threshold_m)
            sub = left[y0 : y0 + h_roi, x0 : x0 + w_roi]
            sub[mask] = (0.4 * sub[mask] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)
        cv2.rectangle(left, (x0, y0), (x0 + w_roi, y0 + h_roi), (0, 255, 255), 1)  # 黃框 ROI
        cv2.rectangle(right, (x0, y0), (x0 + w_roi, y0 + h_roi), (0, 255, 255), 1)
        _panel_label(left, "L - cam0 (ref)")
        _panel_label(right, "R - cam1")
        combo = _compose_lr(left, right)
        big = cv2.resize(
            combo, (combo.shape[1] * DISPLAY_SCALE, combo.shape[0] * DISPLAY_SCALE),
            interpolation=cv2.INTER_NEAREST,
        )
        _draw_slope(big, plane, fr)
        return big

    @staticmethod
    def _patch_depth_m(res, cx: int, cy: int, half: int = 8) -> float | None:
        """res 中心 (cx,cy) 附近小區塊的中位 Z（公尺，前向距離）；無有效點回 None。

        cx,cy 是 res 陣列的局部座標（框選階段整張算故＝process 座標；偵測階段 res 已是
        ROI 子區塊故傳中心 w//2,h//2）。取小區塊中位數避開單一像素的視差雜訊。
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

    @staticmethod
    def _to_qimage(big: np.ndarray) -> QImage:
        rgb = cv2.cvtColor(big, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()


class VideoWindow(QWidget):
    def __init__(self, calib: StereoCalibration, config: Config, cam0, cam1) -> None:
        super().__init__()
        self.calib = calib
        self.config = config
        self.setWindowTitle("前方路面坡度 — 影片")
        self.setFocusPolicy(Qt.StrongFocus)  # 收 Enter / Space
        self.video = VideoLabel()
        self.hint = QLabel(
            "框選階段：滑鼠拉框設定路面 ROI（會記住）、雙擊清除；滿意後按 Enter 開始偵測。"
        )
        layout = QVBoxLayout(self)
        layout.addWidget(self.video, 1)
        layout.addWidget(self.hint)

        self.worker = VideoWorker(calib, config, cam0, cam1)
        self.worker.roi = load_roi(config.roi_path, calib.process_size)
        self.worker.frameReady.connect(self._on_frame)
        self.worker.sessionSaved.connect(self._on_saved)
        self.video.roiSelected.connect(self._on_roi)
        self.worker.start()

    def keyPressEvent(self, e) -> None:
        # Enter：框選階段 → 開始偵測 + 錄影
        if e.key() in (Qt.Key_Return, Qt.Key_Enter) and not self.worker.active:
            self.worker.active = True
            self.hint.setText(
                "● 偵測中：看到想要的路段就按「空白鍵」結束並輸出"
                "（原影片 cam0/cam1 + 偵測影片 detect.mp4 + 趨勢圖 + CSV）。滑鼠仍可重拉 ROI。"
            )
        # Space：偵測中 → 結束，把 Enter 到現在這段輸出成一個 segment
        elif e.key() == Qt.Key_Space and self.worker.active:
            self.hint.setText("結束偵測、輸出中…（原影片 + 偵測影片 + 趨勢圖）")
            self.worker.stop()
        else:
            super().keyPressEvent(e)

    def _on_roi(self, roi) -> None:
        self.worker.roi = roi
        save_roi(self.config.roi_path, roi, self.calib.process_size)

    def _on_frame(self, qimg: QImage, fr: FrameResult) -> None:
        self.video.setPixmap(QPixmap.fromImage(qimg))
        self.video.setFixedSize(qimg.width(), qimg.height())

    def _on_saved(self, seg: str) -> None:
        self.hint.setText(
            f"已輸出 → {seg}（cam0/cam1.mp4 原影片 + detect.mp4 偵測影片 + "
            "road_angle.csv + road_angle_trend.png）。可關閉視窗。"
        )

    def closeEvent(self, event) -> None:
        self.worker.stop()
        self.worker.wait(3000)
        super().closeEvent(event)


def run_video_ui(calib: StereoCalibration, config: Config, cam0, cam1) -> int:
    app = QApplication.instance() or QApplication([])
    win = VideoWindow(calib, config, cam0, cam1)
    win.show()
    return app.exec_()
