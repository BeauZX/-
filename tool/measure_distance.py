"""雙目測距驗證工具：框一個物件 → 雙目量它的距離 → 跟你用捲尺量的實際值比對誤差。

用途跟主專案（估路面坡度）不同：這支只是拿來**驗證雙目深度準不準**。流程：
  1. 開視窗，滑鼠在畫面上「拉一個框」框住你要量的物件（雙擊清除框）。
  2. 框內用 StereoSGBM 算視差 → 3D 點雲，取框內有效點的**中位 Z（前向距離）**當量測值。
  3. 你自己拿捲尺量同一個物件的實際距離，填進右上角的輸入框（公尺），畫面就即時顯示
     量測值、實際值、絕對誤差與百分比誤差。
  4. 按 's'（或按鈕）把當前這筆「量測 / 實際 / 誤差」存進 CSV，方便多量幾組看整體偏差。

跟 src/ 的關係：只**消費** src 的 calib_loader / disparity / live / video_source，
並沿用 src/overlay.py 的顯示輔助（as_bgr/compose_lr/panel_label）；拉框元件是自己的
FitVideoLabel（顯示縮放可變），不 import src/ui.py。
不改 src。

執行（在 Road_angle/ 底下）：
    python3 tool/measure_distance.py                     # 即時雙目鏡頭（預設，戶外白天曝光）
    python3 tool/measure_distance.py --shutter 30000 --gain 5.0  # 室內昏暗：曝光調大免太黑
    python3 tool/measure_distance.py --cam0 a.mp4 --cam1 b.mp4   # 用錄好的兩支影片
    python3 tool/measure_distance.py --expected 3.5      # 先給實際距離(公尺)，開窗就顯示誤差
    python3 tool/measure_distance.py --scale 0.5         # 降解析度換速度(預設 1.0 求準)

註：曝光預設戶外白天 2000µs/1.0（對齊上層 rpi5_dual_camera_capture.py），室內昏暗要
--shutter/--gain 調大。驗證深度時預設用 process_scale=1.0（全解析度算視差，深度誤差最小）；
覺得太慢再用 --scale 調小。近距離物件量不到時，物件可能比最近可測距離還近（見 num_disparities）。
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# 讓 `python3 tool/measure_distance.py` 找得到上一層的 src 套件
_ROOT = Path(__file__).resolve().parent.parent  # 專案根目錄 Road_angle/
sys.path.insert(0, str(_ROOT))

from PyQt5.QtCore import QRect, QSize, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QRubberBand,
    QVBoxLayout,
    QWidget,
)

from src.calib_loader import StereoCalibration, load_calibration
from src.config import DEFAULT, Config
from src.disparity import StereoMatcher, stereo_from_rectified
from src.overlay import as_bgr, compose_lr, panel_label

# 並排畫面顯示時縮放到「總寬不超過這個像素」，避免全解析度(scale=1.0)時視窗被撐爆
MAX_DISPLAY_WIDTH = 1400


class Measurement:
    """一幀在 ROI 內量到的距離統計（公尺）。"""

    __slots__ = ("median", "mean", "std", "p25", "p75", "count")

    def __init__(self, z: np.ndarray) -> None:
        self.median = float(np.median(z))
        self.mean = float(np.mean(z))
        self.std = float(np.std(z))
        self.p25 = float(np.percentile(z, 25))
        self.p75 = float(np.percentile(z, 75))
        self.count = int(z.size)


class MeasureWorker(QThread):
    """背景執行緒：開來源、逐幀 rectify + 在 ROI 內算距離，發出疊好圖的畫面 + 量測值。"""

    frameReady = pyqtSignal(QImage, object)  # (畫面, Measurement 或 None)

    def __init__(self, calib: StereoCalibration, config: Config, source: str,
                 cam0: str | None, cam1: str | None,
                 expected: float | None = None) -> None:
        super().__init__()
        self.calib = calib
        self.config = config
        self.source = source  # "live" 或 "video"
        self.cam0 = cam0
        self.cam1 = cam1
        self.expected = expected  # 實際距離(公尺)，畫面顯示誤差用；None＝只顯示距離
        self._running = True
        self.roi: tuple[int, int, int, int] | None = None  # process 座標，主執行緒設定
        # 顯示縮放：讓「並排總寬」不超過 MAX_DISPLAY_WIDTH（全解析度也不會撐爆視窗）；
        # 小圖最多放大 2 倍。滑鼠座標會用同一個比例換回 process 座標。
        pw, ph = calib.process_size
        combo_w = 2 * pw + 4  # compose_lr 中間有 4px 分隔線
        self.disp_scale = min(2.0, MAX_DISPLAY_WIDTH / combo_w)

    def stop(self) -> None:
        self._running = False

    def _open_source(self):
        if self.source == "live":
            from src.live import LiveStereo

            return LiveStereo(self.config, size=self.calib.native_size)
        from src.video_source import VideoStereo

        return VideoStereo(self.cam0, self.cam1, size=self.calib.native_size, loop=True)

    def run(self) -> None:
        matcher = StereoMatcher.from_config(self.config)
        t_prev = time.monotonic()
        fps = 0.0
        first = True
        try:
            with self._open_source() as cams:
                for img0, img1 in cams.frames():
                    if not self._running:
                        break
                    if first:  # 診斷用：確認有收到幀、亮度夠不夠（太黑=曝光問題）
                        print(f"[measure] first frame OK  cam0 亮度均值={float(img0.mean()):.1f}"
                              f"  cam1={float(img1.mean()):.1f}  (0=全黑, 太低就加 --shutter/--gain)")
                        first = False
                    rect0, rect1 = self.calib.rectify(img0, img1)

                    meas = None
                    note = None
                    if self.roi is not None:
                        pw = self.calib.process_size[0]
                        min_w = matcher.num_disparities + matcher.block_size + 1
                        if int(self.roi[0]) >= pw:
                            # 框在右邊那張(cam1)：測距以左圖(cam0)為基準，右圖不能設 ROI
                            note = "Draw the box on the LEFT (cam0) image"
                        else:
                            # 框太窄（寬 <= num_disparities）會讓 SGBM 算出負寬度而爆記憶體，
                            # 先擋掉、提示框寬一點，不讓程式 crash。
                            x0, y0, x1, y1 = self._clamp_roi()
                            if (x1 - x0) < min_w:
                                note = f"ROI too narrow: draw wider (need >= {min_w}px)"
                            else:
                                res = stereo_from_rectified(
                                    rect0, rect1, self.calib.Q, matcher, (x0, y0, x1, y1)
                                )
                                z = res.pts[..., 2][res.valid & np.isfinite(res.pts[..., 2])]
                                if z.size > 0:
                                    meas = Measurement(z)

                    now = time.monotonic()
                    fps = 0.9 * fps + 0.1 * (1.0 / max(1e-6, now - t_prev))
                    t_prev = now

                    self.frameReady.emit(self._draw(rect0, rect1, meas, fps, note), meas)
        finally:
            pass

    def _clamp_roi(self) -> tuple[int, int, int, int]:
        """把 self.roi 夾在 process 影像範圍內（跟 stereo_from_rectified 的裁切邏輯一致）。"""
        pw, ph = self.calib.process_size
        x0, y0, x1, y1 = self.roi
        x0 = max(0, min(int(x0), pw - 1)); y0 = max(0, min(int(y0), ph - 1))
        x1 = max(x0 + 1, min(int(x1), pw)); y1 = max(y0 + 1, min(int(y1), ph))
        return x0, y0, x1, y1

    def _draw(self, rect0, rect1, meas: Measurement | None, fps: float,
              note: str | None = None) -> QImage:
        left = as_bgr(rect0)
        right = as_bgr(rect1)

        if self.roi is not None:
            x0, y0, x1, y1 = self._clamp_roi()
            cv2.rectangle(left, (x0, y0), (x1, y1), (0, 255, 255), 1)
            cv2.rectangle(right, (x0, y0), (x1, y1), (0, 255, 255), 1)
        panel_label(left, "L - cam0 (ref)")
        panel_label(right, "R - cam1")

        combo = compose_lr(left, right)
        s = self.disp_scale
        big = cv2.resize(
            combo, (int(round(combo.shape[1] * s)), int(round(combo.shape[0] * s))),
            interpolation=cv2.INTER_NEAREST if s >= 1.0 else cv2.INTER_AREA,
        )
        self._overlay(big, meas, fps, note, self.expected)
        rgb = cv2.cvtColor(big, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        return QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()

    @staticmethod
    def _overlay(img: np.ndarray, meas: Measurement | None, fps: float,
                 note: str | None = None, expected: float | None = None) -> None:
        # 畫面顯示「距離」+「fps」(+ 給了 --expected 就多一行誤差)；IQR/std/n 仍寫進 CSV 備查。
        if note is not None:  # 例如框太窄
            lines = [(note, (0, 165, 255), 0.7)]
        elif meas is None:
            lines = [("Drag a box on the object to measure", (0, 255, 255), 0.7)]
        else:
            lines = [(f"Distance: {meas.median:.3f} m", (0, 255, 0), 1.0)]
            if expected is not None:  # 誤差 = 量測 − 實際；黃字強調這是比對結果
                err = meas.median - expected
                pct = 100.0 * err / expected if expected else 0.0
                lines.append(
                    (f"exp {expected:.3f} m  err {err:+.3f} m ({pct:+.1f}%)", (0, 255, 255), 0.7)
                )
            lines.append((f"fps {fps:.1f}", (0, 255, 0), 0.7))
        y = 34
        for text, col, scale in lines:
            cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 5)
            cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, 2)
            y += 34


class FitVideoLabel(QLabel):
    """顯示畫面並支援滑鼠拉框設 ROI、雙擊清除。

    概念跟 src/ui.py 的 VideoLabel 一樣，但顯示縮放是可變的浮點 scale（不是固定 ×2），
    所以滑鼠座標除以這個 scale 換回 process 座標。ROI 只在左圖(cam0)座標系有意義，
    左圖起於 x=0，故直接除以 scale 即得 process 座標。
    """

    roiSelected = pyqtSignal(object)  # (x0,y0,x1,y1) 或 None

    def __init__(self, scale: float) -> None:
        super().__init__()
        self.scale = scale
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
        s = self.scale
        self.roiSelected.emit(
            (int(r.left() / s), int(r.top() / s), int(r.right() / s), int(r.bottom() / s))
        )

    def mouseDoubleClickEvent(self, e) -> None:
        self.roiSelected.emit(None)  # 清除 ROI


class MainWindow(QWidget):
    def __init__(self, calib: StereoCalibration, config: Config, source: str,
                 cam0: str | None, cam1: str | None, outdir: str,
                 expected: float | None = None) -> None:
        super().__init__()
        self.calib = calib
        self.config = config
        self.expected = expected  # 實際距離(公尺)，用來算誤差；None＝只顯示距離
        self.outdir = Path(outdir)
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.outdir / "distance_log.csv"
        # 續號：掃現有 sample_*.png，從最大編號 +1 起，跨多次執行不覆蓋、接著排
        existing = sorted(self.outdir.glob("sample_*.png"))
        self._next_i = 1 + max(
            (int(p.stem.split("_")[1]) for p in existing if p.stem.split("_")[1].isdigit()),
            default=0,
        )
        self._last: Measurement | None = None
        self.setWindowTitle("雙目測距驗證")
        self.setFocusPolicy(Qt.StrongFocus)

        self.worker = MeasureWorker(calib, config, source, cam0, cam1, expected)
        self.video = FitVideoLabel(self.worker.disp_scale)

        self.save_btn = QPushButton("存這筆 (s)")
        self.save_btn.clicked.connect(self._save_sample)
        top = QHBoxLayout()
        top.addStretch(1)
        top.addWidget(self.save_btn)

        self.hint = QLabel(
            "滑鼠拉框框住物件（雙擊清除）；量測距離看畫面左上；按 s 存截圖 + CSV 到 check_dist。"
        )

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.video, 1)
        layout.addWidget(self.hint)

        self.worker.frameReady.connect(self._on_frame)
        self.video.roiSelected.connect(self._on_roi)
        self.worker.start()

    def _on_roi(self, roi) -> None:
        self.worker.roi = roi

    def _on_frame(self, qimg: QImage, meas: Measurement | None) -> None:
        self._last = meas
        self.video.setPixmap(QPixmap.fromImage(qimg))
        self.video.setFixedSize(qimg.width(), qimg.height())

    def _save_sample(self) -> None:
        meas = self._last
        if meas is None:
            self.hint.setText("還沒有量測值：先拉框框住物件再存。")
            return

        # 截圖：抓整個視窗（含左右畫面 + 綠字量測讀數），存遞增編號、不覆蓋
        idx = self._next_i
        self._next_i += 1
        img_name = f"sample_{idx:03d}.png"
        img_path = self.outdir / img_name
        ok = self.grab().save(str(img_path))

        new_file = not self.log_path.exists()
        with open(self.log_path, "a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow([
                    "index", "image", "timestamp", "measured_m",
                    "expected_m", "error_m", "error_pct",
                    "iqr_m", "std_m", "n_points", "roi",
                ])
            exp = self.expected
            err = f"{meas.median - exp:.4f}" if exp is not None else ""
            pct = f"{100.0 * (meas.median - exp) / exp:.2f}" if exp else ""
            w.writerow([
                idx, img_name,
                datetime.now().isoformat(timespec="seconds"),
                f"{meas.median:.4f}",
                "" if exp is None else f"{exp:.4f}", err, pct,
                f"{meas.p75 - meas.p25:.4f}", f"{meas.std:.4f}", meas.count,
                "" if self.worker.roi is None else "|".join(str(v) for v in self.worker.roi),
            ])
        note = f"已存第 {idx} 筆：{img_name} + CSV → {self.outdir}"
        if not ok:
            note = f"⚠ 截圖存檔失敗（{img_name}），但 CSV 已記；資料夾：{self.outdir}"
        self.hint.setText(note)

    def keyPressEvent(self, e) -> None:
        if e.key() == Qt.Key_S:
            self._save_sample()
        else:
            super().keyPressEvent(e)

    def closeEvent(self, event) -> None:
        self.worker.stop()
        self.worker.wait(3000)
        super().closeEvent(event)


def main() -> int:
    ap = argparse.ArgumentParser(description="雙目測距驗證工具（框物件 → 量距離 → 比捲尺）")
    ap.add_argument("--cam0", help="cam0(左) 影片路徑；跟 --cam1 一起給＝用錄影而非即時鏡頭")
    ap.add_argument("--cam1", help="cam1(右) 影片路徑")
    ap.add_argument("--calib", default=str(_ROOT / DEFAULT.calib_path),
                    help="calib.npz 路徑（預設用專案根目錄的 calib.npz，不管從哪跑）")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="process_scale（預設 1.0 求準；調小換速度）")
    ap.add_argument("--outdir", default=str(_ROOT / "check_dist"),
                    help="截圖 + CSV 的輸出資料夾（預設專案根目錄的 check_dist/）")
    # 預設用「戶外白天」固定曝光，對齊上層 rpi5_dual_camera_capture.py 的 SHUTTER_US=2000/
    # GAIN=1.0（實測 Lux≈17000 戶外亮光下，自動曝光也收斂到 ≈2022µs/1.0）。兩顆鏡頭吃
    # 「同一組」才能保證雙目亮度一致(SGBM 匹配需要)。室內昏暗會太黑→調大(如 30000/5.0)。
    ap.add_argument("--shutter", type=int, default=2000,
                    help="即時鏡頭固定快門(µs)，兩顆共用。戶外白天 2000；室內昏暗調大(如 30000)")
    ap.add_argument("--gain", type=float, default=1.0,
                    help="即時鏡頭固定類比增益，兩顆共用。戶外 1.0；室內昏暗調大(如 5.0)")
    ap.add_argument("--expected", type=float, default=None,
                    help="實際距離(公尺，捲尺量的)；給了就在畫面與 CSV 顯示量測誤差")
    args = ap.parse_args()

    if bool(args.cam0) != bool(args.cam1):
        ap.error("--cam0 與 --cam1 要一起給")
    source = "video" if args.cam0 else "live"

    config = replace(DEFAULT, process_scale=args.scale,
                     live_shutter_us=args.shutter, live_gain=args.gain)
    calib = load_calibration(args.calib, process_scale=args.scale)
    nw, nh = calib.native_size
    pw, ph = calib.process_size
    print(f"[measure] source={source}  native={nw}x{nh}  process={pw}x{ph}"
          f"  baseline={calib.baseline_mm:.1f}mm")

    app = QApplication.instance() or QApplication(sys.argv)
    win = MainWindow(calib, config, source, args.cam0, args.cam1, args.outdir, args.expected)
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
