"""IMU 輔助：讀 ICM20948（雙目模組內建的 9 軸感測器），互補濾波算相機自身的
縱向 pitch，用來把「相對相機光軸」的路面坡度換成「相對水平面（重力）」的真實坡度。

演算法（互補濾波）沿用上層錄影程式 rpi5_dual_camera_capture.py 的 imu_process：
加速度計量重力方向 → 絕對傾角（不漂但受車身加減速污染），陀螺儀積分 → 短期平滑
（會漂），兩者用係數 `imu_filter_alpha` 融合。**pitch 的基準來自加速度計**，陀螺
儀只補短期抖動——這正是「用加速度計的角度、不是生陀螺儀」的標準寫法。

跟上層的差異：
- 上層是獨立的 mp.Process 把每筆即時寫 CSV；這裡只需要「當下最新的 pitch」餵給
  每一幀雙目結果，所以用背景 **thread**（同 process、共享記憶體），主迴圈（相機）
  隨時讀最新值即可。
- 缺函式庫 / 硬體讀不到時**優雅退場**：`available=False`、`pitch_deg` 回 None，
  管線照跑純雙目主結果，只是不做重力修正。不讓 IMU 故障拖垮整條路面偵測。

**軸向與零點要在真實硬體上校**（見模組末 __main__ 自測與 config 註解）：ICM20948
在模組上的貼片方向不保證跟這裡假設一致（上層實測 roll 靜置值 ≈ -88.8°，暗示晶片
是側貼/旋轉安裝），對應到「車輛縱向 pitch」的軸與正負號必須實測確認。
"""

from __future__ import annotations

import math
import threading
import time

from .config import Config


class ImuReader:
    """背景執行緒持續讀 ICM20948、互補濾波，隨時提供最新的相機 pitch。

    with ImuReader(config) as imu:
        for ...:
            p = imu.pitch_deg   # float（相機相對水平的 pitch）或 None（IMU 不可用）

    `pitch_deg` 已套用 config.imu_invert_pitch（軸向反轉）與
    config.imu_mount_pitch_offset_deg（IMU 軸↔相機光軸的固定偏移，停平地校一次），
    可直接餵給 roadplane.to_gravity_referenced()。
    """

    def __init__(self, config: Config) -> None:
        self.config = config
        self.available = False
        self._imu = None
        self._pitch: float | None = None  # 融合後的原始 pitch（未套 invert/offset）
        self._yaw: float | None = None  # 融合後的 yaw（只供自測辨識軸向，管線不用）
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # --- 生命週期 ---
    def __enter__(self) -> "ImuReader":
        if not self.config.use_imu:
            return self  # 停用：available 維持 False，pitch_deg 回 None
        try:
            from icm20948 import ICM20948  # 系統 Python 才有，延後 import
        except ImportError:
            print("[imu] 找不到 icm20948 函式庫（pip3 install icm20948）；停用 IMU 修正。")
            return self
        try:
            self._imu = ICM20948()
        except Exception as e:  # I2C 斷線 / 位址掃不到等
            print(f"[imu] ICM20948 初始化失敗：{e}；停用 IMU 修正。")
            return self
        self.available = True
        self._thread = threading.Thread(target=self._loop, name="imu", daemon=True)
        self._thread.start()
        print(f"[imu] 已啟用（互補濾波 alpha={self.config.imu_filter_alpha}, "
              f"offset={self.config.imu_mount_pitch_offset_deg:+.2f}°）")
        return self

    def __exit__(self, *exc) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    # --- 讀值 ---
    @property
    def pitch_deg(self) -> float | None:
        """最新的相機相對水平 pitch（已套 invert + 固定偏移）；不可用時回 None。"""
        if not self.available:
            return None
        with self._lock:
            p = self._pitch
        if p is None:
            return None
        if self.config.imu_invert_pitch:
            p = -p
        return p + self.config.imu_mount_pitch_offset_deg

    # --- 背景取樣迴圈（演算法核心，沿用上層 imu_process）---
    def _loop(self) -> None:
        interval = 1.0 / self.config.imu_sample_hz
        alpha = self.config.imu_filter_alpha
        pitch = 0.0
        last_t: float | None = None
        while not self._stop.is_set():
            loop_start = time.time()
            try:
                ax, ay, az, gx, gy, gz = self._imu.read_accelerometer_gyro_data()
                t = time.time()
                # ── 命名對應「車輛」的實際動作（實測 2026-07，晶片側貼）──
                # 上層 rpi5_dual_camera_capture.py 用感測器本體座標命名，但這台側貼後
                # 錯位：上層的 roll(atan2(ay,az)+gx) 實際是車輛「pitch 俯仰＝上下坡」，
                # 上層的 pitch(atan2(-ax,..)+gy) 實際像車輛「yaw 偏擺」。故這裡直接以
                # 車輛動作命名——pitch=上下坡(要用的)、yaw=偏擺(只供辨識，管線不用)。
                pitch_acc = math.degrees(math.atan2(ay, az))  # 車輛俯仰（上下坡）
                yaw_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))  # 像偏擺
                dt = (t - last_t) if last_t is not None else 0.0
                last_t = t
                if dt > 0:
                    # 互補濾波：陀螺儀積分(短期) + 加速度計(長期)。
                    # pitch 配 gx（繞晶片 X 軸＝車輛俯仰）、yaw 配 gy（繞晶片 Y 軸＝垂直）。
                    pitch = alpha * (pitch + gx * dt) + (1 - alpha) * pitch_acc
                    yaw = alpha * (yaw + gy * dt) + (1 - alpha) * yaw_acc
                else:
                    pitch, yaw = pitch_acc, yaw_acc
                with self._lock:
                    self._pitch = pitch
                    self._yaw = yaw
            except Exception as e:
                print(f"[imu] 讀取失敗：{e}", flush=True)
                time.sleep(0.5)
            sleep_t = interval - (time.time() - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)


def _selftest() -> None:
    """python3 -m src.imu ——即時印 pitch/roll，用來(1)驗證軸向正負(2)校零點偏移。

    校零流程：把相機固定好、對一段確定平坦的地面，跑 `python3 main.py --live`（先把
    config.use_imu 設 True），記下穩定的 pitch_gravity 讀數 G，再把
    config.imu_mount_pitch_offset_deg 設成 -G，之後平地就讀 0、下坡為負、上坡為正。
    這支自測則是更底層地確認「傾斜相機時 pitch 有沒有照預期方向變」。
    """
    from dataclasses import replace

    from .config import DEFAULT

    cfg = replace(DEFAULT, use_imu=True)  # 用真實 config（含剛校好的 offset/invert）驗證
    with ImuReader(cfg) as imu:
        if not imu.available:
            print("IMU 不可用，結束。")
            return
        print("即時 pitch（Ctrl+C 結束）。靜置應接近 0；抬車頭應為正、低頭為負：")
        try:
            while True:
                with imu._lock:
                    raw, yaw = imu._pitch, imu._yaw
                cal = imu.pitch_deg
                if raw is None:
                    print("\r-- ", end="", flush=True)
                else:
                    print(f"\r原始={raw:+7.2f}°  校正後 pitch={cal:+7.2f}°  (yaw={yaw:+7.2f}°)   ",
                          end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n結束。")


if __name__ == "__main__":
    _selftest()
