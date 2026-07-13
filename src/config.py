"""固定參數集中地。

這套是單一固定硬體（IMX219 雙鏡頭、1280x720、基線 ~62mm），所有參數一次調好
就寫死在這裡，平常不用從 CLI 傳。要改就改這個檔（跟 calibrate_stereo.py 一樣的
慣例）。

注意：相機離地高度是程式從路面平面「自動估」的、IMU 俯角是逐幀讀的，兩者都不在
這裡——這裡只放真正「固定不變」的演算法參數。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    # === 校正檔 ===
    calib_path: str = "calib.npz"

    # === 加速：降解析度 + 只算路面 ROI ===
    # process_scale<1 時在縮小的校正影像上算視差（像素少、快很多），幾何由
    # calib_loader 縮放 P/Q 維持正確。0.5 → 640x360，約 4 倍加速。
    process_scale: float = 0.5
    # 只在畫面「下面這個比例」的橫帶上算視差（上方多是天空，浪費算力）。
    # 要略大於 image_bottom_fraction，給 SGBM 一點上下文。
    disparity_roi_fraction: float = 0.55

    # === 視差 (StereoSGBM) ===
    # num_disparities 以「原生解析度」為準，實際會依 process_scale 自動縮放並取
    # 16 倍數。128(原生) → 最近可測約 0.74m (Z=f*baseline/disp)。
    num_disparities: int = 128
    block_size: int = 5
    uniqueness_ratio: int = 10
    speckle_window_size: int = 100
    speckle_range: int = 2

    # === 3D 單位換算 ===
    # calib 的 T 單位是 mm（SQUARE_SIZE_MM 用 mm），reproject 出來也是 mm，
    # 除以這個值換成公尺。
    depth_scale: float = 1000.0

    # === 路面選點範圍（相機座標，公尺）===
    z_min_m: float = 0.5  # 最近距離（量「前方」路面，跳過腳下近路）
    z_max_m: float = 20.0  # 最遠距離（20m 內單點誤差 <~21%，擬平面後角度可靠）
    y_min_m: float = -0.5  # 路面在相機下方(Y 向下為正)，排除天空/高處
    # 註：影像橫帶的限制已由 disparity_roi_fraction 在算視差前處理，這裡不再重複裁列。

    # === RANSAC 平面擬合 ===
    ransac_threshold_m: float = 0.05  # 內點垂直殘差門檻 (5cm)
    ransac_iterations: int = 300
    min_road_points: int = 200  # 路面點數下限，不足視為擬合失敗
    seed: int = 0

    # === 即時鏡頭 (--live，Picamera2) ===
    # 兩顆鏡頭吃「同一組固定曝光」才能保證雙目亮度一致（AWB 維持 auto）。
    # 建議跟 rpi5_dual_camera_capture.py 錄影時的 SHUTTER_US/GAIN 一致。
    # 解析度不在這裡：即時強制用 calib.image_size（校正綁定的解析度）。
    live_shutter_us: int = 8000  # 固定快門 (µs)
    live_gain: float = 1.0  # 固定類比增益
    live_cam0_index: int = 0  # Picamera2 index：0 = i2c@88000 = cam0(左)
    live_cam1_index: int = 1  # 1 = i2c@80000 = cam1(右)

    # === 介面開關 ===
    # True → 即時模式開 PyQt 視覺視窗（影像+路面塗色+坡度角）；
    # False → 純終端機印數字（較快、可 headless）。CLI --ui / --no-ui 可覆寫。
    show_ui: bool = False

    # === 錄影 / session 輸出（寫死一律開啟）===
    # 即時模式一律把 cam0/cam1 錄到 output/segment_NNN/（遞增編號、不覆蓋），
    # 並輸出 road_angle.csv + road_angle_trend.png。
    record: bool = True
    output_dir: str = "output"
    record_fps: float = 10.0  # 影片名義 fps（實際變動，播放速度為近似）
    # 每滿這麼多秒就把當前 segment 收好（寫 CSV/趨勢圖、封 mp4）、換下一段遞增編號。
    # 好處：即時模式跑越久也不怕中途掛掉——最多只損失最後不到這段時間的資料。
    # 設 0（或負值）= 不輪替，維持舊行為（整段只在 Ctrl+C 停止時才一次寫出）。
    segment_seconds: float = 120.0  # 2 分鐘


# 全域預設；整個管線都用它，除非呼叫端特別覆寫。
DEFAULT = Config()
