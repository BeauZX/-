# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

純雙目立體視覺**估計前方路面縱向坡度 (pitch)** 的專案（樹莓派 5 + Waveshare IMX219-83 雙鏡頭）。對相機前方 0.5–30m 的路面點雲擬**一個平面**，算出它相對相機的縱向坡度。主結果是**純雙目幾何**（左右兩顆鏡頭一起算，靠 62mm 基線得到公尺尺度的深度）；IMU 只是**輔助**（把相機安裝俯角補回去，換成相對水平面的絕對坡度）。

三種執行來源／呈現方式（見下）：離線影片、即時鏡頭純終端、即時鏡頭 + PyQt 視覺介面。沒有測試框架、沒有 lint，直接在 Pi 5 上跑。上層目錄（`../`）是產生 `calib.npz` 與錄影的相機/校正專案；本專案只**消費**它的 `calib.npz`，不修改它。

## 執行（一律透過 `main.py`，它只轉呼叫 `src.cli`）

```bash
# 即時鏡頭 + 視覺介面（最常用）：一開視窗自動跑，滑鼠拉框設 ROI、雙擊清除
python3 main.py --live --ui

# 即時鏡頭，純終端機印數字（較快、可 headless）
python3 main.py --live

# 離線：讀錄好的一段（segment 資料夾含 cam0.mp4/cam1.mp4，可選 pts/start_time.json）
python3 main.py <segment資料夾>
python3 main.py --cam0 a.mp4 --cam1 b.mp4        # 兩支 mp4 不在同一夾時
python3 main.py <segment資料夾> --limit 20        # 只跑前 N 幀

# 單獨檢查校正檔
python3 -m src.calib_loader calib.npz

# 語法檢查（唯一的自動化檢查手段，沒有 lint/test）
python3 -m py_compile src/*.py main.py
```

CLI 旗標只有 `--live --ui/--no-ui --calib --cam0 --cam1 --out --limit --quiet`。**所有演算法參數固定在 `src/config.py`**（`Config` dataclass + `DEFAULT`），要調就改那個檔、不從 CLI 傳（沿用上層 `calibrate_stereo.py` 的慣例）。`--record` 已移除：**即時模式一律錄影**（`config.record=True` 寫死）。

## 三種模式與它們的分岔點（`src/cli.py:main`）

- `--live` 決定**來源**：有 → Picamera2 即時鏡頭 (`src/live.py`)；無 → 讀影片 (`src/pairing.py`)。
- `--ui` 決定**呈現**：有 → PyQt 視窗 (`src/ui.py`)；無 → 終端機 + CSV。預設看 `config.show_ui`。
- 即時模式（不論有無 UI）都會經 `src/recorder.py` 把 cam0/cam1 錄到 **`output/segment_NNN/`**（遞增編號、**不覆蓋**），並輸出 `road_angle.csv`（含 `time_s`）+ `road_angle_trend.png`（matplotlib 趨勢圖）。

## Python 環境（非顯而易見，改動前必讀）

- **用系統 Python 3.13 + 系統 `cv2`/`av`**，透過 `.venv` 的 `include-system-site-packages = true` 繼承。`.venv` 這樣建：
  ```bash
  uv venv --python /usr/bin/python3 --system-site-packages
  ```
- **為什麼不能用 uv 隔離的 3.12 venv**：Pi 系統的 `cv2` 是為 Python **3.13** 編譯的 `.so`，ABI 綁死 3.13。用系統這顆 OpenCV 是刻意的——Pi 硬體最佳化過、跟上層相機主程式同一顆，比 pip 泛用 build 適合即時邊緣運算。`pyproject.toml` 的 `dependencies` **故意留空**、`requires-python = ">=3.13"`。
- **例外：`matplotlib` 有用 `uv pip install` 裝進 venv**（趨勢圖用，Agg backend）。它連帶把 **numpy 升到 2.5.1**（venv 內，蓋過系統 2.2.4）；已實測 cv2/av/SGBM 在 2.5.1 下相容。`picamera2` 只有系統有、只在 `--live` 才 import。

## 資料流與模組架構

`main.py` → `src/cli.py` → `src/pipeline.py`。每幀核心是 `estimate_from_result()`：

```
(來源) pairing.iter_pairs 或 live.LiveStereo.frames   左右幀對（同一瞬間 cam0+cam1）
  → disparity.compute_stereo / stereo_from_rectified   校正 → (ROI)SGBM 視差 → reprojectImageTo3D → 3D 點雲(公尺)
  → roadplane.select_road_points                       距離/高度過濾挑路面點
  → roadplane.fit_road_plane                           RANSAC 擬合 Y=aZ+bX+c → 坡度角 → FrameResult
```

- **`src/config.py`** — 全部固定參數單一來源。`StereoMatcher.from_config()`、`process_*`、`LiveStereo`、`recorder` 都從這裡取值。
- **`src/calib_loader.py`** — **只讀取** `calib.npz`（上層 `calibrate_stereo.py` 產生），建 remap 表 + `Q`。支援**降解析度**（見下）。本專案沒有 `calibration.py`；載入器刻意命名 `calib_loader.py` 跟上層「做校正」的工具區隔。
- **`src/disparity.py`** — `StereoMatcher`(StereoSGBM) + `StereoResult`。`compute_stereo` 一步做完；`stereo_from_rectified` 讓 UI「整張只 rectify 一次」還能單獨對 ROI 算視差。
- **`src/roadplane.py`** — 核心幾何（座標系/角度定義見下）。`plane_inlier_mask()` 產生「哪些像素在這片平面上」給 UI 塗綠。
- **`src/pairing.py`** — 離線左右幀配對（時間戳/序號，見下）。
- **`src/live.py`** — Picamera2 即時來源。`src/ui.py` — PyQt 視窗（背景 QThread 運算）。`src/recorder.py` — session 錄影+CSV+趨勢圖。`src/plot.py` — matplotlib 趨勢圖。`src/cli.py` — 分岔三模式 + 終端輸出。

## 加速：降解析度 + ROI（改動 disparity/calib 前必懂）

- `config.process_scale`（預設 0.5）：`calib_loader` 把 `P1/P2` 縮放 s、`Q` 對應縮放，讓 `initUndistortRectifyMap` **從原生 1280×720 直接吐出 640×360 的校正影像**（remap 一步完成去畸變+校正+縮小）。所以 `StereoCalibration` 有兩個尺寸：**`native_size`（相機/影片必須提供的原生解析度）** vs **`process_size`（實際算視差的縮小解析度）**。餵進 `rectify()` 的原圖必須是 native_size。
- `config.disparity_roi_fraction`（預設 0.55）或 UI 滑鼠框：只在校正影像的一塊區域算視差（省算力）。裁 ROI 後 `Q` 的 `cx/cy` 依裁切位移（`Q[0,3]+=x0`、`Q[1,3]+=y0`）維持幾何正確。
- `num_disparities` 以**原生解析度**定義（128），`from_config` 依 `process_scale` 自動縮放並取 16 倍數（縮小影像視差也等比例變小）。
- 實測：process 640×360 + ROI 下即時約 3–7fps（純 SGBM 軟體匹配，Pi 5 沒有硬體立體匹配）。

## 座標系與角度定義（roadplane.py 的核心）

- **相機座標（OpenCV）**：X→右、Y→**下**、Z→前。`reprojectImageTo3D(Q)` 的輸出即此系，單位公尺（已除 `depth_scale`）。
- **平面模型 `Y = a*Z + b*X + c`**（高度對前向 Z、橫向 X 的線性函數，比三參數法向量在近水平時穩定、不病態）。
- **坡度**：`pitch = atan(-a)`（上坡為正）、`roll = atan(-b)`（右高為正）；取負號因 Y 向下、路面上升＝Y 變小。`cam_height_m = -c`（**程式自動估**的離地高度，不是輸入參數）。
- **主結果是「相對相機光軸」的坡度**，不是相對水平面。相機裝車上有安裝俯角偏移，對已知平坦地面讀到的穩定值就是零點偏移，真實坡度 = 讀數 − 偏移。`to_gravity_referenced(plane, imu_pitch_deg)` 用 IMU 做此修正。

## 非顯而易見的限制

- **`calib.npz` 單位是 mm**（`SQUARE_SIZE_MM` 用 mm、`T`≈62mm 基線）→ `reprojectImageTo3D` 出來也是 mm。`config.depth_scale=1000.0` 負責 mm→m；**動它會讓所有深度/角度全錯**。
- **cam0=參考/左影像(`P1`)、cam1=右(`P2`, Tx 負)**。視差以 cam0 為基準；`Q` 還原成 cam0 校正座標系。UI 視窗顯示的底圖是 cam0 一張，但深度/角度是左右兩張一起算。
- **RMS 小 ≠ 角度可信**：牆面也是一片乾淨平面（RMS 小、內點高，pitch 卻 ±80°）。光靠 RMS/內點分不出「路面 vs 牆」，要靠幾何合理性（法向量方向/相機高度/pitch 範圍）或穩定的拍攝 + ROI 框。相機沒固定好時逐幀會鎖到不同的面、角度暴衝，這**不是 bug**。
- **左右幀配對兩種精度，`pairing.iter_pairs` 自動選**：有 `cam*_pts.txt`+`start_time.json` → 絕對時間戳配對（掉幀也對得回）；只有 mp4 → **序號配對**，一邊掉幀後會永遠錯開一幀而不報錯。pts 行序是編碼序需先排序、行數偶爾比可解碼幀多 1（`pairing.py` 已處理）。
- **即時沒有硬體幀同步**：兩顆 Picamera2 各自 `capture_array()`，左右差幾 ms，車速快時視差略誤。錄影用的 `--sync` 在即時串流沒有等價做法。
- **`live.py` 的解析度強制 = `calib.native_size`**；`config.live_shutter_us/live_gain` 固定曝光、AWB auto（雙目亮度一致）。Picamera2 index 0=i2c@88000=cam0(左)、1=i2c@80000=cam1(右)。
- **錄影名義 fps 固定 `record_fps`（實際變動）**，播放速度近似；要精準時間看 CSV 的 `time_s` 欄。

## 常見調參：症狀 → 改哪個 `config.py` 參數

| 症狀 | 調整 | 方向 |
|---|---|---|
| 太慢（fps 不夠） | `process_scale` | 調小（0.5→0.4）；或縮小 `disparity_roi_fraction`、UI 拉小 ROI 框 |
| 綠色太少 / 常「路面擬合失敗」 | `ransac_threshold_m` 放寬、`min_road_points` 調低 | 低紋理路面點少時 |
| 綠色摻到非路面雜訊 | UI 縮小 ROI 框、`ransac_threshold_m` 收緊、`z_max_m` 調小 | |
| 遠端讀數亂跳 | `z_max_m` 調小（30→20） | 遠處視差小、噪點多 |
| 最近的路面沒被測到 | `num_disparities` 調大（16 倍數） | 換來更慢 |
| 即時左右亮度不一致 | `live_shutter_us`/`live_gain` 固定曝光；AWB 維持 auto | 勿改固定色溫 |
| 角度暴衝（±80° 亂跳） | **不是調參問題**：相機沒固定/沒對著路面，見「非顯而易見的限制」的 RMS 說明 | 固定相機 + 拉 ROI 框住路面 |

## 零點偏移校正（尚未實作，之後要做）

主結果是「相對相機光軸」的坡度，相機裝車上有固定安裝俯角，所以平坦路面也讀到非零值。要變成「相對地面的真實坡度」需要**扣掉安裝偏移**——目前**還沒做成功能**，規劃如下：

1. 把相機固定好、對一段**已知平坦**的地面，跑即時，記下穩定的 pitch 讀數（例如 +15°）＝安裝零點偏移。
2. 之後可做成 `config.mount_pitch_offset_deg`，輸出時自動扣掉（真實坡度 = 讀數 − 偏移）。
3. 或改走 IMU 路線：用 `roadplane.to_gravity_referenced(plane, imu_pitch_deg)`，即時讀 9 軸 IMU (ICM20948) 的相機 pitch 做修正，免手動校零點。零點偏移法與 IMU 法二選一或並用。

實作其中任一個時，記得把本節從「尚未實作」更新成實際用法。

## 驗證方式

沒有自動化測試。驗證靠：(1) `py_compile` 抓語法/import；(2) 端到端跑真實 `calib.npz` + 鏡頭/錄影、檢查 CSV 讀數與 UI 綠色塗色是否合理。純算術（3D 點雲→RANSAC 平面→角度）可用**合成資料**驗證（給定已知坡度反推，不需實體棋盤格/相機）；GUI 邏輯可用 `QT_QPA_PLATFORM=offscreen` + 不 start worker 的方式煙霧測試。
