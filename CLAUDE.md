# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

純雙目立體視覺**估計前方路面縱向坡度 (pitch)** 的專案（樹莓派 5 + Waveshare IMX219-83 雙鏡頭）。對相機前方一段路面（距離窗 `config.z_min_m`~`z_max_m`，目前預設 2–12m，見 `config.py`、隨路面素材調）的點雲擬**一個平面**，算出它相對相機的縱向坡度——所以主結果是「前方那段路的平均坡度」，非腳下、也非外插預測；調 `z_min/z_max` 即改「量多遠的前方」（遠處視差小、噪點多，`z_max` 上限受此限制）。主結果是**純雙目幾何**（左右兩顆鏡頭一起算，靠 62mm 基線得到公尺尺度的深度）；IMU 只是**輔助**——把相機自身 pitch 補回去、換成相對水平面的絕對坡度（`slope`＝`pitch_gravity_deg`）。**IMU 輔助已接上**：即時讀實體 ICM20948（`src/imu.py`）、離線影片讀當初錄影同步存下的 `imu_raw.csv`（`src/imu_track.py`），詳見下方 IMU 節。

四種執行來源／呈現方式（見下）：離線影片純終端 (`main.py`)、即時鏡頭純終端、即時鏡頭 + PyQt 視覺介面 (`main.py --live`)、**離線影片 + PyQt 視覺介面**（`run_video.py`：對錄好的影片圈 ROI、Enter 開始偵測、空白鍵切出想要的路段並輸出）。沒有測試框架、沒有 lint，直接在 Pi 5 上跑。上層目錄（`../`）是產生 `calib.npz` 與錄影的相機/校正專案；本專案只**消費**它的 `calib.npz`，不修改它。

## 執行（一律透過 `main.py`，它只轉呼叫 `src.cli`）

```bash
# 即時鏡頭 + 視覺介面（最常用）：開視窗先框選（拉框設 ROI、雙擊清除），按 Enter 才開始預測+錄影
python3 main.py --live --ui

# 即時鏡頭，純終端機印數字（較快、可 headless）
python3 main.py --live

# 離線：讀錄好的一段（segment 資料夾含 cam0.mp4/cam1.mp4，可選 pts/start_time.json）
python3 main.py <segment資料夾>
python3 main.py --cam0 a.mp4 --cam1 b.mp4        # 兩支 mp4 不在同一夾時
python3 main.py <segment資料夾> --limit 20        # 只跑前 N 幀

# 離線影片 + 視覺介面：對錄好的影片圈 ROI、Enter 開始偵測、空白鍵切出想要的路段
python3 run_video.py cam0.mp4 cam1.mp4
python3 run_video.py                             # 不給參數用 run_video.py 開頭寫死的預設路徑

# 單獨檢查校正檔
python3 -m src.calib_loader calib.npz

# IMU 自測/校零（即時讀實體 ICM20948 的相機 pitch：驗證軸向正負、校安裝零點；見 IMU 節）
python3 -m src.imu

# 雙目測距驗證（獨立工具，不在主管線內；框物件量距離比對捲尺，見「測距驗證工具」節）
python3 tool/measure_distance.py

# 語法檢查（唯一的自動化檢查手段，沒有 lint/test）
python3 -m py_compile src/*.py main.py run_video.py tool/measure_distance.py
```

CLI 旗標只有 `--live --ui/--no-ui --calib --cam0 --cam1 --out --limit --quiet`。**所有演算法參數固定在 `src/config.py`**（`Config` dataclass + `DEFAULT`），要調就改那個檔、不從 CLI 傳（沿用上層 `calibrate_stereo.py` 的慣例）。`--record` 已移除：**即時模式一律錄影**（`config.record=True` 寫死）。

## 三種模式與它們的分岔點（`src/cli.py:main`）

- `--live` 決定**來源**：有 → Picamera2 即時鏡頭 (`src/live.py`)；無 → 讀影片 (`src/pairing.py`)。
- `--ui` 決定**呈現**：有 → PyQt 視窗 (`src/ui.py`)；無 → 終端機 + CSV。預設看 `config.show_ui`。
- **UI 兩階段（Enter 閘門）**：`StereoWorker.active` 初始 False＝框選階段（只 rectify + 畫 ROI 框，**跳過 SGBM/擬合/錄影**，預覽更順好瞄準）；主執行緒 `keyPressEvent` 收到 Enter 設 `active=True` 才進預測階段（錄影器此時才**延後建立**，故框選過程不會產生空 segment）。**此閘門只在 UI**；headless `--live` 走 `pipeline.process_live`、不經本 worker，一律直接跑、無需 Enter。
- 即時模式（不論有無 UI）都會經 `src/recorder.py` 把 cam0/cam1 錄到 **`output/segment_NNN/`**（遞增編號、**不覆蓋**），並輸出 `road_angle.csv`（含 `time_s`）+ `road_angle_trend.png`（matplotlib 趨勢圖）。
- **段落每 `config.segment_seconds`（預設 60 秒）自動輪替**：`SessionRecorder.add()` 每幀檢查牆鐘時間，滿了就 `_finalize_segment()`（寫 CSV/趨勢圖、封 mp4）再開下一段 `segment_NNN`。好處是即時跑越久越不怕中途 `kill -9`/當機——CSV/PNG/mp4 都是段落結束才落地，輪替讓**最多只損失最後不到一段的資料**。用牆鐘計時（非幀數），故每段都是真的 N 秒、段內幀數隨當下 fps 變動；**每段 `time_s` 從 0 重新計**。設 `segment_seconds=0` 關閉輪替＝舊行為（只在 Ctrl+C 停止時一次寫出）。

## 第四種模式：離線影片 + UI（`run_video.py`，跟即時 UI 刻意分開）

這條路**不經 `main.py`/`src/cli.py`**，是獨立入口：`run_video.py` → `src/video_ui.py`。刻意**不共用** `src/ui.py`（即時），因為操作流程不同（多了「空白鍵切段」與「偵測疊圖影片」）；兩者只共用通用顯示元件（`ui.py` 的 `VideoLabel` 滑鼠拉框、`_as_bgr`/`_compose_lr`/`_panel_label` 疊圖、`DISPLAY_SCALE`）。**偵測畫面的文字是 `video_ui.py` 自己的 `_draw_slope`（精簡版），不用 `ui.py:_draw_text`**——改 `ui.py` 那幾個共用元件前要知道 `video_ui.py` 有 import。

- **來源是 `src/video_source.py:VideoStereo`**：把兩支 mp4 包成「介面跟 `LiveStereo` 一樣」的 context manager（`frames()` 吐 native 尺寸 BGR 幀對），串流解碼、不一次載入整支，開視窗不卡。用序號配對（第 i 幀對第 i 幀），非 `pairing.iter_pairs` 的時間戳配對（互動預覽夠用）。另有 `first_frame()` 只解第一幀。
- **框選階段「定格」在影片第一幀**（`VideoWorker.run` 前半段）：`first_frame()` 取靜止底圖、Enter 前一直重畫同一張（影像不動好瞄準，拖曳中的黃框每次重畫都更新）。**按 Enter 才用 `VideoStereo(..., loop=True)` 從頭播放** + 偵測（`loop=True` 只用在偵測階段，讓短影片跑完自動回頭直到空白鍵）。注意與即時 `ui.py` 的差異：即時框選階段是鏡頭即時畫面在動，這裡是**定格**。
- **偵測畫面只留精簡三項**（`_draw_slope`）：`slope`（大字＝主結果，前方路面相對水平坡度；無 IMU 時退回 `pitch`）＋一行 `h .. m  roll ..`（相機估計高度＋橫向坡度，當「這片是路面不是牆」的 sanity 燈）。`pitch`(純雙目)/`RMS`/`inliers`/`fps`/`imu` 分量**只寫進 `road_angle.csv`、不上螢幕**。文字顏色仍隨可信度（RMS 小+內點高→綠、否則橘）。
- **ROI 中心距離讀數只在框選階段**：框選畫面左上疊 `ROI center ~ X.X m`＝ROI 框中心的前向距離 Z（`_patch_depth_m` 取中心小區塊中位 Z），對定格幀**整張算一次視差**（`stereo_from_rectified(..., roi=(0,0,pw,ph))`）供拖框時即時查。用途：直接看出「ROI 框太遠/太近」——若中心距離落在 `z_min_m~z_max_m` 窗外，選點全被距離窗濾掉、每幀擬合失敗（畫面顯示紅色 `？？？？`）。**偵測階段畫面不再顯示這個**（已由 `_draw_slope` 取代成 slope/h/roll）。
- **操作流程**：圈 ROI（存 `roi.json`，跟即時共用同一個框）→ **Enter** 進偵測（`VideoWorker.active=True`，此時才建 `SessionRecorder` + `detect.mp4` writer + 載入 `ImuTrack`）→ **空白鍵** `stop()` 收尾。切出的是「Enter 到空白鍵」這一段。
- **IMU 輔助（離線）**：`run_video.py` 的 config 用 `dataclasses.replace(DEFAULT, segment_seconds=0.0, output_dir="output_videos", use_imu=True)`。`use_imu=True` 讓 `VideoWorker` 開偵測時 `ImuTrack.load(cam0, config)` 去讀 **cam0.mp4 同資料夾的 `imu_raw.csv`**（路徑自動推導、不寫死），逐幀補算 `pitch_gravity`。旁邊沒這檔／欄位不對就**靜默退回純雙目**（畫面剩 `pitch`、無 `slope`）。細節見 IMU 節。
- **輸出到 `output_videos/segment_NNN/`**（**不切段**故整段一個 segment、一張趨勢圖；另開資料夾跟即時錄影的 `output/` 分流）。每段比即時多一個 `detect.mp4`＝疊了綠色路面+精簡坡度文字的偵測畫面（process 解析度 ×`DISPLAY_SCALE`）；`cam0/cam1.mp4` 是那段的原影片、`road_angle.csv`/`road_angle_trend.png` 同即時（有 IMU 時趨勢圖畫 slope，見 IMU 節）。

## 測距驗證工具（`tool/measure_distance.py`，獨立於主管線）

驗證雙目深度準不準的**獨立小工具**，跟主專案（估路面坡度）目的不同：框一個物件 → 雙目量它的距離 → 使用者拿捲尺量實際值比對誤差。**只消費 `src` 的 `calib_loader`/`disparity`/`live`/`video_source`，並沿用 `src/ui.py` 的 `_as_bgr`/`_compose_lr`/`_panel_label`**（`FitVideoLabel` 是它自己的滑鼠拉框元件，不共用 `ui.py` 的 `VideoLabel`，因為顯示縮放不同）。不改 `src`。

- **量測公式與主管線完全相同**：一樣走 `disparity.py` 的 rectify → SGBM → `reprojectImageTo3D(Q)` → ÷`depth_scale`。差別只在「事後拿 3D 點做什麼」——這裡取 **ROI 內有效點的中位 Z** 當距離（非擬平面），以及 **`process_scale` 預設 1.0**（求準，主專案即時預設 0.5 求快）。要跟主專案等價比較就 `--scale 0.5`。
- **入口自足**：`_ROOT = Path(__file__).parent.parent` 把專案根加進 `sys.path` 並鎖定 `calib.npz` 為 `_ROOT/calib.npz`——**不管從哪個 cwd 執行都找得到校正檔**（曾踩過在 `tool/` 裡跑找不到 `calib.npz` 的坑）。
- **只能框左圖(cam0)**：測距以 cam0 為基準座標系，`self.roi[0] >= process_width` 即判定框在右圖(cam1)、提示 `Draw the box on the LEFT (cam0) image`；但深度是 cam0+cam1 兩張一起算（雙目本質），cam0 只是結果座標系，非「只用 cam0」。
- **兩個防呆(改 worker 前要知道)**：(1) 顯示縮放 `disp_scale = min(2.0, MAX_DISPLAY_WIDTH/combo_w)`——全解析度並排若直接 ×2 會變 ~5128px 撐爆視窗（黑畫面），故縮到總寬 ≤1400；滑鼠座標同除此 scale。(2) 框寬 < `num_disparities+block_size` 時**跳過 SGBM**（否則 OpenCV 算出負寬度爆記憶體 OOM），提示 `ROI too narrow`。
- **曝光旗標獨立於錄影**：`--shutter`/`--gain`（預設 30000/5.0，室內偏亮）用 `dataclasses.replace` 套進**副本 config**，不動 `src/config.py`，故**不影響 `main.py --live` 錄影的曝光**（那個仍讀 `config.py` 的 8000/1.0）。兩顆鏡頭吃同一組固定曝光（雙目亮度一致 SGBM 才配得準）。
- **輸出**：按 `s` 用 `QWidget.grab()` 截整個視窗存 `check_dist/sample_NNN.png`（**遞增、不覆蓋、跨執行接續編號**：啟動時掃現有 `sample_*.png` 取 max+1）+ 追加一列到 `check_dist/distance_log.csv`（含中位距離、IQR、std、點數）。畫面綠字只顯示 `Distance` + `fps`（簡報乾淨），IQR/std/n 仍寫進 CSV 備查。`check_dist/` 已 gitignore。

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
- **`src/live.py`** — Picamera2 即時來源。`src/ui.py` — 即時 PyQt 視窗（背景 QThread 運算）。`src/recorder.py` — session 錄影+CSV+趨勢圖。`src/plot.py` — matplotlib 趨勢圖。`src/cli.py` — 分岔 `main.py` 三模式 + 終端輸出。
- **`src/video_source.py`** — 離線影片來源 `VideoStereo`（介面同 `LiveStereo`，循環播放）。**`src/video_ui.py`** — 影片專用 PyQt 視窗（`VideoWorker`/`VideoWindow`，Enter 開始/空白鍵切段+`detect.mp4`），由 `run_video.py` 啟動，跟 `ui.py` 分開（見「第四種模式」）。
- **`src/imu.py`** — 即時 IMU 輔助：背景 thread 讀實體 ICM20948、互補濾波算相機 pitch（`ImuReader.pitch_deg`）。`src/imu_track.py` — 離線 IMU 輔助：讀錄影同步存的 `imu_raw.csv`、對齊到每個影片幀（`ImuTrack.pitch_for_frame`）。兩者都是**輔助**、缺了就退回純雙目（見 IMU 節）。`FrameResult.with_imu()` 把相機 pitch 補成 `pitch_gravity_deg`。

## 加速：降解析度 + ROI（改動 disparity/calib 前必懂）

- `config.process_scale`（預設 0.5）：`calib_loader` 把 `P1/P2` 縮放 s、`Q` 對應縮放，讓 `initUndistortRectifyMap` **從原生 1280×720 直接吐出 640×360 的校正影像**（remap 一步完成去畸變+校正+縮小）。所以 `StereoCalibration` 有兩個尺寸：**`native_size`（相機/影片必須提供的原生解析度）** vs **`process_size`（實際算視差的縮小解析度）**。餵進 `rectify()` 的原圖必須是 native_size。
- `config.disparity_roi_fraction`（預設 0.55）或 UI 滑鼠框：只在校正影像的一塊區域算視差（省算力）。裁 ROI 後 `Q` 的 `cx/cy` 依裁切位移（`Q[0,3]+=x0`、`Q[1,3]+=y0`）維持幾何正確。
- **UI ROI 會被記住、且 headless 也吃**：UI 拉框設好的 ROI 經 `src/roi_store.py` 存到 `config.roi_path`（預設 `roi.json`，已 gitignore）。**兩個即時入口都會在啟動時 `load_roi` 套用它**——`--live --ui`（`ui.py` MainWindow）與純終端 `--live`（`cli._run_live` → `process_live(..., roi=)`），所以「UI 框一次 → 之後直接 `python3 main.py --live` 」成立，兩模式共用同一個框。存 process 座標 + `process_size` 防呆：換 `process_scale`/解析度使舊框失效時 `load_roi` 回 None＝退回預設橫帶（`disparity_roi_fraction`）。ROI 是 process 座標，因透視其「垂直位置」約略對應前方距離（框低＝近、框高＝遠），需與 `z_min_m/z_max_m` 講一致點才留得下。要 headless 回到預設橫帶：UI 雙擊清除、或刪 `roi.json`。
- `num_disparities` 以**原生解析度**定義（128），`from_config` 依 `process_scale` 自動縮放並取 16 倍數（縮小影像視差也等比例變小）。
- 實測：process 640×360 + ROI 下即時約 3–7fps（純 SGBM 軟體匹配，Pi 5 沒有硬體立體匹配）。

## 座標系與角度定義（roadplane.py 的核心）

- **相機座標（OpenCV）**：X→右、Y→**下**、Z→前。`reprojectImageTo3D(Q)` 的輸出即此系，單位公尺（已除 `depth_scale`）。
- **平面模型 `Y = a*Z + b*X + c`**（高度對前向 Z、橫向 X 的線性函數，比三參數法向量在近水平時穩定、不病態）。
- **坡度**：`pitch = atan(-a)`（上坡為正）、`roll = atan(-b)`（右高為正）；取負號因 Y 向下、路面上升＝Y 變小。`cam_height_m = -c`（**程式自動估**的離地高度，不是輸入參數）。
- **主結果是「相對相機光軸」的坡度**，不是相對水平面。相機裝車上有安裝俯角偏移，對已知平坦地面讀到的穩定值就是零點偏移，真實坡度 = 讀數 − 偏移。`to_gravity_referenced(plane, imu_pitch_deg)` 用 IMU 做此修正。

## 非顯而易見的限制

- **`calib.npz` 單位是 mm**（`SQUARE_SIZE_MM` 用 mm、`T`≈62mm 基線）→ `reprojectImageTo3D` 出來也是 mm。`config.depth_scale=1000.0` 負責 mm→m；**動它會讓所有深度/角度全錯**。
- **cam0=參考/左影像(`P1`)、cam1=右(`P2`, Tx 負)**。視差以 cam0 為基準；`Q` 還原成 cam0 校正座標系。即時 UI (`ui.py`) 把 cam0(左)/cam1(右)兩顆校正後影像**並排顯示**（避免誤會只用單顆），但綠色路面內點只疊在左圖(cam0)——mask 是 cam0 像素座標，右圖同像素被視差平移、塗上去會錯位；右圖只畫黃色 ROI 框。滑鼠拉框設 ROI 只在左圖座標系生效。深度/角度是左右兩張一起算。（`video_ui.py` 離線 UI 也同樣並排顯示 cam0/cam1，並沿用 `ui.py` 的 `_as_bgr`/`_compose_lr`/`_panel_label`；`detect.mp4` 因此也是並排畫面。）
- **RMS 小 ≠ 角度可信**：牆面也是一片乾淨平面（RMS 小、內點高，pitch 卻 ±80°）。光靠 RMS/內點分不出「路面 vs 牆」，要靠幾何合理性（法向量方向/相機高度/pitch 範圍）或穩定的拍攝 + ROI 框。相機沒固定好時逐幀會鎖到不同的面、角度暴衝，這**不是 bug**。
- **左右幀配對兩種精度，`pairing.iter_pairs` 自動選**：有 `cam*_pts.txt`+`start_time.json` → 絕對時間戳配對（掉幀也對得回）；只有 mp4 → **序號配對**，一邊掉幀後會永遠錯開一幀而不報錯。pts 行序是編碼序需先排序、行數偶爾比可解碼幀多 1（`pairing.py` 已處理）。
- **即時沒有硬體幀同步**：兩顆 Picamera2 各自 `capture_array()`，左右差幾 ms，車速快時視差略誤。錄影用的 `--sync` 在即時串流沒有等價做法。
- **`live.py` 的解析度強制 = `calib.native_size`**；`config.live_shutter_us/live_gain` 固定曝光、AWB auto（雙目亮度一致）。Picamera2 index 0=i2c@88000=cam0(左)、1=i2c@80000=cam1(右)。
- **錄影名義 fps 固定 `record_fps`（實際變動）**，播放速度近似；要精準時間看 CSV 的 `time_s` 欄。
- **UI 疊字用 `cv2.putText` 的 Hershey 字型，畫不出中文**：即時 `ui.py:_draw_text` 與離線 `video_ui.py:_draw_slope` 的失敗訊息「路面擬合失敗」在畫面上都會變成一串紅色 `??????`（每個中文字一個 `?`）。這**不是當機**，就是「這一幀沒擬出平面」的指示（多半因場景裡沒有點落在 `z_min~z_max`，例如室內近物配 `z_min=5m`）。要讓它可讀就把訊息改英文，或換能畫中文的繪字方式。

## 常見調參：症狀 → 改哪個 `config.py` 參數

| 症狀 | 調整 | 方向 |
|---|---|---|
| 太慢（fps 不夠） | `process_scale` | 調小（0.5→0.4）；或縮小 `disparity_roi_fraction`、UI 拉小 ROI 框 |
| 綠色太少 / 常「路面擬合失敗」 | `ransac_threshold_m` 放寬、`min_road_points` 調低 | 低紋理路面點少時 |
| 綠色摻到非路面雜訊 | UI 縮小 ROI 框、`ransac_threshold_m` 收緊、`z_max_m` 調小 | |
| 遠端讀數亂跳 | `z_max_m` 調小（如 20→15） | 遠處視差小、噪點多 |
| 想量更遠的前方又要精度 | `process_scale` 調回 1.0（全解析度算視差） | 深度誤差約減半、可拉大 `z_max_m`，代價是慢很多 |
| 最近的路面沒被測到 | `num_disparities` 調大（16 倍數） | 換來更慢 |
| 即時左右亮度不一致 | `live_shutter_us`/`live_gain` 固定曝光；AWB 維持 auto | 勿改固定色溫 |
| 角度暴衝（±80° 亂跳） | **不是調參問題**：相機沒固定/沒對著路面，見「非顯而易見的限制」的 RMS 說明 | 固定相機 + 拉 ROI 框住路面 |

## IMU 輔助（把「相對相機光軸」換成「相對水平面」的真實坡度）

主結果 `pitch_deg` 是相機相對坡度，相機裝車上有安裝俯角＋行駛中車體會俯仰，所以要 IMU 提供「相機自身相對水平的 pitch」，補成真實坡度：**`pitch_gravity_deg`（畫面上叫 `slope`）＝ `pitch_deg` ＋ `imu_pitch_deg`**（`roadplane.to_gravity_referenced` / `FrameResult.with_imu`）。IMU 是**輔助**，缺了整條管線照跑純雙目、行為不變。

**兩個 IMU 來源（不同時刻的資料，不可混用）：**
- **即時 `src/imu.py:ImuReader`** — 背景 thread 讀**現在插著的**實體 ICM20948，互補濾波（`imu_filter_alpha`）算相機 pitch。`--live` 且 `config.use_imu=True` 時由 `pipeline.process_live` 每幀取最新值。缺函式庫/I2C 斷線→`available=False`、`pitch_deg` 回 None，優雅退場。
- **離線 `src/imu_track.py:ImuTrack`** — 影片是「別的時間錄的」，當下感測器讀值用不上；改讀當初錄影**同步存下的 `imu_raw.csv`**（上層 `rpi5_dual_camera_capture.py` 產生，跟 cam0.mp4 同資料夾）。`run_video.py` 走這條（`use_imu=True`）。用 `pts`+`start_time.json` 把每筆 IMU 對到最近的影片幀（沿用 `pairing._load_sorted_pts`/`_nearest_idx`），`loop` 播放靠取模循環對應；缺 pts 時退化成序號等比例對應。

**非顯而易見的坑（改 IMU 相關前必讀）：**
- **`imu_raw.csv` 的軸命名錯位**：上層錄影程式用**感測器本體座標**命名，實測（2026-07，晶片側貼）「上層的 `roll` 其實是車輛 pitch（上下坡）、上層的 `pitch` 其實像 yaw」。**已把該 CSV 欄位改名**成 `pitch_deg`(=車輛俯仰)/`yaw_deg`，`ImuTrack` 直接讀改名後的 `pitch_deg` 欄。若拿到**沒改名**的舊 CSV（只有原始 `roll_deg`/`pitch_deg`），`ImuTrack` 找不到可用 `pitch_deg`→回 None。
- **`ImuTrack` 直接用 CSV 的 `pitch_deg` 欄、不再套 config 的 `imu_invert_pitch`/`imu_mount_pitch_offset_deg`**：那欄是錄影程式已互補濾波＋套過自己零點的值（非生 atan2），正負號與即時 `imu.py` 一致（都 `atan2(ay,az)+gx`）；config 那組偏移是給「生 atan2 歸零」用的（即時 `ImuReader` 才套），重複套會雙重扣。驗證：對生值重算 `atan2(ay,az)` 中位 ≈ -94°，跟 `python3 -m src.imu` 靜置讀值吻合＝同一套慣例。
- **靜默退回**：`ImuTrack.load` 找不到 `imu_raw.csv` 時**目前不印警示**、直接回 None。判斷有沒有吃到 IMU 看：終端機有無 `[imu_track] 已載入…`、畫面標題是 `slope`(有)還是 `pitch`(無)、CSV 的 `pitch_gravity_deg` 欄有沒有值。
- **趨勢圖跟著 IMU 切換**：`recorder._write_trend` 若任一幀有 `pitch_gravity_deg` 就畫 slope（標題 `Road slope trend (gravity-referenced)`、綠線落在真實坡度附近），否則畫純雙目 pitch（含安裝俯角、綠線在 ~+15°）。

**還沒做：平地零點校正。** 上述 `slope` 仍帶約 1~2° 的殘餘安裝俯角零點（即時法與離線法各自的零點來源不同）。要讓「平路≈0°」還缺一步：把相機固定、對**已知平坦**地面跑一次、記下穩定的 `slope` 讀數 G，做成 `config.mount_pitch_offset_deg` 輸出時自動扣掉（真實坡度 = slope − G）。`config.imu_mount_pitch_offset_deg` 目前只校到「IMU 生值靜置歸零」、不是這個平地零點。**實作平地零點時記得更新本段。**

## 驗證方式

沒有自動化測試。驗證靠：(1) `py_compile` 抓語法/import；(2) 端到端跑真實 `calib.npz` + 鏡頭/錄影、檢查 CSV 讀數與 UI 綠色塗色是否合理。純算術（3D 點雲→RANSAC 平面→角度）可用**合成資料**驗證（給定已知坡度反推，不需實體棋盤格/相機）；GUI 邏輯可用 `QT_QPA_PLATFORM=offscreen` + 不 start worker 的方式煙霧測試。
