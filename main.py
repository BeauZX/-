"""road-angle 進入點。實際邏輯在 src 套件裡。

=== 常用指令（直接複製）===

即時鏡頭 + 視覺介面（最常用）。開窗先拉框設 ROI，按 Enter 才開始偵測+錄影：
    python3 main.py --live --ui

室內測試太暗時（只有這次執行變亮，src/config.py 的預設值不會被改）：
    python3 main.py --live --ui --shutter 20000 --gain 4.0

    調亮順序：先拉 --shutter，--gain 最後才動（增益會放大雜訊，讓 SGBM 更配不出
    低紋理路面的視差）。戶外預設 2000/1.0；室內明亮 10000/1.0、一般 20000/2.0、
    昏暗 30000/4.0。兩顆鏡頭一定吃同一組固定曝光——各自自動曝光會收斂到不同亮度，
    雙目就配不準（所以沒有「自動曝光」這個選項，只能換固定值）。

即時鏡頭，純終端機印數字（較快、看不到綠色內點；ROI 沿用 roi.json）：
    python3 main.py --live

離線跑一段錄影：
    python3 main.py SEG_DIR --calib calib.npz
    python3 main.py SEG_DIR --limit 20        # 只跑前 N 幀
或直接：
    python3 -m src.cli SEG_DIR --calib calib.npz

離線影片 + 視覺介面（另一個入口，不經本檔）：
    python3 run_video.py cam0.mp4 cam1.mp4

輸出都在 config.output_dir/segment_NNN/（即時）或 output_videos/（run_video.py）。
注意路徑是相對「執行時所在的目錄」，所以先 cd 到專案根目錄再跑。
"""

from src.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
