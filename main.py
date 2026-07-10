"""road-angle 進入點。實際邏輯在 src 套件裡。

離線跑一段錄影：
    python3 main.py SEG_DIR --calib calib.npz
或直接：
    python3 -m src.cli SEG_DIR --calib calib.npz
"""

from src.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
