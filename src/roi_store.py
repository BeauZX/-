"""記住上次 UI 拉的 ROI 框，下次開視窗直接套用。

存的是 process 座標的 (x0,y0,x1,y1) + 當時的 process_size。載入時比對 process_size，
不一致（換了 process_scale/解析度、舊框失去意義）就當作沒存、回 None。回 None ==
「用預設下方橫帶」，不論原因是沒存過、作廢、或使用者雙擊清除。
"""

from __future__ import annotations

import json
from pathlib import Path

Roi = tuple[int, int, int, int]


def save_roi(path: str | Path, roi: Roi | None, process_size: tuple[int, int]) -> None:
    """把目前 ROI（可為 None＝已清除）寫進 path。寫檔失敗不致命，只印警告。"""
    data = {
        "roi": list(roi) if roi is not None else None,
        "process_size": list(process_size),
    }
    try:
        Path(path).write_text(json.dumps(data))
    except OSError as e:
        print(f"[roi_store] 存 ROI 失敗（忽略）：{e}")


def load_roi(path: str | Path, process_size: tuple[int, int]) -> Roi | None:
    """讀回 ROI；檔案不存在/毀損/process_size 不符/已清除 都回 None。"""
    p = Path(path)
    if not p.exists():
        return None
    try:
        data = json.loads(p.read_text())
    except (OSError, ValueError):
        return None
    if list(process_size) != data.get("process_size"):
        return None  # 解析度變了，舊框作廢
    roi = data.get("roi")
    if not roi or len(roi) != 4:
        return None
    return tuple(int(v) for v in roi)
