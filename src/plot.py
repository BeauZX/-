"""用 matplotlib 畫角度趨勢圖並存成 PNG（Agg backend，免顯示器）。

看「前方路面坡度隨時間怎麼變」。輸入是每幀的 pitch（可含 roll），失敗幀為 None
會斷線留空。
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # 免 GUI，直接存檔
import matplotlib.pyplot as plt


def save_angle_trend(
    path: str,
    indices: list[int],
    pitches: list[float | None],
    rolls: list[float | None] | None = None,
    title: str = "Road pitch trend",
) -> None:
    """畫 pitch（與可選 roll）對 frame index 的折線圖，存到 path。"""
    fig, ax = plt.subplots(figsize=(10, 4.2), dpi=110)

    xp = [i for i, v in zip(indices, pitches) if v is not None]
    yp = [v for v in pitches if v is not None]
    ax.plot(xp, yp, color="#1a8a1a", lw=1.6, label="pitch (longitudinal)")

    if rolls is not None:
        xr = [i for i, v in zip(indices, rolls) if v is not None]
        yr = [v for v in rolls if v is not None]
        ax.plot(xr, yr, color="#f08a00", lw=1.2, alpha=0.8, label="roll (lateral)")

    if yp:
        mean = sum(yp) / len(yp)
        ax.axhline(mean, color="#1a8a1a", ls="--", lw=0.8, alpha=0.5,
                   label=f"pitch mean {mean:+.1f} deg")

    ax.axhline(0, color="#888", lw=0.8)
    ax.set_xlabel("frame")
    ax.set_ylabel("angle (deg)")
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
