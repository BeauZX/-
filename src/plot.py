"""用 matplotlib 畫角度趨勢圖並存成 PNG（Agg backend，免顯示器）。

看「前方路面坡度隨時間怎麼變」。輸入是每幀的 pitch（可含 roll），失敗幀為 None
會斷線留空。

圖的重點是**一眼看出上坡還是下坡**，所以畫的**不是原始讀數、而是「扣掉基準」的偏差**
——這樣 **0 就是平路、正數＝上坡、負數＝下坡**，跟直覺一致。原始 pitch 含固定的
相機安裝俯角（實測落在 +9~+15°），直接畫會整條線浮在 +10° 附近、看不出上下坡。

三層畫法：
  1. 淡色細線＝原始逐幀偏差（保留真實雜訊，不隱藏資料）
  2. 深色粗線＝滾動**中位數**平滑（擬合退化時會冒單幀暴衝，中位數不被尖刺拉走）
  3. 0 線上下著色：平滑線在 0 之上塗綠(UPHILL +)、之下塗橘(DOWNHILL −)

**基準怎麼來的決定了這張圖能不能信**（呼叫端 `recorder._write_trend` 決定，扣掉的
數值一定印在圖片下緣）：
  - 有 IMU → 基準＝0（真正的水平面），上下坡是絕對的
  - 無 IMU → 基準＝**本段中位數**，因為純雙目讀數含固定的相機安裝俯角、零點未知。
    此時上下坡是**相對本段平均**的，整段都是下坡的話仍會被畫成平的——這個限制
    直接寫在圖上，別讓人誤讀。要絕對零點得做 CLAUDE.md 講的平地零點校正。

文字一律用英文：matplotlib 預設字型沒有中文字形（跟 cv2 putText 同一個坑）。
"""

from __future__ import annotations

from statistics import median

import matplotlib

matplotlib.use("Agg")  # 免 GUI，直接存檔
import matplotlib.pyplot as plt

# 中文字型。matplotlib 跟 cv2.putText 是兩回事——疊圖(overlay.py)畫不出中文只能用英文，
# 但這裡走 FreeType，Pi 上的文泉驛正黑就能畫。找不到時靜靜退回 DejaVu（中文會變空框，
# 但不會當掉）。該字型缺 U+2212 減號字形，故關掉 unicode_minus 改用 ASCII '-'。
plt.rcParams["font.sans-serif"] = ["WenQuanYi Zen Hei", "Noto Sans CJK TC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def _rolling_median(ys: list[float], window: int) -> list[float]:
    """滾動中位數平滑。

    用中位數而非平均：路面點雲退化（例如內點全擠在一條車道線上）時會冒出單幀
    ±20° 的離群值，平均會被一根尖刺整段拉歪，中位數不會。
    """
    if window <= 1 or len(ys) < 3:
        return list(ys)
    half = window // 2
    return [median(ys[max(0, i - half) : i + half + 1]) for i in range(len(ys))]


def save_angle_trend(
    path: str,
    indices: list[int],
    pitches: list[float | None],
    rolls: list[float | None] | None = None,
    title: str = "路面坡度趨勢",
    pitch_label: str = "坡度變化",
    baseline: float | None = None,
    baseline_label: str = "基準",
    smooth_window: int | None = None,
    show_raw: bool = False,
    fps: float | None = None,
) -> None:
    """畫「相對基準的坡度偏差」對 detect.mp4 幀序的折線圖，存到 path。

    pitch_label：Y 軸名稱。有 IMU 輔助時呼叫端會傳 slope（相對水平面），
    純雙目時維持 pitch（相對相機光軸）。
    baseline：要扣掉的零點（度）。None＝自動取本段中位數（純雙目沒有絕對零點時用）。
    baseline_label：0 代表什麼（「本段中位數」/「水平面」），印在下緣註記。
        Y 軸刻意只寫「坡度變化」——但**不能只寫「坡度」**：畫的是扣掉基準後的差值，
        寫「坡度」會讓人以為 +20 就是 20° 的路。「變化」兩字表達是差值，其餘細節下緣講。
    fps：錄影的名義 fps（`config.record_fps`）。給了 X 軸就換算成**影片秒數**，
        跟播放器上看到的時間一致（不用再自己把幀號除以 fps）；None 則畫幀號。
    smooth_window：滾動中位數的視窗（幀）。None＝依資料長度自動取（約 len/25、至少 5）。
    show_raw：疊上原始逐幀細線。**預設關閉**——擬合退化時原始值會衝到 ±55°，
        Y 軸被撐開後真正有意義的平滑線會被壓成中間一條細帶，反而看不出上下坡。
        要檢查雜訊程度時再開，或直接查 road_angle.csv。
    """
    fig, ax = plt.subplots(figsize=(9, 3.8), dpi=110)

    xp = [i for i, v in zip(indices, pitches) if v is not None]
    yp = [v for v in pitches if v is not None]

    if not yp:  # 整段都擬合失敗：仍要產出檔案（呼叫端不預期例外），但要看得出是空的
        ax.text(0.5, 0.5, "這一段沒有任何有效幀", transform=ax.transAxes,
                ha="center", va="center", color="#c00000", fontsize=13)
        ax.set_title(title)
        fig.tight_layout()
        fig.savefig(path)
        plt.close(fig)
        return

    base = median(yp) if baseline is None else baseline
    win = smooth_window if smooth_window is not None else max(5, (len(yp) // 25) | 1)
    dev = [v - base for v in yp]  # 扣掉基準：0＝平路、正＝上坡、負＝下坡
    sm = _rolling_median(dev, win)

    # x 軸＝本段內從 0 起算，有 fps 就直接換算成「影片秒數」——這樣圖上讀到的數字
    # 就是播放器上的時間，不用再自己把幀號除以 fps。CSV 的 index 換算寫在下緣。
    off = xp[0]
    xf = [(i - off) / fps for i in xp] if fps else [i - off for i in xp]

    ax.fill_between(xf, 0, sm, where=[v >= 0 for v in sm], interpolate=True,
                    color="#1a8a1a", alpha=0.25)
    ax.fill_between(xf, 0, sm, where=[v <= 0 for v in sm], interpolate=True,
                    color="#c05000", alpha=0.25)
    if show_raw:
        ax.plot(xf, dev, color="#1a8a1a", lw=0.6, alpha=0.25)
    ax.plot(xf, sm, color="#0d5c0d", lw=2.2)
    ax.axhline(0, color="#333333", lw=1.1)

    if rolls is not None:  # 預設不畫，見 recorder._write_trend 的說明
        xr = [i - off for i, v in zip(indices, rolls) if v is not None]
        yr = [v for v in rolls if v is not None]
        ax.plot(xr, yr, color="#f08a00", lw=1.0, alpha=0.7, label="roll (lateral)")
        ax.legend(loc="best", fontsize=8)

    # 方向標在左上/左下角，不放 0 線旁邊——曲線本來就在 0 附近遊走，會疊在一起。
    # 有這兩個字 + 綠橘塗色就夠讀，所以主線/塗色都不再進圖例（圖例整個省掉）。
    # 白底 bbox：曲線振幅大時會頂到角落的字；沒疊到時看不出有底。
    box = dict(boxstyle="square,pad=0.15", fc="white", ec="none", alpha=0.7)
    ax.text(0.012, 0.94, "上坡 +", transform=ax.transAxes, va="top", ha="left",
            fontsize=11, fontweight="bold", color="#0d5c0d", bbox=box)
    ax.text(0.012, 0.06, "下坡 -", transform=ax.transAxes, va="bottom", ha="left",
            fontsize=11, fontweight="bold", color="#a04000", bbox=box)

    ax.set_xlabel("影片秒數（跟播放器顯示的時間一致）" if fps else "影片幀號", fontsize=9)
    ax.set_ylabel(f"{pitch_label}（度）", fontsize=9)
    ax.set_title(title, fontsize=11)
    ax.grid(True, axis="y", alpha=0.2)  # 只留水平格線：看的是高低，不是 x 位置
    ax.tick_params(labelsize=8)

    # 下緣：0 代表什麼（Y 軸只寫「變化」，這裡補完）、扣了多少、平滑多寬、對到 CSV 哪幾列。
    parts = [f"0 = {baseline_label}（原始 {base:+.2f} 度）", f"{win} 幀中位數平滑"]
    parts.append(f"對應 csv index {off}-{xp[-1]}")
    fig.text(0.01, 0.015, "　·　".join(parts), fontsize=7.5, color="#666666",
             ha="left", va="bottom")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    plt.close(fig)
