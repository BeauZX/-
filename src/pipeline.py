"""離線管線：一段錄影 → 每幀前方路面縱向坡度。

把各模組串起來：
    pairing.iter_pairs  (左右幀對)
      → disparity.stereo_to_points  (校正→視差→3D 點雲, 公尺)
      → roadplane.select_road_points + fit_road_plane  (路面平面→坡度角)

輸出每幀一筆 FrameResult。IMU 只在最後 to_gravity_referenced 當輔助修正用，
不影響純雙目主結果。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np

from .calib_loader import StereoCalibration
from .config import DEFAULT, Config
from .disparity import StereoMatcher, StereoResult, compute_stereo
from .pairing import iter_pairs
from .roadplane import RoadPlane, fit_road_plane, select_road_points


@dataclass
class FrameResult:
    index: int
    pitch_deg: float | None  # 純雙目、相機相對的縱向坡度；擬合失敗為 None
    roll_deg: float | None
    cam_height_m: float | None
    n_inliers: int
    n_road_points: int
    rms_m: float | None
    time_diff_ms: float | None  # 左右幀時間差（配對品質指標）
    # IMU 輔助（只有 use_imu 開啟時才有值，否則維持 None＝純雙目行為不變）：
    imu_pitch_deg: float | None = None  # 相機自身相對水平的 pitch（IMU 讀）
    pitch_gravity_deg: float | None = None  # 路面相對「水平面」的真實坡度＝相機相對 + IMU

    @classmethod
    def failed(cls, index: int, n_pts: int, time_diff_ms: float | None) -> "FrameResult":
        return cls(index, None, None, None, 0, n_pts, None, time_diff_ms)

    def with_imu(self, imu_pitch_deg: float | None) -> "FrameResult":
        """套上 IMU 相機 pitch，補算相對水平面的真實坡度（就地修改並回傳自己）。

        imu_pitch_deg 為 None（IMU 不可用/關閉）或本幀擬合失敗時，兩個欄位維持 None。
        """
        self.imu_pitch_deg = imu_pitch_deg
        if imu_pitch_deg is not None and self.pitch_deg is not None:
            self.pitch_gravity_deg = self.pitch_deg + imu_pitch_deg
        return self

    @classmethod
    def from_plane(
        cls, index: int, plane: RoadPlane, time_diff_ms: float | None
    ) -> "FrameResult":
        return cls(
            index=index,
            pitch_deg=plane.pitch_deg,
            roll_deg=plane.roll_deg,
            cam_height_m=plane.cam_height_m,
            n_inliers=plane.n_inliers,
            n_road_points=plane.n_points,
            rms_m=plane.rms_m,
            time_diff_ms=time_diff_ms,
        )


def estimate_from_result(
    config: Config,
    rng: np.random.Generator,
    index: int,
    res: StereoResult,
    time_diff_ms: float | None = None,
) -> FrameResult:
    """已算好的 StereoResult → 路面坡度。UI worker 用這個（算一次可兼顧畫圖）。"""
    # 影像橫帶已由 disparity 的 ROI 裁切處理，這裡不再重複裁列 (=1.0)，
    # 只靠距離/高度過濾 + RANSAC 把非路面點濾掉。
    road = select_road_points(
        res.pts,
        res.valid,
        z_min=config.z_min_m,
        z_max=config.z_max_m,
        y_min=config.y_min_m,
        image_bottom_fraction=1.0,
    )
    plane = fit_road_plane(
        road,
        threshold_m=config.ransac_threshold_m,
        iterations=config.ransac_iterations,
        min_inliers=config.min_road_points,
        rng=rng,
    )
    if plane is None:
        return FrameResult.failed(index, len(road), time_diff_ms)
    return FrameResult.from_plane(index, plane, time_diff_ms)


def estimate_pair(
    calib: StereoCalibration,
    matcher: StereoMatcher,
    config: Config,
    rng: np.random.Generator,
    index: int,
    img0: np.ndarray,
    img1: np.ndarray,
    time_diff_ms: float | None = None,
    roi: tuple[int, int, int, int] | None = None,
) -> FrameResult:
    """單一左右幀對 → 路面坡度。離線(process_segment)與即時(process_live)共用。

    roi=(x0,y0,x1,y1)（process 座標）時只在框內算視差；None 退回預設下方橫帶。
    """
    res = compute_stereo(calib, matcher, img0, img1, roi)
    return estimate_from_result(config, rng, index, res, time_diff_ms)


def process_segment(
    calib: StereoCalibration,
    seg_dir: str | Path,
    *,
    config: Config = DEFAULT,
    cam0_mp4: str | Path | None = None,
    cam1_mp4: str | Path | None = None,
) -> Iterator[FrameResult]:
    """走訪一段錄影，逐幀產生路面坡度結果。參數全部來自 config（固定）。"""
    matcher = StereoMatcher.from_config(config)
    rng = np.random.default_rng(config.seed)

    for pair in iter_pairs(seg_dir, cam0_mp4=cam0_mp4, cam1_mp4=cam1_mp4):
        yield estimate_pair(
            calib, matcher, config, rng, pair.index, pair.img0, pair.img1, pair.time_diff_ms
        )


def process_live(
    calib: StereoCalibration,
    *,
    config: Config = DEFAULT,
    max_frames: int | None = None,
    recorder=None,
    roi: tuple[int, int, int, int] | None = None,
    imu=None,
) -> Iterator[FrameResult]:
    """接兩顆即時鏡頭，逐幀產生路面坡度結果，直到 Ctrl+C 或達到 max_frames。

    recorder（SessionRecorder，可選）非 None 時，每幀把原生 cam0/cam1 錄下來。
    roi（process 座標，可選）非 None 時只在框內算視差——headless 用它套用 UI 記住
    的 roi.json，跟 UI 模式吃同一個框。
    imu（ImuReader，可選）非 None 時，每幀取最新相機 pitch 補算相對水平面的真實坡度
    （pitch_gravity_deg）；None＝不做 IMU 修正，只有純雙目相機相對坡度。
    """
    from .live import LiveStereo  # 延後 import：只有即時模式才需要 picamera2

    matcher = StereoMatcher.from_config(config)
    rng = np.random.default_rng(config.seed)

    with LiveStereo(config, size=calib.image_size) as cams:
        i = 0
        for img0, img1 in cams.frames():
            fr = estimate_pair(calib, matcher, config, rng, i, img0, img1, None, roi)
            if imu is not None:
                fr.with_imu(imu.pitch_deg)
            if recorder is not None:
                recorder.add(img0, img1, fr)
            yield fr
            i += 1
            if max_frames is not None and i >= max_frames:
                break
