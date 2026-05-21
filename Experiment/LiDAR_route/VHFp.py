"""
進行方向を0°に固定し、障害物をLiDARに近づけた時の目標経路の変化を見る。
"""

from typing import List, Tuple
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt


@dataclass
class VHFPlus_config:
    sector_deg: int = 5
    threshold: float = 1.0
    robot_radius: float = 0.2
    safety_distance: float = 0.1


class VFHPlus:
    def __init__(self, cfg: VHFPlus_config):
        self.cfg = cfg

    def build_histogram(self, angles, distances) -> np.ndarray:
        """
        angles: degree
        distances: meter
        """
        hist = np.zeros(self.cfg.sector_deg)

        for angle, dist in zip(angles, distances):

            if dist <= 0.01:
                continue

            sector = int(angle // self.cfg.sector_deg)

            # 近い障害物ほど危険度を上げる
            magnitude = 1.0 / dist
            hist[sector] += magnitude

        return hist

    def smooth_histogram(self, hist: np.ndarray) -> np.ndarray:
        kernel = np.array([1, 2, 3, 2, 1])
        kernel = kernel / kernel.sum()

        return np.convolve(hist, kernel, mode="same")

    def find_candidate_directions(self, hist: np.ndarray) -> np.ndarray:
        candidates = []

        for i, value in enumerate(hist):
            if value < self.cfg.threshold:
                angle = i * self.cfg.sector_deg
                candidates.append(angle)

        return np.ndarray(candidates)

    def choose_direction(self, candidates: np.ndarray, target_angle=0) -> float:

        if len(candidates) == 0:
            # self.lidar.stop()
            raise RuntimeError("経路の候補が見つかりませんでした。")

        candidates = np.array(candidates)
        diff = np.abs(candidates - target_angle)
        idx = np.argmin(diff)

        return candidates[idx]

    def compute(self, angles, distances, target_angle=0):

        hist = self.build_histogram(angles, distances)
        hist = self.smooth_histogram(hist)

        direction = self.choose_direction(
            self.find_candidate_directions(hist),
            target_angle,
        )

        return direction, hist


def demo():

    lidar_angles = np.arange(360)
    lidar_distances = np.ones(360) * 3.0

    lidar_distances[350:360] = 0.5
    lidar_distances[0:10] = 0.5
    vfh = VFHPlus(cfg=VHFPlus_config())

    direction, hist = vfh.compute(
        lidar_angles,
        lidar_distances,
        target_angle=0,
    )

    print(f"selected direction: {direction}")
    plt.figure(figsize=(10, 4))
    plt.plot(hist)
    plt.title("Polar Histogram")
    plt.xlabel("Sector")
    plt.ylabel("Obstacle Density")
    plt.show()


if __name__ == "__main__":
    demo()
