from dataclasses import dataclass
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar


@dataclass
class LiDARConfig:
    port: str = "/dev/ttyUSB0"


class LiDAR:
    def __init__(self, cfg: LiDARConfig):
        self.cfg = cfg
        self.lidar = None
        self.scan_generator = None

    def start(self):
        self.lidar = RPLidar(self.cfg.port)

        print("LiDAR info:", self.lidar.get_info())
        print("Health:", self.lidar.get_health())

        self.scan_generator = self.lidar.iter_scans()

    def get_scan_once(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.lidar is None:
            raise RuntimeError("LiDAR が起動されていません")
        if self.scan_generator is None:
            raise RuntimeError("lidar.iter_scans が機能していません")

        scan = next(self.scan_generator)

        angles = []
        distances = []

        for quality, angle, distance in scan:
            angles.append(angle)
            distances.append(distance)

        return (
            np.array(angles, dtype=np.float32),
            np.array(distances, dtype=np.float32),
        )

    def stop(self):
        if self.lidar is None:
            return

        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def visualizer_start(self, max_distance=8000):
        plt.ion()
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection="polar")
        self.ax.set_ylim(0, max_distance)
        self.scatter = self.ax.scatter([], [], s=5)
        self.ax.set_title("RPLiDAR Real-time Scan")

    def visualizer_update(self, angles, distances):
        points = np.column_stack((angles, distances))
        self.scatter.set_offsets(points)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == "__main__":
    lidar = LiDAR(LiDARConfig())

    try:
        lidar.start()

        while True:
            angles, distances = lidar.get_scan_once()
            lidar.visualizer_update(angles, distances)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        lidar.stop()
