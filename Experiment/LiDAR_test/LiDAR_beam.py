"""
LiDARの周りに物体を近づけてみる。その時のLiDARが一番壁に近いと判断した方角の変化を確かめる

注記:
rplidarライブラリのiter_scans()は、内部のiter_measures()がバッファ溢れを検知した際に
stop()→start()で自動リカバリしようとするが、このリカバリ処理自体が
「停止コマンドを送ってから0.1秒待つだけ」でセンサーの送信停止を保証しておらず、
get_health()の応答待ち中にスキャンデータの取りこぼしが発生し
'Incorrect descriptor starting bytes' を引き起こすことが分かった。
そのため、このコードではiter_scans()を使わず、iter_measures()を自前で制御し、
バッファ溢れの監視・対処もこちら側で安全に行う。
"""

from dataclasses import dataclass
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar


@dataclass
class LiDARConfig:
    port: str = "/dev/ttyUSB0"
    max_buf_meas: int = 6000  # この値を超えたら明示的にバッファをクリアする


class LiDAR:
    def __init__(self, cfg: LiDARConfig):
        self.cfg = cfg
        self.lidar = None
        self.measure_generator = None
        self._pending_point = None  # 周の境界で次回に持ち越す1点

    def start(self):
        self.lidar = RPLidar(self.cfg.port)

        print("LiDAR info:", self.lidar.get_info())
        print("Health:", self.lidar.get_health())

        # iter_scans()は使わず、低レベルのiter_measures()を自前で制御する
        # max_buf_meas=Falseでライブラリ内部の自動stop/startリカバリを無効化し、
        # 代わりにこちら側でバッファ監視を行う
        self.measure_generator = self.lidar.iter_measures(max_buf_meas=False)

    def _clean_buffer_if_needed(self):
        # スキャンを止めずに、シリアルの受信バッファだけを直接クリアする
        # （lidar.stop()のような状態変更を挟まないので、安全に呼べる）
        waiting = self.lidar._serial.in_waiting
        if waiting > self.cfg.max_buf_meas:
            print(f"バッファに{waiting}バイト溜まっています。クリアします。")
            self.lidar._serial.reset_input_buffer()

    def get_scan_once(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.lidar is None:
            raise RuntimeError("LiDAR が起動されていません")
        if self.measure_generator is None:
            raise RuntimeError("lidar.iter_measures が機能していません")

        angles = []
        distances = []

        # 前回の境界で持ち越した点があれば、今回のスキャンの先頭として使う
        if self._pending_point is not None:
            angle, distance = self._pending_point
            angles.append(angle)
            distances.append(distance)
            self._pending_point = None

        # new_scanが立つまで1点ずつ集め、1周分をまとめて返す
        for new_scan, quality, angle, distance in self.measure_generator:
            self._clean_buffer_if_needed()

            if new_scan and angles:
                # この点は次の周の先頭なので、今回は返さず持ち越す
                self._pending_point = (angle, distance)
                break

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
        lidar.visualizer_start()

        while True:
            angles, distances = lidar.get_scan_once()
            lidar.visualizer_update(angles, distances)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        lidar.stop()
