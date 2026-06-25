"""
LiDARが止まらずに動くことを確認する。

設計:
lidar.iter_scans()は、forループを回す間も裏でLiDARからのデータを受信し続けている
(実質whileループ)。forループ内の処理が重いと、その間にOS側の受信バッファが
溢れてしまうことが分かっている。

また、溢れたバッファをreset_input_buffer()などで明示的にクリアする方法は、
RPLidarの5バイト固定フレームの境界を無視して切り捨てるため、フレーミングが
ズレて別の例外を引き起こすことも確認済み。OS側のバッファには触れないのが安全。

そのため、この実装では以下のように役割を分離する。
- 読み取り専用スレッド: next()を呼び続けるだけの軽い処理。
  最新の1周分(scan)を、ロックを使って参照ごと差し替える。
- メインスレッド（経路計算などの重い処理を想定）: 好きなタイミングで
  最新のscanの参照を読むだけ。OS側のバッファやシリアルポートには触れない。

参照の差し替え(代入)はPython側のメモリ操作であり、シリアルポートの
バッファを操作するわけではないので、フレーミングを壊すリスクはない。
"""

import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from rplidar import RPLidar, RPLidarException


@dataclass
class LiDARConfig:
    port: str = "/dev/ttyUSB0"


class LiDAR:
    def __init__(self, cfg: LiDARConfig):
        self.cfg = cfg
        self.lidar = None

        self._read_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        # 読み取りスレッドが書き込み、呼び出し側が読み取る「最新の1周分」
        # (quality, angle, distance)のタプルのリスト
        self._latest_scan: List[Tuple[int, float, float]] = []

    def start(self):
        self.lidar = RPLidar(self.cfg.port)

        print("LiDAR info:", self.lidar.get_info())
        print("Health:", self.lidar.get_health())

        self._stop_event.clear()
        self._read_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
        )
        self._read_thread.start()

    def _read_loop(self):
        # next()を呼び続けるだけの軽いループ。参照の差し替え以外、
        # 重い処理は一切行わない。
        try:
            for scan in self.lidar.iter_scans():
                if self._stop_event.is_set():
                    break

                with self._lock:
                    self._latest_scan = scan

        except RPLidarException as e:
            print(f"LiDAR読み取りスレッドが停止しました: {e}")

    def get_latest_scan(self) -> List[Tuple[int, float, float]]:
        # 呼び出し側はこれを好きなタイミングで呼ぶ。参照を読むだけなので軽い。
        with self._lock:
            return self._latest_scan

    def find_nearest(self, scan: List[Tuple[int, float, float]]) -> Tuple[float, float]:
        # scanの中から一番距離が近い点の(角度, 距離)を返す
        # ここでは「重い処理」の代表例として扱う
        nearest_angle = -1.0
        nearest_distance = float("inf")

        for quality, angle, distance in scan:
            if distance <= 0.0:
                continue
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_angle = angle

        return nearest_angle, nearest_distance

    def stop(self):
        self._stop_event.set()

        if self._read_thread is not None:
            # iter_scans()はジェネレータなので、シリアル読み取り待ちの
            # 最中はstop_eventをすぐには見にいけない。タイムアウトを
            # 設けて、ハングしてもメイン側の終了処理は止めない。
            self._read_thread.join(timeout=2.0)

        if self.lidar is None:
            return

        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()


if __name__ == "__main__":
    lidar = LiDAR(LiDARConfig())

    PROCESS_INTERVAL_SEC = 0.1  # 重い処理を行う間隔（経路計算などを想定）

    try:
        lidar.start()

        while True:
            scan = lidar.get_latest_scan()

            if scan:
                # ここに経路計算など重い処理を書く想定。
                # 今はその代表例としてfind_nearestを使う。
                angle, distance = lidar.find_nearest(scan)
                print(f"angle={angle:.1f}deg, distance={distance:.1f}mm")

            time.sleep(PROCESS_INTERVAL_SEC)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        lidar.stop()
