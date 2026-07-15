"""
C++を書くのは面倒なので、pythonで書いてclaudeに書き直してもらう。
"""

import threading
import numpy as np

MAX_DATA_SIZE = 8000  # ROSのlidar点群用のバッファの長さ
purpose_idx: int  # 画像ラズパイから来たボールへの方角

lock = threading.Lock()  # C++でどう制御するかは未定


def dirctionCallback() -> None:
    """
    画像ラズパイから目的の方角を受信した時の割り込み
    詳しい実装は未定
    """
    with lock:
        purpose_theta = np.pi / 3
        # θをLiDARからの受信バッファのインデックスに変換
        purpose_idx = int(float(MAX_DATA_SIZE) * (purpose_theta / np.pi) / 2)


class LidarRouting:

    real_data = [0] * MAX_DATA_SIZE  # LiDARの割り込みが最初に書き換える配列
    temp_data = [0] * MAX_DATA_SIZE  # ここに取り出してから経路を計算

    RADIUS_OF_MYSELF = 1  # ロボット自身の半径
    THRESH_DANGER_DIST = 2.0  # 危険な壁の近さ

    alpha = 2  # 方角ごとのヒューリスティックの計算で使う係数。具体的な値は実験で求める
    beta = 3

    def scanCallback(self, scan) -> None:
        """
        LiDARからの受信割り込み
        """
        with lock:
            self.real_data = scan

    def calc_invalid_width(self, dist) -> int:
        """
        壁が近い点からどの範囲に衝突危険性があるか計算
        """
        invalid_theta = np.arcsin(self.RADIUS_OF_MYSELF / dist)
        invalid_width = MAX_DATA_SIZE * invalid_theta / np.pi / 2
        return int(invalid_width)

    def calc_direction_score(self, dist, idx) -> float:
        """
        方角ごとの経路としての妥当さを計算。
        壁から遠い(distが大きい) なら高スコア
        """
        with lock:
            temp_idx = purpose_idx

        delta_idx = min(temp_idx - idx, MAX_DATA_SIZE - temp_idx + idx)
        return np.pow(np.e, dist * self.alpha) / (delta_idx * self.beta)

    def calc_route(self) -> float:
        """
        進行方向として最も妥当な方角を計算
        """
        with lock:
            temp_data = self.real_data  # データを取り出してから計算開始

        valid_data = [1] * 8000

        # 危険な（壁が近い）方向とその周辺の角度を排除
        for i in range(MAX_DATA_SIZE):

            if temp_data[i] < self.THRESH_DANGER_DIST:

                invalid_width = self.calc_invalid_width(temp_data)
                start = max(i - invalid_width, 0)
                finish = min(i + invalid_width, MAX_DATA_SIZE)

                for j in range(start, finish, 1):
                    valid_data[j] = 0

        # 壁が遠くて目的に近い方向を選択
        best_score = 0
        best_dir = 0

        for i in range(MAX_DATA_SIZE):
            if not valid_data[i]:
                # 排除されてたらスキップ
                pass

            score = self.calc_direction_score(dist=temp_data[i], idx=i)

            if score > best_score:
                best_dir = i

        # θに変換して完了
        return np.pi * best_dir / float(MAX_DATA_SIZE)
