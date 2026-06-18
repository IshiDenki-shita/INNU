"""
ロボットが目的の方向に移動するようにモーターを回すコード
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class MotorConfig:
    max_duty: int = 255  # デューティー比の最大値


class MotorSpinner:
    # 基本
    num_motor = 4  # モーターの数
    # 進行方向
    target_speed = 0
    target_direction = 0
    # モーターごとのデューティー比
    motor_duties = [0] * num_motor

    def __init__(self):
        # モーターとの接続など
        ...

    def DirUpdate(self, direction: np.float16, speed: np.float16):
        # モーターのPWM出力を計算し、更新する関数
        ...

    def PWMshootor(self):
        # PWMを出力し続ける。別スレッドで動き続ける。
        ...

    def stop(self):
        # モーターとの接続解除など
        ...
