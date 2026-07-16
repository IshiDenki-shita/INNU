"""
目標の進行方向が変化したときにduty比を滑らかに変化させるロジック
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class MotorConfig:
    max_duty = 255  # デューティー比の最大値
    pwm_freq = 3000  # PWM周波数

    # モーターのピン配置
    MA_1 = 20
    MA_2 = 21
    MB_1 = 0
    MB_2 = 0
    MC_1 = 0
    MC_2 = 0
    MD_1 = 0
    MD_2 = 0


class MotorSpinner:

    motor_duties = [0] * 8

    def __init__(self, cfg: MotorConfig) -> None:
        self.cfg = cfg

    def start(self):
        # set_PWM_frequencyを実行
        ...

    def DutyUpdate(self, dist: float, theta: float):
        """
        ボールに到達するためのduty比を計算
        """
        ...

    def PWMshooter(self):
        # set_PWM_dutycycleを実行
        ...

    def stop(self):
        # モーターとの接続解除など
        ...


def demo():
    spinner = MotorSpinner(MotorConfig())

    spinner.start()

    while True:
        try:
            purpose_dist = 3.0  # カメラからの受信
            purpose_theta = np.pi / 3.0  # 別のクラスで計算

            spinner.DutyUpdate(purpose_dist, purpose_theta)
        except KeyboardInterrupt:
            break

    spinner.stop()
