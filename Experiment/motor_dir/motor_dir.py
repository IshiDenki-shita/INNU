"""
ロボットが目的の方向に移動するようにモーターを回すコード
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class MotorConfig:
    max_duty: int = 255  # デューティー比の最大値


class MotorSpinner:
    def __init__(self):
        # モーターとの接続など
        ...

    def DirUpdate(self, direction: np.float16):
        # モーターへの
        ...

    def PWMshootor(self):
        # PWMを出力し続ける。別スレッドで動き続ける。
        ...

    def stop(self):
        # モーターとの接続解除など
        ...
