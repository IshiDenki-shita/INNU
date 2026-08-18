"""
色認識キャリブレーション用: ラズパイカメラで撮影するだけのコード

カメラ起動から一定時間（デフォルト5秒）待ってから撮影し、保存する。
撮影した画像ファイルをMacに転送して color_extract.py で使う。
"""

import time
from dataclasses import dataclass
from pathlib import Path
import cv2
import numpy as np

from Experiment.mods.camera_public import CameraPublic

if __name__ == "__main__":
    WARM_UP_SEC: float = 5.0
    save_path = Path("home/ubuntu/Desktop/INNU/Experiment/color_picking/photos")

    cam = CameraPublic(STORAGE_PATH=save_path)

    print(f"カメラ起動。{WARM_UP_SEC}秒後に撮影します...")
    time.sleep(WARM_UP_SEC)

    img = cam.get_frame()

    cam.save_frame(img=img)
    print(f"保存しました: {save_path}")
