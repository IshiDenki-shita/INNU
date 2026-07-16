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
from picamera2 import Picamera2


@dataclass(frozen=True)
class CaptureConfig:
    # input/output path
    PHOTO_PATH: Path = Path("Experiment/color_picking/photos")
    PHOTO_NAME: str = "capture.jpg"
    # img property
    WIDTH: int = 640
    HEIGHT: int = 480
    # カメラ起動から撮影までの待機秒数（露出・ホワイトバランス安定待ち）
    WARM_UP_SEC: float = 5.0


class CameraCapture:
    def __init__(self, config: CaptureConfig):
        self.cfg = config
        self.cfg.PHOTO_PATH.mkdir(parents=True, exist_ok=True)

    def capture_photo(self) -> np.ndarray:
        picam2 = Picamera2()

        config = picam2.create_still_configuration(
            main={"size": (self.cfg.WIDTH, self.cfg.HEIGHT)}
        )
        picam2.configure(config)
        picam2.start()

        print(f"カメラ起動。{self.cfg.WARM_UP_SEC}秒後に撮影します...")
        time.sleep(self.cfg.WARM_UP_SEC)

        image_rgb = picam2.capture_array()
        picam2.stop()

        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        save_path = self.cfg.PHOTO_PATH / self.cfg.PHOTO_NAME
        cv2.imwrite(str(save_path), image_bgr)
        print(f"保存しました: {save_path}")

        return image_bgr


if __name__ == "__main__":
    cfg = CaptureConfig()
    camera = CameraCapture(cfg)
    camera.capture_photo()
