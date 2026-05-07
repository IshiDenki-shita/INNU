import math
import time
from pathlib import Path
import datetime
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import cv2
from picamera2 import Picamera2


@dataclass(frozen=True)
class CameraConfig:
    # input/output path
    PHOTO_PATH: Path = Path("Experiment/camera/photos")
    # img property
    QUALITY: int = 95
    WIDTH: int = 640
    HEIGHT: int = 480
    # noise remove
    MORPH_KERNEL_SHAPE: Tuple = (5, 5)
    MORPH_ITERATION: int = 1
    # FPS
    TARGET_FPS: int = 10
    FRAME_INTERVAL: float = 1.0 / TARGET_FPS


class Utility:
    def __init__(self) -> None: ...


class CameraPublic:
    def __init__(self, config: CameraConfig):
        self.cfg = config
        self.cfg.PHOTO_PATH.mkdir(parents=True, exist_ok=True)

    def start_camera(self):
        picam2 = Picamera2()

        config = picam2.create_preview_configuration(
            main={"size": (self.cfg.WIDTH, self.cfg.HEIGHT), "format": "BGR888"},
            controls={
                "FrameDurationLimits": (
                    int(1000000 / self.cfg.TARGET_FPS),
                    int(1000000 / self.cfg.TARGET_FPS),
                )
            },
        )
        picam2.configure(config)
        picam2.start()

        return picam2

    def save_photo(self, img: np.ndarray, photo_name: str):
        save_path = self.cfg.PHOTO_PATH / photo_name

        is_success = cv2.imwrite(
            str(save_path), img, [cv2.IMWRITE_JPEG_QUALITY, self.cfg.QUALITY]
        )
        if not is_success:
            raise RuntimeError(f"画像を保存できませんでした。file_path {save_path}")

    def remove_noise(self, img: np.ndarray) -> np.ndarray:
        kernel = np.ones(self.cfg.MORPH_KERNEL_SHAPE, dtype=np.uint8)

        clean = cv2.morphologyEx(
            src=img,
            op=cv2.MORPH_OPEN,
            kernel=kernel,
            iterations=self.cfg.MORPH_ITERATION,
        )

        return clean

    def extract_red_hsv(self, img: np.ndarray) -> np.ndarray:
        red = img.copy()
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 赤色範囲
        # 赤はHSV空間で0/180をまたぐので2つ必要
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # マスク作成
        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        red_bin = cv2.bitwise_or(src1=mask1, src2=mask2)
        # red[np.where(red_bin == 0)] = 0

        return red_bin

    def extract_red_bgr(self, img: np.ndarray) -> np.ndarray: ...


if __name__ == "__main__":
    cam_cfg = CameraConfig()
    cam = CameraPublic(config=cam_cfg)

    picam2 = cam.start_camera()
    time.sleep(1)  # warm up

    prev_time = time.time()
    fps_history = []
    count = 0

    try:
        while count < 500:
            start_time = time.perf_counter()
            frame = picam2.capture_array()

        print(f"Target FPS: {cam_cfg.TARGET_FPS}\nActual FPS: {fps_history}")
        print(f"FPS mean: {sum(fps_history) / len(fps_history)}")

    finally:
        picam2.stop()
