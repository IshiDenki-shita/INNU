import time
from datetime import datetime
from pathlib import Path
from typing import Tuple
import numpy as np
import cv2
from picamera2 import Picamera2


class CameraPublic:

    def __init__(
        self,
        # input/output path
        SAMPLE_PATH: Path = Path("Experiment/ball_detection/sample/RedBall.jpeg"),
        STORAGE_PATH: Path = Path("Experiment/photos/caps/"),
        # img property
        QUALITY: int = 95,
        WIDTH: int = 640,
        HEIGHT: int = 480,
        # noise remove
        MORPH_KERNEL_SHAPE: Tuple = (5, 5),
        MORPH_ITERATION: int = 1,
        # FPS
        TARGET_FPS: int = 30,
        # red color range (BGR)
        LOWER_BGR: np.ndarray = np.array([0, 0, 100]),
        UPPER_BGR: np.ndarray = np.array([100, 100, 255]),
    ):
        self.SAMPLE_PATH = SAMPLE_PATH
        self.STORAGE_PATH = STORAGE_PATH
        self.QUALITY = QUALITY
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.MORPH_KERNEL_SHAPE = MORPH_KERNEL_SHAPE
        self.MORPH_ITERATION = MORPH_ITERATION
        self.TARGET_FPS = TARGET_FPS
        self.LOWER_BGR = LOWER_BGR
        self.UPPER_BGR = UPPER_BGR
        # 引数でないクラス変数
        self.FRAME_INTERVAL = 1.0 / TARGET_FPS
        self.cam = None

    def start_camera(self):
        self.cam = Picamera2()

        config = self.cam.create_preview_configuration(
            main={"size": (self.WIDTH, self.HEIGHT), "format": "BGR888"},
            controls={
                "FrameDurationLimits": (
                    int(1000000 / self.TARGET_FPS),
                    int(1000000 / self.TARGET_FPS),
                )
            },
        )
        self.cam.configure(config)
        self.cam.start()
        time.sleep(2)

    def get_frame(self) -> np.ndarray:
        return self.cam.capture_array()

    def save_frame(self, img) -> None:
        self.STORAGE_PATH.mkdir(exist_ok=True)
        pic_name = str(datetime.now())
        cv2.imwrite(filename=self.STORAGE_PATH / pic_name, img=img)

    def get_sample_photo(self):
        img = cv2.imread(str(self.SAMPLE_PATH))

        if img is None:
            raise ValueError("サンプル画像を取得できませんでした。")

        return img

    def remove_noise(self, img: np.ndarray) -> np.ndarray:
        kernel = np.ones(self.MORPH_KERNEL_SHAPE, dtype=np.uint8)

        clean = cv2.morphologyEx(
            src=img,
            op=cv2.MORPH_OPEN,
            kernel=kernel,
            iterations=self.MORPH_ITERATION,
        )

        return clean

    def extract_red_bgr(self, img: np.ndarray) -> np.ndarray:
        red_bin = cv2.inRange(img, self.LOWER_BGR, self.UPPER_BGR)
        return red_bin
