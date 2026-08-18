"""
Raspberry pi 4Bで、一秒間に何枚の写真を撮れるか実験
"""

import math
import time
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import cv2

from Experiment.mods.camera_public import CameraPublic


@dataclass
class BallDetectionConfig:
    # extracting red
    ACCUMULATER_RATIO: float = 2
    MIN_CIRCLE_DIST: float = 50
    CANNY_THRESH: float = 100
    CIRCLE_VOTE_THRESH: float = 20
    MIN_RADIUS: int = 10
    MAX_RADIUS: int = 200


class Utility:
    def __init__(self) -> None: ...


class BallDetection:
    def __init__(self, config: BallDetectionConfig) -> None:
        self.cfg = config

    def find_circle_hough(
        self, img: np.ndarray
    ) -> List[Tuple[float, float, float]] | bool:
        circles = cv2.HoughCircles(
            image=img,
            method=cv2.HOUGH_GRADIENT,
            dp=self.cfg.ACCUMULATER_RATIO,
            minDist=self.cfg.MIN_CIRCLE_DIST,
            param1=self.cfg.CANNY_THRESH,
            param2=self.cfg.CIRCLE_VOTE_THRESH,
            minRadius=self.cfg.MIN_RADIUS,
            maxRadius=self.cfg.MAX_RADIUS,
        )

        if circles is None:
            return False

        return circles

    # not required probably
    def calc_s_hough(self, circles: List[Tuple[float, float, float]]) -> List[float]:
        Ss = []
        for circle in circles:
            r = circle[2]
            Ss.append(r * r * math.pi)
        return Ss


if __name__ == "__main__":
    QUALITY: int = 95
    WIDTH: int = 640
    HEIGHT: int = 480
    MORPH_KERNEL_SHAPE = (5, 5)
    MORPH_ITERATION: int = 1
    TARGET_FPS: int = 30
    FRAME_INTERVAL = 1.0 / TARGET_FPS

    cam = CameraPublic()
    ball_cfg = BallDetectionConfig()
    ball = BallDetection(config=ball_cfg)

    cap = cv2.VideoCapture(0)  # Camera Module v2
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # ウォームアップ
    time.sleep(1)

    prev_time = time.time()
    fps_history = []
    count = 0

    while count < 500:
        start_time = time.time()

        ret, frame = cap.read()
        if not ret:
            print("画像を取得できませんでした")
            break

        # --- ここに処理を書く（例：何もしない） ---
        red = cam.extract_red_hsv(img=frame)
        ball.find_circle_hough(img=red)
        # ------------------------------------

        elapsed = time.time() - start_time
        sleep_time = FRAME_INTERVAL - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)

        # 実FPS計測（確認用）
        now = time.time()
        actual_fps = 1.0 / (now - prev_time)
        prev_time = now
        fps_history.append(actual_fps)

    print(f"Target FPS: {TARGET_FPS}\nActual FPS: {fps_history:.2f}")
    print(f"mean: {sum(fps_history) / len(fps_history)}")
