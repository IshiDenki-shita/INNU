"""
コードの完成予想を書いてみるパレット(カメラモジュールに関して)
"""

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


@dataclass(frozen=True)
class BallDetectionConfig:
    # find contour
    MIN_CIRCLE_SIZE: int = 100
    THRESH_CIRCULARITY: float = 0.8
    # hough transformation
    ACCUMULATOR_RATIO: float = 2
    MIN_CIRCLE_DIST: float = 50
    CANNY_THRESH: float = 100
    CIRCLE_VOTE_THRESH: float = 20
    MIN_RADIUS: int = 10
    MAX_RADIUS: int = 200


class Utility:
    def __init__(self) -> None: ...


class CameraPublic:
    def __init__(self, config: CameraConfig):
        self.cfg = config
        self.cfg.PHOTO_PATH.mkdir(parents=True, exist_ok=True)

        # calibration
        data = np.load("camera_calibration.npz")
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]

        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            dist_coeffs,
            (self.cfg.WIDTH, self.cfg.HEIGHT),
            1,
            (self.cfg.WIDTH, self.cfg.HEIGHT),
        )

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            camera_matrix,
            dist_coeffs,
            None,
            new_camera_matrix,
            (self.cfg.WIDTH, self.cfg.HEIGHT),
            cv2.CV_16SC2,
        )

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


class BallDetection:
    def __init__(self, config: BallDetectionConfig) -> None:
        self.cfg = config

    def find_red_circle(self, red: np.ndarray) -> Tuple[int, int, float]:

        contours, hierarchy = cv2.findContours(
            image=red, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE
        )

        best_contour = None
        best_circularity = 0
        best_area = 0
        for contour in contours:
            area = cv2.contourArea(contour=contour, oriented=False)

            if area < self.cfg.MIN_CIRCLE_SIZE:
                continue

            perimeter = cv2.arcLength(curve=contour, closed=True)
            circularity = 4 * math.pi * area / (perimeter * perimeter)

            if circularity < self.cfg.THRESH_CIRCULARITY:
                continue
            if circularity > best_circularity:
                best_circularity = circularity
                best_contour = contour
                best_area = area

        if best_contour is None:
            return (-1, -1, 0)

        M = cv2.moments(best_contour)

        if M["m00"] == 0:
            return (-1, -1, 0)

        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])

        return (center_x, center_y, best_area)

    def find_circle_hough(self, red: np.ndarray) -> np.ndarray | List:
        red = red.astype(np.uint8)

        circles = cv2.HoughCircles(
            image=red,
            method=cv2.HOUGH_GRADIENT,
            dp=self.cfg.ACCUMULATOR_RATIO,
            minDist=self.cfg.MIN_CIRCLE_DIST,
            param1=self.cfg.CANNY_THRESH,
            param2=self.cfg.CIRCLE_VOTE_THRESH,
            minRadius=self.cfg.MIN_RADIUS,
            maxRadius=self.cfg.MAX_RADIUS,
        )

        if circles is None:
            return []

        return circles

    # not required probably
    def calc_circle_hough(
        self, circles: List[Tuple[float, float, float, float]]
    ) -> List[float]:

        features = []
        for circle in circles:
            x = circle[0]
            y = circle[1]
            r = circle[2]
            S = r * r * math.pi
            features.append((x, y, r, S))
        return features


if __name__ == "__main__":
    cam_cfg = CameraConfig()
    cam = CameraPublic(config=cam_cfg)
    ball_cfg = BallDetectionConfig()
    ball = BallDetection(config=ball_cfg)

    picam2 = cam.start_camera()
    time.sleep(1)  # warm up

    prev_time = time.time()
    fps_history = []
    count = 0

    try:
        while count < 500:
            start_time = time.perf_counter()
            frame = picam2.capture_array()

            # --- ここに処理を書く（例：何もしない） ---
            caliblated = cv2.remap(
                frame, cam.map1, cam.map2, interpolation=cv2.INTER_LINEAR
            )
            red = cam.extract_red_hsv(img=caliblated)
            clean = cam.remove_noise(red)

            circles = ball.find_circle_hough(red=red)
            features = ball.calc_circle_hough(circles)

            x, y, S = ball.find_red_circle(red=clean)
            # ------------------------------------

            count += 1

            # 実FPS計測（確認用）
            now = time.perf_counter()
            actual_fps = 1.0 / (now - prev_time)
            prev_time = now
            fps_history.append(actual_fps)

        print(f"Target FPS: {cam_cfg.TARGET_FPS}\nActual FPS: {fps_history}")
        print(f"FPS mean: {sum(fps_history) / len(fps_history)}")

    finally:
        picam2.stop()
