import sys
import time
import logging
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Union
import numpy as np
import cv2
from picamera2 import Picamera2

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger(__name__)


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
    TARGET_FPS: int = 30
    FRAME_INTERVAL: float = 1.0 / TARGET_FPS


@dataclass(frozen=True)
class BallDetectionConfig:
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


class BallDetection:
    def __init__(self, config: BallDetectionConfig) -> None:
        self.cfg = config

    def find_circle_hough(self, red: np.ndarray) -> Union[np.ndarray, List]:
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

    def calc_circle_hough(
        self, circles: Union[np.ndarray, List]
    ) -> List[Tuple[float, float, float, float]]:

        features = []
        if len(circles) == 0:
            return features

        # cv2.HoughCirclesの戻り値は shape (1, N, 3) なので [0] で余分な次元を外す
        for circle in circles[0]:
            x, y, r = circle
            S = r * r * np.pi
            features.append((float(x), float(y), float(r), float(S)))

        return features

    def draw_detection_overlay(
        self,
        original_img: np.ndarray,
        red_mask: np.ndarray,
        features: List[Tuple[float, float, float, float]],
    ) -> np.ndarray:
        overlay_img = original_img.copy()
        overlay_img[red_mask > 0] = [0, 0, 255]

        for x, y, r, S in features:
            center = (int(x), int(y))
            cv2.circle(overlay_img, center, int(r), (0, 255, 0), thickness=2)
            cv2.drawMarker(
                overlay_img,
                center,
                (255, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=16,
                thickness=2,
            )

        return overlay_img


if __name__ == "__main__":
    cam_cfg = CameraConfig()
    cam = CameraPublic(config=cam_cfg)
    ball_cfg = BallDetectionConfig()
    ball = BallDetection(config=ball_cfg)

    picam2 = cam.start_camera()
    logger.debug("Camera started")
    time.sleep(1)  # warm up

    prev_time = time.time()

    try:
        while True:
            frame = picam2.capture_array()

            # --- ここに処理を書く（例：何もしない） ---
            red = cam.extract_red_hsv(img=frame)
            clean = cam.remove_noise(red)

            circles = ball.find_circle_hough(red=red)
            features = ball.calc_circle_hough(circles)
            # ------------------------------------

            for x, y, r, S in features:
                logger.debug(f"Detected center: (x={x}, y={y}), r={r}, area={S}")

            overlay = ball.draw_detection_overlay(
                original_img=frame, red_mask=clean, features=features
            )
            cv2.imshow("Hough Circle Detection", overlay)

            # 実FPS計測（確認用）
            now = time.perf_counter()
            actual_fps = 1.0 / (now - prev_time)
            prev_time = now
            logger.debug(f"Actual FPS: {actual_fps:.2f}")

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
