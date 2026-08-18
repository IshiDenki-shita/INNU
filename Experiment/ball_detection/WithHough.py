import sys
import time
import logging
from dataclasses import dataclass
from typing import List, Tuple, Union
import numpy as np
import cv2

from Experiment.mods.camera_public import CameraPublic

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger(__name__)


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
    cam = CameraPublic(QUALITY=95, TARGET_FPS=30)
    ball_cfg = BallDetectionConfig()
    ball = BallDetection(config=ball_cfg)

    picam2 = cam.start_camera()
    logger.debug("Camera started")
    time.sleep(1)  # warm up

    prev_time = time.time()

    try:
        while True:
            frame = cam.get_frame()

            # --- ここに処理を書く（例：何もしない） ---
            red = cam.extract_red_bgr(img=frame)
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
