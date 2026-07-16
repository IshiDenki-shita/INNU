import sys
import logging
from pathlib import Path
from typing import Tuple
from dataclasses import dataclass
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
    PHOTO_PATH: Path = Path("Experiment/ball_detection/sample/RedBall.jpeg")
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
    # find contour
    MIN_CIRCLE_SIZE: int = 100
    THRESH_CIRCULARITY: float = 0.8
    # calc ball position
    theta_board = np.array([make pixel-theta matching when the camera condition confirmed])
    distance_correction_val: np.float16 = this val will be decided by experiment



class Utility:
    def __init__(self) -> None: ...


class CameraPublic:
    def __init__(self, config: CameraConfig):
        self.cfg = config
        self.cam = None

    def start_camera(self):
        self.cam = Picamera2()

        config = self.cam.create_preview_configuration(
            main={"size": (self.cfg.WIDTH, self.cfg.HEIGHT), "format": "BGR888"},
            controls={
                "FrameDurationLimits": (
                    int(1000000 / self.cfg.TARGET_FPS),
                    int(1000000 / self.cfg.TARGET_FPS),
                )
            },
        )
        self.cam.configure(config)
        self.cam.start()

    def get_frame(self) -> np.ndarray:
        return self.cam.capture_array()

    def get_sample_photo(self):
        img = cv2.imread(str(self.cfg.PHOTO_PATH))

        if img is None:
            raise ValueError("サンプル画像を取得できませんでした。")

        return img

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

        # 赤〜ピンクまで広めに許容

        # 0付近
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([20, 255, 255])

        # 180付近
        lower_red2 = np.array([160, 50, 70])
        upper_red2 = np.array([180, 255, 255])

        # マスク作成
        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)

        red_bin = cv2.bitwise_or(mask1, mask2)

        return red_bin


class BallDetection:
    def __init__(self, config: BallDetectionConfig) -> None:
        self.cfg = config

    def find_circle_contour(self, red: np.ndarray) -> Tuple[int, int, float]:

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
            circularity = 4 * np.pi * area / (perimeter * perimeter)

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

    def calc_ball_position(self, crood: Tuple[int, int], S: float) -> Tuple[float, float]:
        # direction
        theta_board = self.cfg.theta_board
        theta = theta_board[crood]

        # distance
        distance = np.sqrt(S, dtype='float16') / self.distance_correction_val # meter

        return theta, distance


    def plot_red_detection(
        self,
        original_img: np.ndarray,
        red_mask: np.ndarray,
    ) -> None:

        import matplotlib.pyplot as plt

        contours, hierarchy = cv2.findContours(
            image=red_mask,
            mode=cv2.RETR_EXTERNAL,
            method=cv2.CHAIN_APPROX_NONE,
        )

        overlay_img = original_img.copy()
        overlay_img[red_mask > 0] = [0, 0, 255]

        overlay_img_rgb = cv2.cvtColor(
            overlay_img,
            cv2.COLOR_BGR2RGB,
        )
        contour_img = cv2.cvtColor(
            red_mask,
            cv2.COLOR_GRAY2BGR,
        )

        cv2.drawContours(
            image=contour_img,
            contours=contours,
            contourIdx=-1,
            color=(0, 255, 0),
            thickness=2,
        )

        contour_img_rgb = cv2.cvtColor(
            contour_img,
            cv2.COLOR_BGR2RGB,
        )

        fig, axes = plt.subplots(
            1,
            2,
            figsize=(14, 6),
        )

        # 画像1
        axes[0].imshow(overlay_img_rgb)
        axes[0].set_title("Original + Extracted Red")
        axes[0].axis("off")

        # 画像2
        axes[1].imshow(contour_img_rgb)
        axes[1].set_title("Binary + Contours")
        axes[1].axis("off")

        plt.tight_layout()

        plt.show()

    def draw_detection_overlay(
        self,
        original_img: np.ndarray,
        red_mask: np.ndarray,
        center_x: int,
        center_y: int,
    ) -> np.ndarray:
        overlay_img = original_img.copy()
        overlay_img[red_mask > 0] = [0, 0, 255]

        if center_x >= 0 and center_y >= 0:
            cv2.circle(overlay_img, (center_x, center_y), 6, (0, 255, 0), thickness=-1)
            cv2.drawMarker(
                overlay_img,
                (center_x, center_y),
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

    cam.start_camera()
    logger.debug("Camera started")

    try:
        while True:
            img = cam.get_frame()
            red = cam.extract_red_hsv(img=img)
            clean = cam.remove_noise(red)

            x, y, S = ball.find_circle_contour(red=clean)
            logger.debug(f"Detected center: (x={x}, y={y}), area={S}")

            overlay = ball.draw_detection_overlay(
                original_img=clean, red_mask=clean, center_x=x, center_y=y
            )
            cv2.imshow("Red Ball Detection", overlay)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        cv2.destroyAllWindows()



