import sys
import logging
from pathlib import Path
from typing import Tuple
from dataclasses import dataclass
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
    # find contour
    MIN_CIRCLE_SIZE: int = 100
    THRESH_CIRCULARITY: float = 0.8


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
    cam = CameraPublic(TARGET_FPS=30)
    ball_cfg = BallDetectionConfig()
    ball = BallDetection(config=ball_cfg)

    cam.start_camera()
    logger.debug("Camera started")

    try:
        while True:
            img = cam.get_frame()
            red = cam.extract_red_bgr(img=img)
            clean = cam.remove_noise(red)

            x, y, S = ball.find_circle_contour(red=clean)
            logger.debug(f"Detected center: (x={x}, y={y}), area={S}")

            overlay = ball.draw_detection_overlay(
                original_img=img, red_mask=clean, center_x=x, center_y=y
            )
            cv2.imshow("Red Ball Detection", overlay)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        cv2.destroyAllWindows()
