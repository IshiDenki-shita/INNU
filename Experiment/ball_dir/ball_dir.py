import math
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import cv2
import matplotlib.pyplot as plt


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
    # calc ball position
    theta_board = np.array([make pixel-theta matching when the camera condition confirmed])
    distance_correction_val: np.float16 = this val will be decided by experiment


class Utility:
    def __init__(self) -> None: ...


class CameraPublic:
    def __init__(self, config: CameraConfig):
        self.cfg = config
        # self.cfg.PHOTO_PATH.mkdir(parents=True, exist_ok=True)

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

    def visualize_detection(
        self,
        img: np.ndarray,
        red_mask: np.ndarray,
        center_x: int,
        center_y: int,
    ) -> None:

        vis = img.copy()
        overlay = np.zeros_like(vis)
        overlay[:, :, 2] = red_mask
        vis = cv2.addWeighted(vis, 1.0, overlay, 0.5, 0)

        if center_x >= 0 and center_y >= 0:
            cross_size = 15
            thickness = 3
            color = (0, 255, 0)
            cv2.line(
                vis,
                (center_x - cross_size, center_y - cross_size),
                (center_x + cross_size, center_y + cross_size),
                color,
                thickness,
            )
            cv2.line(
                vis,
                (center_x + cross_size, center_y - cross_size),
                (center_x - cross_size, center_y + cross_size),
                color,
                thickness,
            )

            cv2.putText(
                img=vis,
                text=f"({center_x}, {center_y})",
                org=(center_x + 20, center_y - 20),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.7,
                color=color,
                thickness=2,
            )

        vis_rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(10, 7))
        plt.imshow(vis_rgb)
        plt.title("Red Ball Detection")
        plt.axis("off")
        plt.show()

    def calc_ball_position(self, crood: Tuple[int, int], S: float) -> Tuple[float, float]:
        # direction
        theta_board = self.cfg.theta_board
        theta = theta_board[crood]

        # distance
        distance = np.sqrt(S, dtype='float16') / self.distance_correction_val # meter

        return theta, distance


if __name__ == "__main__":
    cam_cfg = CameraConfig()
    cam = CameraPublic(config=cam_cfg)
    ball_cfg = BallDetectionConfig()
    ball = BallDetection(config=ball_cfg)

    # testing
    img = cam.get_sample_photo()
    red = cam.extract_red_hsv(img=img)
    clean = cam.remove_noise(red)
    x, y, S = ball.find_circle_contour(red=clean)
    theta, distance = ball.calc_ball_position(crood=(x,y), S=S)

    print(f"x={x}, y={y}, area={S}, theta={theta}, distance={distance}")

    ball.visualize_detection(
        img=img,
        red_mask=clean,
        center_x=x,
        center_y=y,
    )
