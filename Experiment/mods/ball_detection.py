"""
red画像を受け取って、ロボットからみたボールの座標を返すためのクラス
"""

from typing import Tuple
import numpy as np
import cv2

from Experiment.mods.logger import logger


class BallDetection:

    def __init__(
        self,
        # find contour
        # ↓ボールと認識する最小の円の面積
        MIN_CIRCLE_SIZE: int = 100,
        # ↓ボールと認識する最小の円形度
        THRESH_CIRCULARITY: float = 0.8,
        # calc ball position
        # ↓画像上のxy座標とロボットからみたthetaの変換表
        theta_board: np.ndarray = np.array([]),
        # ↓画像上の面積とロボットからの距離の較正値
        distance_correction_val: np.float16 = np.float16(1.0),
        # detection hysterises
        hysterises_length: int = 10,  # ←ボールの座標の履歴の長さ
        THRESH_INTENCITY: float = 30.0,  # ←履歴の座標の分散の閾値
    ) -> None:

        self.MIN_CIRCLE_SIZE: int = MIN_CIRCLE_SIZE
        self.THRESH_CIRCULARITY: float = THRESH_CIRCULARITY
        self.theta_board: np.ndarray = theta_board
        self.distance_correction_val: np.float16 = distance_correction_val
        self.hysterises_length: int = hysterises_length
        self.THRESH_INTENCITY: float = THRESH_INTENCITY

        # detection hysterises
        self.x_history = np.asarray([0] * self.hysterises_length, dtype=np.float32)
        self.y_history = np.asarray([0] * self.hysterises_length, dtype=np.float32)
        self.history_idx = 0

    def run(self, red, is_xy=False):

        x, y, S = self.find_circle_contour(red=red)
        logger.debug(f"Detected center: (x={x}, y={y}), area={S}")

        x, y = self.calc_crood_with_hysteresis(x, y)

        if (x, y) == (-1, -1):
            return 0.0, 0.0, 0  # ロスト時のtheta, distance, detected

        if is_xy:
            return x, y, 1
        else:
            return self.calc_ball_position((x, y), S)

    def find_circle_contour(self, red: np.ndarray) -> Tuple[int, int, float]:

        contours, hierarchy = cv2.findContours(
            image=red, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE
        )

        best_contour = None
        best_circularity = 0
        best_area = 0
        for contour in contours:
            area = cv2.contourArea(contour=contour, oriented=False)

            if area < self.MIN_CIRCLE_SIZE:
                continue

            perimeter = cv2.arcLength(curve=contour, closed=True)
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            if circularity < self.THRESH_CIRCULARITY:
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

    def calc_crood_with_hysteresis(self, x, y):
        """
        直近10フレームのボールの座標の重心を目標座標として決定
        """
        idx = self.history_idx % self.hysterises_length

        self.x_history[idx] = x
        self.y_history[idx] = y
        self.history_idx += 1

        x_ave = np.mean(self.x_history)
        y_ave = np.mean(self.y_history)

        # 履歴点が1つの中心にどれくらい集まっているかを、平均二乗距離で計算
        intensity = np.mean(
            (self.x_history - x_ave) ** 2 + (self.y_history - y_ave) ** 2
        )

        if intensity > self.THRESH_INTENCITY:
            return (-1, -1)

        ans = (int(x_ave), int(y_ave))

        return ans

    def calc_ball_position(
        self, crood: Tuple[int, int], S: float
    ) -> Tuple[float, float, int]:
        # direction
        theta_board = self.theta_board
        theta = theta_board[crood]

        # distance
        distance = np.sqrt(S, dtype="float16") / self.distance_correction_val  # meter

        return theta, distance, 1  # 最後の1はボール検出成功のフラグ(detectedフラグ)

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
