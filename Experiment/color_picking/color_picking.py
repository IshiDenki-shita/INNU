"""
カメラの色認識（赤色検出）のHSV閾値をキャリブレーションするコード

1. ラズベリーパイのカメラで写真を撮る
2. 表示された画像上で赤い部分をマウスクリックでマーク
3. クリックしたピクセルのBGR/HSV値を保存
4. 赤色のHSV範囲を計算し直す
"""

import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
from picamera2 import Picamera2


@dataclass(frozen=True)
class ColorCalibrationConfig:
    # input/output path
    PHOTO_PATH: Path = Path("Experiment/color_picking/photos")
    PHOTO_NAME: str = "capture.jpg"
    SAMPLES_PATH: Path = Path("Experiment/color_picking/samples")
    SAMPLES_NAME: str = "red_samples.npz"
    # img property
    WIDTH: int = 640
    HEIGHT: int = 480
    # warm up
    WARM_UP_SEC: float = 2.0
    # HSV範囲計算時のマージン
    HUE_MARGIN: int = 10
    SAT_MARGIN: int = 40
    VAL_MARGIN: int = 40


class CameraCapture:
    def __init__(self, config: ColorCalibrationConfig):
        self.cfg = config
        self.cfg.PHOTO_PATH.mkdir(parents=True, exist_ok=True)

    def capture_photo(self) -> np.ndarray:
        picam2 = Picamera2()

        config = picam2.create_still_configuration(
            main={"size": (self.cfg.WIDTH, self.cfg.HEIGHT)}
        )
        picam2.configure(config)
        picam2.start()

        time.sleep(self.cfg.WARM_UP_SEC)  # 露出・ホワイトバランス安定待ち

        image_rgb = picam2.capture_array()
        picam2.stop()

        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        save_path = self.cfg.PHOTO_PATH / self.cfg.PHOTO_NAME
        cv2.imwrite(str(save_path), image_bgr)
        print(f"保存しました: {save_path}")

        return image_bgr


class PixelPicker:
    """
    画像を表示し、クリックされたピクセルのBGR/HSV値を収集する
    """

    def __init__(self, config: ColorCalibrationConfig):
        self.cfg = config
        self.cfg.SAMPLES_PATH.mkdir(parents=True, exist_ok=True)

        self.picked_bgr: List[Tuple[int, int, int]] = []
        self.image_bgr: Optional[np.ndarray] = None
        self.image_hsv: Optional[np.ndarray] = None
        self.display: Optional[np.ndarray] = None
        self.window_name = "赤い部分をクリック（終了は q）"

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.image_bgr is None:
            return

        bgr = tuple(int(v) for v in self.image_bgr[y, x])
        hsv = tuple(int(v) for v in self.image_hsv[y, x])
        self.picked_bgr.append(bgr)

        cv2.circle(self.display, (x, y), 4, (0, 255, 0), -1)
        cv2.imshow(self.window_name, self.display)

        print(f"pick: (x={x}, y={y}) BGR={bgr} HSV={hsv}")

    def pick_pixels(self, image_bgr: np.ndarray) -> List[Tuple[int, int, int]]:
        self.image_bgr = image_bgr
        self.image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        self.display = image_bgr.copy()

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._on_mouse)
        cv2.imshow(self.window_name, self.display)

        print("赤い部分を左クリックしてください。終了する場合は q キー。")
        while True:
            key = cv2.waitKey(20) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()

        return self.picked_bgr

    def save_samples(self, samples_bgr: List[Tuple[int, int, int]]) -> Path:
        if not samples_bgr:
            raise RuntimeError("サンプルが1つも選択されていません")

        bgr_arr = np.array(samples_bgr, dtype=np.uint8)
        hsv_arr = cv2.cvtColor(bgr_arr.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(
            -1, 3
        )

        save_path = self.cfg.SAMPLES_PATH / self.cfg.SAMPLES_NAME
        np.savez(save_path, bgr=bgr_arr, hsv=hsv_arr)
        print(f"サンプルを保存しました: {save_path}")

        return save_path


class RedRangeCalculator:
    """
    収集したサンプルから赤色のHSV範囲（lower_red1/2, upper_red1/2）を計算し直す
    """

    def __init__(self, config: ColorCalibrationConfig):
        self.cfg = config

    def calc_range(
        self, hsv_samples: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if hsv_samples.size == 0:
            raise RuntimeError("サンプルがありません")

        hue = hsv_samples[:, 0].astype(np.int32)
        sat = hsv_samples[:, 1].astype(np.int32)
        val = hsv_samples[:, 2].astype(np.int32)

        # 赤はHue=0付近とHue=180付近の両方にまたがるので、
        # Hue<=90（0側）とHue>90（180側）の2グループに分けて集計する
        low_hue = hue[hue <= 90]
        high_hue = hue[hue > 90]

        sat_min = max(int(sat.min()) - self.cfg.SAT_MARGIN, 0)
        sat_max = min(int(sat.max()) + self.cfg.SAT_MARGIN, 255)
        val_min = max(int(val.min()) - self.cfg.VAL_MARGIN, 0)
        val_max = min(int(val.max()) + self.cfg.VAL_MARGIN, 255)

        if low_hue.size > 0:
            lower_red1 = np.array(
                [max(int(low_hue.min()) - self.cfg.HUE_MARGIN, 0), sat_min, val_min]
            )
            upper_red1 = np.array(
                [min(int(low_hue.max()) + self.cfg.HUE_MARGIN, 10), sat_max, val_max]
            )
        else:
            lower_red1 = np.array([0, sat_min, val_min])
            upper_red1 = np.array([10, sat_max, val_max])

        if high_hue.size > 0:
            lower_red2 = np.array(
                [max(int(high_hue.min()) - self.cfg.HUE_MARGIN, 170), sat_min, val_min]
            )
            upper_red2 = np.array(
                [min(int(high_hue.max()) + self.cfg.HUE_MARGIN, 180), sat_max, val_max]
            )
        else:
            lower_red2 = np.array([170, sat_min, val_min])
            upper_red2 = np.array([180, sat_max, val_max])

        return lower_red1, upper_red1, lower_red2, upper_red2


if __name__ == "__main__":
    cfg = ColorCalibrationConfig()

    camera = CameraCapture(cfg)
    photo = camera.capture_photo()

    picker = PixelPicker(cfg)
    samples_bgr = picker.pick_pixels(photo)
    picker.save_samples(samples_bgr)

    hsv_samples = cv2.cvtColor(
        np.array(samples_bgr, dtype=np.uint8).reshape(-1, 1, 3), cv2.COLOR_BGR2HSV
    ).reshape(-1, 3)

    calculator = RedRangeCalculator(cfg)
    lower_red1, upper_red1, lower_red2, upper_red2 = calculator.calc_range(hsv_samples)

    print("--- 新しい赤色HSV範囲 ---")
    print(f"lower_red1 = {lower_red1}")
    print(f"upper_red1 = {upper_red1}")
    print(f"lower_red2 = {lower_red2}")
    print(f"upper_red2 = {upper_red2}")
