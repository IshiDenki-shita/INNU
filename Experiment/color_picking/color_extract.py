"""
色認識キャリブレーション用: 保存済み画像から赤色をピックし、HSV範囲を計算するコード

1. color_capture.py で撮影した画像を読み込む
2. 表示された画像上で赤い部分をマウスクリックでマーク
3. クリックしたピクセルのBGR/HSV値を保存
4. 赤色のHSV範囲を計算し直す

picamera2 に依存しないため Mac 上で実行できる。
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class ExtractConfig:
    PHOTO_PATH: Path = Path("Experiment/color_picking/photos")
    PHOTO_NAME: str = "capture.jpg"
    SAMPLES_PATH: Path = Path("Experiment/color_picking/samples")
    SAMPLES_NAME: str = "red_samples.npz"
    # BGR範囲計算時のマージン
    B_MARGIN: int = 40
    G_MARGIN: int = 40
    R_MARGIN: int = 40


class PixelPicker:
    """
    画像を表示し、クリックされたピクセルのBGR/HSV値を収集する
    """

    def __init__(self, config: ExtractConfig):
        self.cfg = config
        self.cfg.SAMPLES_PATH.mkdir(parents=True, exist_ok=True)
        self.picked_bgr: List[Tuple[int, int, int]] = []
        self.image_bgr: Optional[np.ndarray] = None
        self.display: Optional[np.ndarray] = None
        self.window_name = "赤い部分をクリック（終了は q）"

    def load_photo(self) -> np.ndarray:
        photo_path = self.cfg.PHOTO_PATH / self.cfg.PHOTO_NAME
        image_bgr = cv2.imread(str(photo_path))

        if image_bgr is None:
            raise RuntimeError(f"画像を読み込めませんでした: {photo_path}")

        return image_bgr

    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.image_bgr is None:
            return

        bgr = tuple(int(v) for v in self.image_bgr[y, x])
        self.picked_bgr.append(bgr)

        cv2.circle(self.display, (x, y), 4, (0, 255, 0), -1)
        cv2.imshow(self.window_name, self.display)

        print(f"pick: (x={x}, y={y}) BGR={bgr}")

    def pick_pixels(self, image_bgr: np.ndarray) -> List[Tuple[int, int, int]]:
        self.image_bgr = image_bgr
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

        save_path = self.cfg.SAMPLES_PATH / self.cfg.SAMPLES_NAME
        np.savez(save_path, bgr=bgr_arr)
        print(f"サンプルを保存しました: {save_path}")

        return save_path


class RedRangeCalculator:
    """
    収集したサンプルから赤色のBGR範囲（lower_bgr, upper_bgr）を計算し直す
    """

    def __init__(self, config: ExtractConfig):
        self.cfg = config

    def calc_range(self, bgr_samples: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if bgr_samples.size == 0:
            raise RuntimeError("サンプルがありません")

        b = bgr_samples[:, 0].astype(np.int32)
        g = bgr_samples[:, 1].astype(np.int32)
        r = bgr_samples[:, 2].astype(np.int32)

        lower_bgr = np.array(
            [
                max(int(b.min()) - self.cfg.B_MARGIN, 0),
                max(int(g.min()) - self.cfg.G_MARGIN, 0),
                max(int(r.min()) - self.cfg.R_MARGIN, 0),
            ]
        )
        upper_bgr = np.array(
            [
                min(int(b.max()) + self.cfg.B_MARGIN, 255),
                min(int(g.max()) + self.cfg.G_MARGIN, 255),
                min(int(r.max()) + self.cfg.R_MARGIN, 255),
            ]
        )

        return lower_bgr, upper_bgr


if __name__ == "__main__":
    cfg = ExtractConfig()

    picker = PixelPicker(cfg)
    photo = picker.load_photo()
    samples_bgr = picker.pick_pixels(photo)
    picker.save_samples(samples_bgr)

    bgr_samples = np.array(samples_bgr, dtype=np.uint8)

    calculator = RedRangeCalculator(cfg)
    lower_bgr, upper_bgr = calculator.calc_range(bgr_samples)

    print("--- 新しい赤色BGR範囲 ---")
    print(f"lower_bgr = {lower_bgr}")
    print(f"upper_bgr = {upper_bgr}")
