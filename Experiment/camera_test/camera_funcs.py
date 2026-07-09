"""
ラズパイのカメラモジュールで写真が撮れることを確認
"""

import sys
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Tuple
import numpy as np
import cv2
from picamera2 import Picamera2
import logging

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
    TARGET_FPS: int = 10
    FRAME_INTERVAL: float = 1.0 / TARGET_FPS


def prepare_photo_dir(cfg: CameraConfig):
    logger.debug("prepare_photo_dir() called")
    cfg.PHOTO_PATH.mkdir(parents=True, exist_ok=True)
    logger.debug(f"prepare_photo_dir() succeeded: {cfg.PHOTO_PATH.resolve()}")


def start_camera(cfg: CameraConfig) -> Picamera2:
    logger.debug("start_camera() called")
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={"size": (cfg.WIDTH, cfg.HEIGHT), "format": "BGR888"},
        controls={
            "FrameDurationLimits": (
                int(1000000 / cfg.TARGET_FPS),
                int(1000000 / cfg.TARGET_FPS),
            )
        },
    )
    picam2.configure(config)
    picam2.start()
    logger.debug("start_camera() succeeded: camera is running")

    return picam2


def save_photo(cfg: CameraConfig, img: np.ndarray, photo_name: str):
    logger.debug(f"save_photo() called: photo_name={photo_name}, shape={img.shape}")
    save_path = cfg.PHOTO_PATH / photo_name

    is_success = cv2.imwrite(
        str(save_path), img, [cv2.IMWRITE_JPEG_QUALITY, cfg.QUALITY]
    )
    if not is_success:
        logger.debug(f"save_photo() failed: {save_path}")
        raise RuntimeError(f"画像を保存できませんでした。file_path {save_path}")
    logger.debug(f"save_photo() succeeded: {save_path}")


def remove_noise(cfg: CameraConfig, img: np.ndarray) -> np.ndarray:
    logger.debug("remove_noise() called")
    kernel = np.ones(cfg.MORPH_KERNEL_SHAPE, dtype=np.uint8)

    clean = cv2.morphologyEx(
        src=img,
        op=cv2.MORPH_OPEN,
        kernel=kernel,
        iterations=cfg.MORPH_ITERATION,
    )
    logger.debug("remove_noise() succeeded")
    return clean


def extract_red_hsv(img: np.ndarray) -> np.ndarray:
    logger.debug("extract_red_hsv() called")
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
    count_red = cv2.countNonZero(red_bin)
    logger.debug(f"extract_red_hsv() succeeded: detected red pixels={count_red}")

    return red_bin


def extract_red_bgr(img: np.ndarray) -> np.ndarray: ...


def stop_camera(picam2: Picamera2):
    logger.debug("stop_camera() called")
    picam2.stop()
    logger.debug("stop_camera() succeeded")


if __name__ == "__main__":
    logger.debug("Program started")
    cam_cfg = CameraConfig()
    prepare_photo_dir(cam_cfg)

    picam2 = start_camera(cam_cfg)
    logger.debug("Waiting for camera warm-up")
    time.sleep(1)  # warm up

    prev_time = time.time()
    fps_history = []
    count = 0

    try:
        logger.debug("Capture loop started")
        while count < 500:
            start_time = time.perf_counter()
            frame = picam2.capture_array()

            elapsed = time.perf_counter() - start_time
            fps = 1 / elapsed
            fps_history.append(fps)
            if count % 10 == 0:
                logger.debug(
                    f"Frame {count} captured successfully: shape={frame.shape}, FPS={fps:.2f}"
                )
            count += 1
        logger.debug(f"Capture loop finished: total frames={count}")

        print(f"Target FPS: {cam_cfg.TARGET_FPS}\nActual FPS: {fps_history}")
        print(f"FPS mean: {sum(fps_history) / len(fps_history)}")

    finally:
        stop_camera(picam2)
        logger.debug("Program finished")
