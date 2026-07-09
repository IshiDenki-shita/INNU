"""
ラズパイのカメラモジュールで写真が撮れることを確認
"""

import sys
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Tuple
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
    PHOTO_PATH: Path = Path("Experiment/camera_test/photos")
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
        while count < 150:
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
