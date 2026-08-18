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

from Experiment.mods.camera_public import CameraPublic

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger(__name__)


def prepare_photo_dir(photo_path):
    logger.debug("prepare_photo_dir() called")
    photo_path.mkdir(parents=True, exist_ok=True)
    logger.debug(f"prepare_photo_dir() succeeded: {photo_path.resolve()}")


def start_camera(WIDTH, HEIGHT, TARGET_FPS) -> Picamera2:
    logger.debug("start_camera() called")
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "BGR888"},
        controls={
            "FrameDurationLimits": (
                int(1000000 / TARGET_FPS),
                int(1000000 / TARGET_FPS),
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
    TARGET_FPS: int = 30
    PHOTO_PATH = Path("Experiment/camera/photos")

    logger.debug("Program started")
    cam = CameraPublic(SAMPLE_PATH=PHOTO_PATH, QUALITY=95, TARGET_FPS=30)
    prepare_photo_dir(PHOTO_PATH)

    picam2 = start_camera(WIDTH=640, HEIGHT=480, TARGET_FPS=TARGET_FPS)
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

        print(f"Target FPS: {TARGET_FPS}\nActual FPS: {fps_history}")
        print(f"FPS mean: {sum(fps_history) / len(fps_history)}")

    finally:
        stop_camera(picam2)
        logger.debug("Program finished")
