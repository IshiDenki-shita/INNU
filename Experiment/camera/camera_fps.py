import time
from pathlib import Path
import datetime
import numpy as np
import cv2
from picamera import Picamera

PHOTO_PATH = Path("Experiment/camera/photos")
QUALITY = 95

WIDTH = 640
HEIGHT = 480

TARGET_FPS = 10
FRAME_INTERVAL = 1.0 / TARGET_FPS

cap = cv2.VideoCapture(0)  # Camera Module v2
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

# ウォームアップ
time.sleep(1)

prev_time = time.time()

while True:
    start_time = time.time()

    ret, frame = cap.read()
    if not ret:
        continue

    # --- ここに処理を書く（例：何もしない） ---
    # 処理を追加してもFPSはこの後で制御される

    elapsed = time.time() - start_time
    sleep_time = FRAME_INTERVAL - elapsed

    if sleep_time > 0:
        time.sleep(sleep_time)

    # 実FPS計測（確認用）
    now = time.time()
    actual_fps = 1.0 / (now - prev_time)
    prev_time = now

    print(f"Target FPS: {TARGET_FPS}, Actual FPS: {actual_fps:.2f}")
