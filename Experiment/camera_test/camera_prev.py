#!/usr/bin/env python3

import time
import cv2
from pathlib import Path
from picamera2 import Picamera2

save_path = Path("/home/ubuntu/Desktop/INNU/Experiment/camera_test/photos")
save_path.mkdir(exist_ok=True)
save_path = Path("/home/ubuntu/Desktop/INNU/Experiment/camera_test/photos/capture")


def main():
    # カメラ初期化
    picam2 = Picamera2()

    config = picam2.create_still_configuration()
    picam2.configure(config)

    picam2.start()

    # カメラの露出・ホワイトバランスが安定するまで待機
    time.sleep(2)

    # 撮影
    image = picam2.capture_array()

    picam2.stop()

    # BGRへ変換（OpenCV表示用）
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # 保存
    cv2.imwrite(save_path, image)
    print(f"保存しました: {save_path}")

    # 表示
    cv2.imshow("Captured Image", image)
    print("何かキーを押すと終了します。")

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
