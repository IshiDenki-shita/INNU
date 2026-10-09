# 赤色領域からボールを検出し、方向と距離を推定する処理です。
# 座標履歴による安定化と検出結果の可視化を行います。

import cv2

from Experiment.mods.camera_public import CameraPublic
from Experiment.mods.ball_detection import BallDetection
from Experiment.mods.logger import logger

if __name__ == "__main__":
    cam = CameraPublic(TARGET_FPS=30)
    ball = BallDetection()

    cam.start_camera()
    logger.debug("Camera started")

    try:
        while True:
            img = cam.get_frame()
            red = cam.extract_red_bgr(img=img)
            clean = cam.remove_noise(red)

            x, y = ball.run(red=red, is_xy=True)

            overlay = ball.draw_detection_overlay(
                original_img=clean, red_mask=clean, center_x=int(x), center_y=int(y)
            )
            cv2.imshow("Red Ball Detection", overlay)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        cv2.destroyAllWindows()
