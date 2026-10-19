# 座標履歴を用いて赤いボールの検出を安定化する実験コードです。
# 検出位置から方向・距離を算出し、結果を画像上に表示します。

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
            if img is None:
                continue

            # 赤色抽出とノイズ除去
            red = cam.extract_red_bgr(img=img)
            clean = cam.remove_noise(red)

            # ノイズ除去後のマスク画像(clean)を用いてボール位置を検出
            x, y, is_detected = ball.run(red=clean, is_xy=True)

            # 1チャンネル(グレースケール/2値)のマスクであることを保証
            if len(clean.shape) == 3:
                mask_single = cv2.cvtColor(clean, cv2.COLOR_BGR2GRAY)
            else:
                mask_single = clean

            # 元のカラー画像(img)の上に検出結果と赤色マスクをオーバーレイ描画
            overlay = ball.draw_detection_overlay(
                original_img=img,
                red_mask=mask_single,
                center_x=int(x),
                center_y=int(y),
            )
            cv2.imshow("Red Ball Detection", overlay)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        cam.stop_camera()
        cv2.destroyAllWindows()
