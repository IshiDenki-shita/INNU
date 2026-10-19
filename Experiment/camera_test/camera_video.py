# Picamera2の映像をリアルタイム表示するカメラ確認用スクリプトです。
# 必要に応じて赤色マスクを重ね、終了時にカメラを停止します。
"""
ラズパイのカメラモジュールで撮影した映像をリアルタイムでデスクトップに表示する
"""

from Experiment.mods.camera_public import CameraPublic

if __name__ == "__main__":
    cam = CameraPublic()

    try:
        cam.start_camera()
        cam.show_live(show_mask=True)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        if cam.cam is not None:
            cam.cam.stop()
