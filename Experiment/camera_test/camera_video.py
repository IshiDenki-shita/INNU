"""
ラズパイのカメラモジュールで撮影した映像をリアルタイムでデスクトップに表示する
"""

from dataclasses import dataclass

import cv2
from picamera2 import Picamera2


@dataclass(frozen=True)
class CameraConfig:
    # img property
    WIDTH: int = 640
    HEIGHT: int = 480
    # FPS
    TARGET_FPS: int = 30
    # window
    WINDOW_NAME: str = "INNU Camera"


class CameraViewer:
    def __init__(self, config: CameraConfig):
        self.cfg = config
        self.picam2 = None

    def start(self):
        self.picam2 = Picamera2()

        config = self.picam2.create_preview_configuration(
            main={"size": (self.cfg.WIDTH, self.cfg.HEIGHT), "format": "BGR888"},
            controls={
                "FrameDurationLimits": (
                    int(1000000 / self.cfg.TARGET_FPS),
                    int(1000000 / self.cfg.TARGET_FPS),
                )
            },
        )
        self.picam2.configure(config)
        self.picam2.start()

    def get_frame(self):
        if self.picam2 is None:
            raise RuntimeError("カメラが起動されていません")

        return self.picam2.capture_array()

    def show(self, frame):
        cv2.imshow(self.cfg.WINDOW_NAME, frame)

    def stop(self):
        if self.picam2 is not None:
            self.picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cam_cfg = CameraConfig()
    viewer = CameraViewer(cam_cfg)

    try:
        viewer.start()

        while True:
            frame = viewer.get_frame()
            viewer.show(frame)

            # 'q'キーで終了
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        viewer.stop()
