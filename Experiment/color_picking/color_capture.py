# カメラ映像をリアルタイム表示し、キー操作で写真を保存します。
"""
色認識キャリブレーション用の写真を撮影するコード

ライブ映像を確認しながらスペースキーを押すと、
Experiment/color_picking/photos/capture.jpg に写真を保存する。
qキーまたはEscキーで終了する。
"""

from pathlib import Path

import cv2

from Experiment.mods.camera_public import CameraPublic


def main() -> None:
    save_path = Path("Experiment/color_picking/photos/capture.jpg")
    window_name = "Color Capture - SPACE: capture / q: quit"

    cam = CameraPublic()

    try:
        cam.start_camera()
        cv2.namedWindow(window_name)
        print("カメラ映像を表示しています。スペースキーで撮影してください。")
        print("終了するには q キーまたは Esc キーを押してください。")

        while True:
            frame = cam.get_frame()
            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

            if key == ord(" "):
                save_path.parent.mkdir(parents=True, exist_ok=True)
                if not cv2.imwrite(str(save_path), frame):
                    raise RuntimeError(f"写真を保存できませんでした: {save_path}")
                print(f"写真を保存しました: {save_path}")

    finally:
        cv2.destroyAllWindows()
        cam.stop_camera()


if __name__ == "__main__":
    main()
