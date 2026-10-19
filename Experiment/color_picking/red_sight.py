"""
赤と捉えた範囲が正しいか目視で確認するスクリプト

CameraPublic を使用してカメラを起動し、
元映像と赤色抽出マスク画像をリアルタイムで横並び表示します。
'q' キーを押すと安全に終了します。
"""

import sys
from Experiment.mods.camera_public import CameraPublic


def main():
    cam = CameraPublic()

    try:
        print("カメラを起動しています...")
        cam.start_camera()

        print("リアルタイム表示を開始します。('q' キーで終了)")
        # show_mask=True で「元映像」と「赤色抽出結果」を横並び表示
        cam.show_live(show_mask=True)

    except Exception as e:
        print(f"エラーが発生しました: {e}", file=sys.stderr)

    finally:
        print("カメラを停止し、リソースを解放します...")
        cam.stop_camera()


if __name__ == "__main__":
    main()
