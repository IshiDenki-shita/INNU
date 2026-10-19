"""
赤と捉えた範囲が正しいか目視確認するスクリプト

CameraPublic から取得した BGR 画像に対して
既存の BGR 赤色抽出ロジックを実行し、リアルタイムで確認します。
'q' キーを押すと終了します。
"""

import sys
import cv2
from Experiment.mods.camera_public import CameraPublic


def main():
    cam = CameraPublic()

    try:
        print("カメラを起動しています...")
        cam.start_camera()
        print("リアルタイム表示を開始します。('q' キーで終了)")

        while True:
            # 1. カメラから BGR 形式のフレームを取得 (CameraPublic 内で変換済)
            frame = cam.get_frame()
            if frame is None:
                continue

            # 2. BGR 画像に対して赤色抽出およびノイズ除去を実行
            red_bin = cam.extract_red_bgr(frame)
            clean_red = cam.remove_noise(red_bin)

            # 3. 表示用に 1 チャンネルのマスク画像を BGR 3 チャンネルへ変換
            mask_bgr = cv2.cvtColor(clean_red, cv2.COLOR_GRAY2BGR)

            # 4. 元映像と抽出結果（マスク）を横並びに結合して表示
            display = cv2.hconcat([frame, mask_bgr])
            cv2.imshow("Red Sight (Left: BGR Frame / Right: Red Mask)", display)

            # 'q' キーでループ終了
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except Exception as e:
        print(f"エラーが発生しました: {e}", file=sys.stderr)

    finally:
        print("カメラを停止し、リソースを解放します...")
        cam.stop_camera()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
