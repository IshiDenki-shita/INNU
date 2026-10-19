"""
赤と捉えた範囲が正しいか目視確認するスクリプト (RGB -> BGR 変換適用版)

Picamera2 から取得した RGB 画像を OpenCV 用の BGR 画像へ変換し、
皮膚などが青く映る色反転を補正した上で、既存の BGR 赤色抽出ロジックで確認します。
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
            # 1. カメラから生のフレーム（RGB形式）を取得
            frame_rgb = cam.get_frame()
            if frame_rgb is None:
                continue

            # 2. RGB から BGR へ色空間を変換（皮膚の青かぶり・色反転を修正）
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            # 3. 補正後の BGR 画像に対して既存の赤色抽出およびノイズ除去を実行
            red_bin = cam.extract_red_bgr(frame_bgr)
            clean_red = cam.remove_noise(red_bin)

            # 4. 表示用に 1 チャンネルのマスク画像を BGR 3 チャンネルへ変換
            mask_bgr = cv2.cvtColor(clean_red, cv2.COLOR_GRAY2BGR)

            # 5. 元映像（色補正済）と抽出結果（マスク）を横並びに結合して表示
            display = cv2.hconcat([frame_bgr, mask_bgr])
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
