# カメラ映像で赤色として抽出される領域を目視確認するスクリプトです。
# CameraPublicのライブ表示機能を使って検出結果を表示します。
"""
赤と捉えた範囲が正しいか目視で確認するコード
"""

from Experiment.mods.camera_public import CameraPublic

if __name__ == "__main__":
    cam = CameraPublic()
