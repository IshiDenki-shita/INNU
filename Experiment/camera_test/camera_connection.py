"""
カメラと接続できるか試すコード
"""

#!/usr/bin/env python3

from picamera2 import Picamera2

try:
    print("カメラを初期化中...")
    picam2 = Picamera2()

    print("プレビュー設定を作成中...")
    config = picam2.create_preview_configuration()

    print("設定を適用中...")
    picam2.configure(config)

    print("カメラを起動中...")
    picam2.start()

    print("✅ カメラの接続に成功しました。")

    picam2.stop()

except Exception as e:
    print("❌ カメラの接続に失敗しました。")
    print(type(e).__name__)
    print(e)
