"""
画像認識・ボール追従 メインプログラム (app.py)

カメラから映像を取得し、赤ボールの抽出・相対座標(方位角θ, 距離d)の算出を行い、
通信仕様に従って115200bps / 40Hz で LiDAR ラズパイへ UART 送信します。
"""

import signal
import time
from gpiozero import LED

from Experiment.mods.ball_detection import BallDetection
from Experiment.mods.camera_public import CameraPublic
from Experiment.mods.logger import logger
from Experiment.mods.uart_sender import UARTSender


def main():
    # 0. デバッグ用LEDの初期化 (A:赤=16, B:黄=20, C:緑=21)[span_1](start_span)[span_1](end_span)
    led_red = LED(16)
    led_yellow = LED(20)
    led_green = LED(21)

    # 1. 各モジュールの初期化 (通信仕様: 115200 bps / 40 Hz)
    camera = CameraPublic(TARGET_FPS=40)
    detector = BallDetection()
    uart = UARTSender(
        port="/dev/serial0",
        baudrate=115200,
        max_send_hz=40.0,
    )

    # 2. 安全な終了処理用フラグとシグナルハンドラの設定
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        logger.info("終了信号を受け取りました。安全に停止処理を実行します...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 3. カメラ・UARTの起動
    logger.info("カメラおよびUART送信モジュールを起動しています...")
    try:
        camera.start_camera()
        uart.start()
        # カメラとの接続が正常な場合、赤LEDをON[span_2](start_span)[span_2](end_span)
        led_red.on()
        logger.info("メインループを開始しました。(Ctrl+C で停止)")
    except Exception as e:
        logger.error(f"モジュールの起動に失敗しました: {e}")
        # カメラとの接続に問題がある場合、赤LEDを点滅[span_3](start_span)[span_3](end_span)
        led_red.blink()
        time.sleep(3)  # エラー点滅を目視確認するための待機
        led_red.off()
        return

    uart_send_count = 0

    # 4. メインループ
    try:
        while running:
            # (1) フレーム取得
            frame = camera.get_frame()
            if frame is None:
                logger.warning("フレームの取得に失敗しました。")
                time.sleep(0.01)
                continue

            # (2) 赤色抽出およびノイズ除去
            red_bin = camera.extract_red_bgr(frame)
            clean_red = camera.remove_noise(red_bin)

            # (3) ボール検出と相対座標(theta, distance, detected)の算出
            theta, distance, detected = detector.run(clean_red)
            detected_bool = bool(detected)

            # ボール検出状態に応じて黄LEDを操作[span_4](start_span)[span_4](end_span)
            if detected_bool:
                led_yellow.on()  # 赤ボールを検出
            else:
                led_yellow.off()  # 赤ボールをロスト

            # (4) UARTフレーム送信
            sent = uart.send(
                theta_rad=float(theta),
                distance_m=float(distance),
                detected=detected_bool,
            )

            # UART送信状態に応じて緑LEDを操作[span_5](start_span)[span_5](end_span)
            if sent:
                if uart_send_count % 2 == 0:
                    led_green.on()  # 送信完了(偶数回)
                else:
                    led_green.off()  # 送信完了(奇数回)

                uart_send_count += 1

                # ログ出力
                if detected_bool:
                    logger.info(
                        f"[UART送信] θ: {theta:.3f} rad, d: {distance:.3f} m, det: {detected_bool}"
                    )
                else:
                    logger.info("[UART送信] ボール未検出 (detected=False)")

            # 40Hzの周期維持用スリープ
            time.sleep(1.0 / 40.0)

    except Exception as e:
        logger.error(f"実行中にエラーが発生しました: {e}", exc_info=True)
        # その他エラー発生時は全LEDをOFFにする[span_6](start_span)[span_6](end_span)
        led_red.off()
        led_yellow.off()
        led_green.off()

    # 5. リソースの解放処理
    finally:
        logger.info("リソースを解放中...")
        # 終了時も全LEDを安全に消灯[span_7](start_span)[span_7](end_span)
        led_red.off()
        led_yellow.off()
        led_green.off()

        try:
            camera.stop_camera()
        except Exception as e:
            logger.error(f"カメラ停止エラー: {e}")

        try:
            uart.stop()
        except Exception as e:
            logger.error(f"UART停止エラー: {e}")

        logger.info("アプリケーションを正常に停止しました。")


if __name__ == "__main__":
    main()
