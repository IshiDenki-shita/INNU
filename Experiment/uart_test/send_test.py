"""
UART送信テスト用の最小構成コード。
UARTSender(Experiment/mods/uart_sender.py)を使い、
既知のテストパターンを送り続けるだけのプログラム。
C++側のreceive_test.cppと組み合わせて、
送った値と受信した値が一致するか、不正フレームが破棄されるかを確認する。

実行(リポジトリのルートから):
  python3 -m Experiment.uart_test.send_test                       # /dev/serial0、10Hz
  python3 -m Experiment.uart_test.send_test /dev/ttyUSB0 --hz 20  # ポートと送信頻度(最大30Hz)を指定
  python3 -m Experiment.uart_test.send_test --fault               # 20フレームごとに不正フレームを混ぜる
  Ctrl+Cで終了。
"""

import argparse
import math
import time

from Experiment.mods.logger import logger
from Experiment.mods.uart_sender import UARTSender

# --fault指定時に順番に送る不正フレームの種類
FAULT_KINDS = ["crc", "version", "nan", "range", "sof", "sequence"]


def send_fault_frame(uart: UARTSender, kind: str, theta: float, dist: float) -> None:
    """受信側が仕様どおり破棄(またはログ)するかを確認するための不正フレームを送る。"""
    if kind == "sequence":
        # 正しいフレームだがsequenceを1つ飛ばす(通信欠落として記録されるはず)
        uart.advance_sequence()
        uart.send_raw(uart.build_frame(theta, dist, True))
    else:
        if kind == "crc":
            frame = bytearray(uart.build_frame(theta, dist, True))
            frame[13] ^= 0xFF  # CRC下位byteを壊す
        elif kind == "version":
            frame = bytearray(uart.build_frame(theta, dist, True, version=0x02))
        elif kind == "nan":
            frame = bytearray(uart.build_frame(float("nan"), dist, True))
        elif kind == "range":
            frame = bytearray(uart.build_frame(theta, -1.0, True))
        elif kind == "sof":
            frame = bytearray(uart.build_frame(theta, dist, True))
            frame[1] = 0x56  # sof1を壊す
        else:
            raise ValueError(kind)
        uart.send_raw(bytes(frame))
    uart.advance_sequence()
    logger.debug(f"sent FAULT frame: kind={kind}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UART送信テスト")
    parser.add_argument("port", nargs="?", default="/dev/serial0")
    parser.add_argument("--hz", type=float, default=10.0, help="送信頻度(最大30)")
    parser.add_argument("--fault", action="store_true", help="不正フレームも送る")
    args = parser.parse_args()
    if not (0.0 < args.hz <= 50.0):
        parser.error(f"--hzは0より大きく{50.0}以下にしてください")
    return args


if __name__ == "__main__":
    args = parse_args()

    uart = UARTSender(port=args.port)
    uart.start()
    logger.debug("UART test sender started")

    # テストパターン:
    # ・theta_radを-175°〜180°で5°ずつスイープ(180°=πの境界値も含む)
    # ・distance_mは0.5〜2.5mでゆっくり変化
    # ・10回に1回はdetected=Falseを送る(仕様どおりtheta=0.0, distance=0.0になる)
    # ・--fault指定時は20フレームごとに不正フレームを1つ混ぜる
    theta_deg = -180.0
    step_count = 0
    interval = 1.0 / args.hz

    try:
        while True:
            step_count += 1
            detected = step_count % 10 != 0

            theta_deg += 5.0
            if theta_deg > 180.0:
                theta_deg = -175.0
            theta_rad = math.radians(theta_deg)
            distance_m = 1.5 + 1.0 * math.sin(step_count / 10.0)

            if args.fault and step_count % 20 == 5:
                kind = FAULT_KINDS[(step_count // 20) % len(FAULT_KINDS)]
                send_fault_frame(uart, kind, theta_rad, distance_m)
            else:
                seq = uart.sequence
                if uart.send(theta_rad, distance_m, detected):
                    sent_theta = theta_rad if detected else 0.0
                    sent_dist = distance_m if detected else 0.0
                    logger.debug(
                        f"sent: seq={seq:3d} theta={sent_theta:+.4f} rad "
                        f"({math.degrees(sent_theta):+7.2f} deg) "
                        f"distance={sent_dist:.3f} m detected={int(detected)}"
                    )

            time.sleep(interval)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        uart.stop()
