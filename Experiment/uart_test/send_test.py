"""
UART送信テスト用の最小構成コード。
send_image.pyからカメラ・ボール検出要素を取り除き、
既知のテストパターンを送り続けるだけのプログラム。
C++側のuart_receive_test.cppと組み合わせて、
送った値と受信した値が一致するかを目視で確認する。

実行:
  python3 uart_send_test.py                  # デフォルトポート(/dev/serial0)を使用
  python3 uart_send_test.py /dev/ttyUSB0      # ポートを指定する場合
  Ctrl+Cで終了。
"""

import sys
import time
import struct
import logging
from typing import Optional
from dataclasses import dataclass

import serial

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class UARTConfig:
    port: str = "/dev/serial0"  # LiDARラズパイへのUART送信ポート
    baudrate: int = 9600
    write_timeout: float = 1.0  # 秒


class UARTSender:
    """
    赤ボールの (angle, distance, detected) をフレーム化してUART送信する。
    フレーム構成: [開始バイト 0xAA][angle: float32][distance: float32][detected: uint8][checksum: uint8]
    checksumはペイロード(angle+distance+detected, 9byte)のXOR。
    """

    START_BYTE = 0xAA

    def __init__(self, config: UARTConfig) -> None:
        self.cfg = config
        self.ser: Optional[serial.Serial] = None

    def start(self) -> None:
        self.ser = serial.Serial(
            port=self.cfg.port,
            baudrate=self.cfg.baudrate,
            write_timeout=self.cfg.write_timeout,
        )
        logger.debug(f"UART opened: {self.cfg.port} @ {self.cfg.baudrate}bps")

    def send(self, angle: float, distance: float, detected: bool) -> None:
        if self.ser is None:
            raise RuntimeError("UARTSender.start() を先に呼んでください。")

        payload = struct.pack(
            "<ffB", float(angle), float(distance), 1 if detected else 0
        )
        checksum = self._calc_checksum(payload)
        frame = bytes([self.START_BYTE]) + payload + bytes([checksum])

        self.ser.write(frame)

    @staticmethod
    def _calc_checksum(payload: bytes) -> int:
        checksum = 0
        for b in payload:
            checksum ^= b
        return checksum

    def stop(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None


if __name__ == "__main__":
    uart_cfg = UARTConfig()
    if len(sys.argv) >= 2:
        uart_cfg = UARTConfig(port=sys.argv[1])

    uart = UARTSender(config=uart_cfg)
    uart.start()
    logger.debug("UART test sender started")

    # テストパターン:
    # ・angleを-180〜180度で0.2秒ごとにスイープ、distanceは固定値
    # ・10回に1回はdetected=Falseを送り、ロスト時の挙動も検証できるようにする
    angle_deg = -180.0
    step_count = 0

    try:
        while True:
            step_count += 1
            detected = step_count % 10 != 0

            if detected:
                angle_deg += 5.0
                if angle_deg > 180.0:
                    angle_deg = -180.0
                angle_send = angle_deg
                distance_send = 500.0
            else:
                angle_send = 0.0
                distance_send = -1.0

            uart.send(angle=angle_send, distance=distance_send, detected=detected)
            logger.debug(
                f"sent: angle={angle_send:+.2f} distance={distance_send:+.2f} "
                f"detected={int(detected)}"
            )

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        uart.stop()
