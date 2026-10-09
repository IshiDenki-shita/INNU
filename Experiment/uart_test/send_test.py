"""
UART送信テスト用の最小構成コード。
通信仕様書「通信の設計」(2026/10/09)のフレーム形式に従い、
既知のテストパターンを送り続けるだけのプログラム。
C++側のreceive_test.cppと組み合わせて、
送った値と受信した値が一致するか、不正フレームが破棄されるかを確認する。

フレーム(15byte固定、リトルエンディアン):
  [0]AA [1]55 [2]version=01 [3]sequence [4-7]theta_rad f32 [8-11]distance_m f32
  [12]flags(bit0=detected) [13-14]CRC-16/CCITT-FALSE(byte2〜12が対象、下位byte先)

実行:
  python3 send_test.py                       # /dev/serial0、10Hz
  python3 send_test.py /dev/ttyUSB0 --hz 20  # ポートと送信頻度(最大30Hz)を指定
  python3 send_test.py --fault               # 20フレームごとに不正フレームを混ぜる
  Ctrl+Cで終了。
"""

import argparse
import math
import struct
import time
from typing import Optional

import serial

from Experiment.mods.logger import logger

# float32に丸めたπ。受信側もfloat32同士で範囲判定するので、送信側も揃えて判定する。
PI_F32 = struct.unpack("<f", struct.pack("<f", math.pi))[0]


class UARTSender:
    """
    ボールの (theta_rad, distance_m, detected) をフレーム化してUART送信する。
    フレーム構成は通信仕様書3章に従う(15byte固定)。
    """

    SOF0 = 0xAA
    SOF1 = 0x55
    VERSION = 0x01
    FLAG_DETECTED = 0x01
    FRAME_SIZE = 15

    def __init__(
        self,
        port: str = "/dev/serial0",  # LiDARラズパイへのUART送信ポート
        baudrate: int = 9600,
        write_timeout: float = 1.0,
        max_send_hz: float = 30.0,
    ) -> None:

        self.ser: Optional[serial.Serial] = None
        self.sequence = 0  # 次に送るsequence番号
        self._last_send_time: Optional[float] = None

        self.port: str = port
        self.baudrate: int = baudrate
        self.write_timeout: float = write_timeout
        self.max_send_hz: float = max_send_hz

        # CRC実装の自己確認(CRC-16/CCITT-FALSEの標準チェック値)
        if self.crc16(b"123456789") != 0x29B1:
            raise RuntimeError("CRC-16/CCITT-FALSEの実装が仕様と一致しません。")

    def start(self) -> None:
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            write_timeout=self.write_timeout,
        )
        logger.debug(f"UART opened: {self.port} @ {self.baudrate}bps 8N1")

    def send(self, theta_rad: float, distance_m: float, detected: bool) -> bool:
        """
        フレームを送信する。送信したらTrue。
        max_send_hzを超える頻度で呼ばれた場合は送信せずFalseを返す。
        detected=Falseのときは仕様どおり theta=0.0, distance=0.0 を送る。
        """
        if self.ser is None:
            raise RuntimeError("UARTSender.start() を先に呼んでください。")

        now = time.monotonic()
        if (
            self._last_send_time is not None
            and now - self._last_send_time < 1.0 / self.max_send_hz
        ):
            return False

        if not detected:
            theta_rad, distance_m = 0.0, 0.0
        else:
            self._validate(theta_rad, distance_m)

        self.ser.write(self.build_frame(theta_rad, distance_m, detected))
        self._last_send_time = now
        self.advance_sequence()
        return True

    def build_frame(
        self,
        theta_rad: float,
        distance_m: float,
        detected: bool,
        version: Optional[int] = None,
    ) -> bytes:
        """検証なしでフレームを組み立てる(不正フレームのテスト送信にも使う)。"""
        if version is None:
            version = self.VERSION
        flags = self.FLAG_DETECTED if detected else 0

        # byte 2〜12(11byte): version, sequence, theta, distance, flags
        body = struct.pack(
            "<BBffB", version, self.sequence, theta_rad, distance_m, flags
        )
        crc = self.crc16(body)
        return bytes([self.SOF0, self.SOF1]) + body + struct.pack("<H", crc)

    def send_raw(self, frame: bytes) -> None:
        if self.ser is None:
            raise RuntimeError("UARTSender.start() を先に呼んでください。")
        self.ser.write(frame)

    def advance_sequence(self) -> None:
        self.sequence = (self.sequence + 1) & 0xFF

    @staticmethod
    def _validate(theta_rad: float, distance_m: float) -> None:
        theta32 = struct.unpack("<f", struct.pack("<f", theta_rad))[0]
        if not math.isfinite(theta32) or not (-PI_F32 < theta32 <= PI_F32):
            raise ValueError(f"theta_radが範囲外です(-π < θ ≤ π): {theta_rad}")
        if not math.isfinite(distance_m) or distance_m < 0.0:
            raise ValueError(f"distance_mは0以上の有限値にしてください: {distance_m}")

    @staticmethod
    def crc16(data: bytes) -> int:
        """CRC-16/CCITT-FALSE: 多項式0x1021, 初期値0xFFFF, 反転なし, XOR出力0x0000"""
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

    def stop(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None


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
    parser.add_argument("port", nargs="?", default=uart.port)
    parser.add_argument("--hz", type=float, default=10.0, help="送信頻度(最大30)")
    parser.add_argument("--fault", action="store_true", help="不正フレームも送る")
    args = parser.parse_args()
    if not (0.0 < args.hz <= uart.max_send_hz):
        parser.error(f"--hzは0より大きく{uart.max_send_hz}以下にしてください")
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
