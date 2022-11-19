import serial
import time
import enum
from typing import List
from dataclasses import dataclass


class SerialError(enum.IntEnum):
    NoError = enum.auto()
    WriteError = enum.auto()
    ReceiveSizeError = enum.auto()
    InvalidHeaderError = enum.auto()
    ChecksumError = enum.auto()
    OtherError = enum.auto()


class SerialDecodeType(enum.IntEnum):
    SignedInt16 = enum.auto()
    UnSignedInt16 = enum.auto()
    SignedInt32 = enum.auto()
    UnSignedInt32 = enum.auto()


@dataclass
class SerialCommand:
    command_header: List[int]
    return_size: int
    exist_checksum: bool


class SerialCommunicator():
    def __init__(self, device_name: str, baud: int = 115200, timeout: float = 1) -> None:
        self._device_name = device_name
        self._baudrate = baud
        self._timeout = timeout

        self._port = None
        self.open(device_name, baud, timeout)

    def open(self, device_name: str, baud: int, timeout: float) -> None:
        try:
            self._port = serial.Serial(
                port=device_name,
                baudrate=baud,
                timeout=timeout)
        except Exception as e:
            print(f"Serial port can't open {e}")

    def write(self, buf: bytes, flush: bool = False) -> int:
        return self._port.write(buf)

    def read(self, wait_sec: int, size: int = 0) -> bytes:
        time.sleep(wait_sec)
        return self._port.read(size)

    def readwrite(self):
        ...

    def calc_checksum(self, buf: List[int]) -> int:
        buf_sum = sum(buf)
        return buf_sum & 0xff

    def check_error(self, buf: List[int], command: SerialCommand) -> SerialError:
        if len(buf) != command.return_size:
            return SerialError.ReceiveSizeError

        for b, h in zip(buf, command.command_header):
            if b != h:
                return SerialError.InvalidHeaderError

        if command.exist_checksum:
            sc = self.calc_checksum(buf[:-1])
            if buf[-1] != sc:
                return SerialError.ChecksumError

        return SerialError.NoError

    def encode(self, buf: bytes):
        ...

    def decode(self, buf: bytes, bit_size: int, signed: bool, big: bool = True):

        if len(buf) < bit_size/8:
            raise ValueError("decode buffer error")

        dec = int.from_bytes(buf, byteorder="big" if big else "little")
        if not signed:
            return dec

        bit = bit_size
        if dec >> bit:
            raise ValueError("decode buffer error")

        return dec - (dec >> (bit - 1) << bit)
