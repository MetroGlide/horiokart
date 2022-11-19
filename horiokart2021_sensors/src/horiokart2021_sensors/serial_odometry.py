from serial_communicator import SerialCommunicator, SerialError, SerialCommand
from dataclasses import dataclass
from typing import List, Any


@dataclass
class OdometryData:
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vth: float = 0.0

    raw: bytes = bytes([])
    error: SerialError = SerialError.NoError


@dataclass(frozen=True)
class OdomCommand:
    zero_reset = SerialCommand(
        command_header=bytes([0x24, 0x73]),
        return_size=3,
        exist_checksum=False
    )
    get_odom = SerialCommand(
        command_header=bytes([0x24, 0x75]),
        return_size=19,
        exist_checksum=True
    )


class SerialOdometry:
    def __init__(self, communicator: SerialCommunicator) -> None:
        self._comm = communicator
        self._sleep_sec = 0.05

    def set_sleep_sec(self, sec: float):
        self._sleep_sec = sec

    def get_odom(self) -> OdometryData:

        # write
        ret_byte_num = self._comm.write(OdomCommand.get_odom.command_header)

        if ret_byte_num < len(OdomCommand.get_odom.command_header):
            odom = OdometryData()
            odom.error = SerialError.WriteError
            return odom

        # read
        ret_buf = self._comm.read(
            wait_sec=self._sleep_sec,
            size=OdomCommand.get_odom.return_size
        )

        # check error
        e = self._comm.check_error(
            buf=ret_buf,
            header=OdomCommand.get_odom.command_header,
            size=OdomCommand.get_odom.return_size
        )
        if e != SerialError.NoError:
            odom = OdometryData()
            odom.raw = ret_buf
            odom.error = e
            return odom

        # decode data
        odom = self.decode(ret_buf=ret_buf)
        odom.error = e

        return odom

    def zero_reset(self, retry: int) -> SerialError:
        for _ in range(retry):
            # -- あとで共通化する --
            # write
            ret_byte_num = self._comm.write(
                OdomCommand.zero_reset.command_header)

            if ret_byte_num < len(OdomCommand.zero_reset.command_header):
                continue

            # read
            ret_buf = self._comm.read(
                wait_sec=self._sleep_sec,
                size=OdomCommand.zero_reset.return_size
            )

            # check error
            e = self._comm.check_error(
                buf=ret_buf,
                command=OdomCommand.zero_reset
            )
            if e != SerialError.NoError:
                continue

            return SerialError.NoError

        return SerialError.OtherError

    def decode(self, ret_buf: bytes) -> OdometryData:
        odom = OdometryData()

        odom.x = self._comm.decode(ret_buf[2:6], bit_size=32, signed=True)
        odom.y = self._comm.decode(ret_buf[6:10], bit_size=32, signed=True)
        odom.th = self._comm.decode(ret_buf[10:12], bit_size=16, signed=False)
        odom.vx = self._comm.decode(ret_buf[12:14], bit_size=16, signed=True)
        odom.vy = self._comm.decode(ret_buf[14:16], bit_size=16, signed=True)
        odom.vth = self._comm.decode(ret_buf[16:18], bit_size=16, signed=True)

        odom.x = odom.x / 1000  # mm -> m
        odom.y = odom.y / 1000  # mm -> m
        odom.th = odom.th / 10000  # (1/10000)rad -> rad
        odom.vx = odom.vx / 1000  # mm/sec -> m/sec
        odom.vy = odom.vy / 1000  # mm/sec -> m/sec
        odom.vth = odom.vth / 10000  # (1/10000)rad/sec -> rad/sec

        odom.raw = ret_buf

        return odom

    def is_alive(self):
        ...


if __name__ == "__main__":
    import pprint

    serial = SerialCommunicator("")
    odom = SerialOdometry(communicator=serial)

    # x:100.5m, y:200.6m th:1.5rad vx:20.3m/s vy:14.7m/s vth:2.1rad/s
    buf = bytes([
        0x24, 0x75,
        0x00, 0x01, 0x88, 0x94,
        0x00, 0x03, 0x0F, 0x98,
        0x3A, 0x98,
        0x4F, 0x4C,
        0x39, 0x6C,
        0x52, 0x08,
        0x00
    ])
    o = odom.decode(buf)
    pprint.pprint(o)

    # x:-100.5m, y:-200.6m th:-1.5rad vx:-20.3m/s vy:-14.7m/s vth:-2.1rad/s
    buf = bytes([
        0x24, 0x75,
        0xFF, 0xFE, 0x77, 0x6C,
        0xFF, 0xFC, 0xF0, 0x68,
        0x3A, 0x98,
        0xB0, 0xB4,
        0xC6, 0x94,
        0xAD, 0xF8,
        0x00
    ])
    o = odom.decode(buf)
    pprint.pprint(o)
