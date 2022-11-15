from serial_communicator import SerialCommunicator, SerialError, SerialCommand
from dataclasses import dataclass
from typing import List, Any


@dataclass
class OdometryData:
    x: float
    y: float
    th: float
    vx: float
    vy: float
    vth: float

    raw: bytes
    error: SerialError


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

        odom.x = self._comm.decode(ret_buf[2:6], byte_size=32, signed=True)
        odom.y = self._comm.decode(ret_buf[6:10], byte_size=32, signed=True)
        odom.th = self._comm.decode(ret_buf[10:12], byte_size=16, signed=False)
        odom.vx = self._comm.decode(ret_buf[12:14], byte_size=16, signed=True)
        odom.vy = self._comm.decode(ret_buf[14:16], byte_size=16, signed=True)
        odom.vth = self._comm.decode(ret_buf[16:18], byte_size=16, signed=True)

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
