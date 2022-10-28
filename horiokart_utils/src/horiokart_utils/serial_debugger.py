import serial
import argparse
import time
import re


parser = argparse.ArgumentParser()
parser.add_argument("--port", default="/dev/ttyACM0")
parser.add_argument("--baud", default=115200, type=int)
parser.add_argument("--timeout", default=3, type=int)
args = parser.parse_args()


port = args.port
baud = args.baud
timeout = args.timeout

print(f"port:{port}")

ser = serial.Serial(port, baud, timeout=timeout)

def readwrite(send_buf):
    print(f"write:{''.join([f'0x{x:02x} ' for x in send_buf])}")
    print(f"raw:{send_buf}")
    print("---")

    ser.write(send_buf)
    line = ser.readline()
    print(f"read:{''.join([f'0x{x:02x} ' for x in line])}")
    print(f"raw:{line}")
    print("---")

h = b""
while True:
    print()
    s = input("Wait list like [0x01, 0xff] or 'r' to repeat...:")

    if s == 'q':
        break
    if s == 'r':
        readwrite(h)
        continue

    m = re.match(r"\[(.*)\]", s)
    if m is not None:
        send_buf = bytes(map(lambda x:int(x, 16), m.groups()[0].split(",")))
        readwrite(send_buf)

        h = send_buf
        continue

    print(f"not match:{s}")

    time.sleep(0.1)
