import serial
import argparse
import time
import re


parser = argparse.ArgumentParser()
parser.add_argument("--port", default="/dev/ttyHoriokart-odom")
parser.add_argument("--baud", default=115200, type=int)
parser.add_argument("--timeout", default=3, type=float)
args = parser.parse_args()


port = args.port
baud = args.baud
timeout = args.timeout

send_buf = [0x24, 0x75]

print(f"port:{port}")

ser = serial.Serial(port, baud, timeout=timeout)

def readwrite(send_buf):
    #print(f"write:{''.join([f'0x{x:02x} ' for x in send_buf])}")
    #print(f"raw:{send_buf}")
    #print("---")

    ser.flushInput()
    ser.flushOutput()
    ser.write(send_buf)

    time.sleep(0.01)

    #line = ser.readline()
    line = ser.read(19)
    #print(f"read:{''.join([f'0x{x:02x} ' for x in line])}")
    #print(f"raw:{line}")
    #print("---")

    if len(line) != 19:
        print(f"----- ERROR: 1 -----")
    elif line[0] != 0x24 or line[1] != 0x75:
        print(f"----- ERROR: 2 -----")


while True:
    s = time.time()
    for _ in range(10):
        readwrite(send_buf)

        time.sleep(0.05)
    e = time.time()
    print(f"time:{e-s}sec, {10/(e-s)}hz")
