

import can

global bus
bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=117000)

import signal
import time
import sys

def handle_sigint(signum, frame):
    global bus
    bus.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, handle_sigint)

dic = {
    1:"0100",
    2:"1000",
    3:"0000",
    4:"0001",
    5:"0010",
    6:"0000",
    7:"0000",
    8:"1111"}


while True:
  msg = bus.recv()
  if msg is not None:
      buffer = msg.data
      value = 0
      if buffer[0] in dic.keys():
        print(dic[buffer[0]])
      elif len(buffer) == 5:
        value = (buffer[1] << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4]
        print(hex(value))
      else:
        print(f"ID: {hex(msg.arbitration_id)}, Data: {str(msg.data)}")



