import sys
sys.path.append("/home/kevinm/Documents/projects/drone/openpilot/cereal")
import socket
import termios
import threading
import cv2
import capnp
import numpy as np
MAX_BUFFER_SIZE = 10*1024*1024  # 10MB
from collections import namedtuple
import time
from inputs import get_gamepad

log_capnp = capnp.load("./cereal/log.capnp")
car_capnp = capnp.load("./cereal/car.capnp")
custom_capnp = capnp.load("./cereal/custom.capnp")

stop = False
class CarControl:
  def __init__(self, roll, pitch, yaw, throttle):
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw
    self.throttle = throttle

carState = CarControl(0, 0, 0, 0)

def start_vehicle():
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
          s.connect(("127.0.0.1", 4069))
          s.send(bytes([3]))
          confirmed = s.recv(1)
          if len(confirmed) == 0:
            raise Exception("Failed to start vehicle")

          if confirmed[0] != 0:
            raise Exception("Failed to start vehicle")

def land_vehicle():
      with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
          s.connect(("127.0.0.1", 4069))
          s.send(bytes([4]))
          confirmed = s.recv(1)
          if len(confirmed) == 0:
            raise Exception("Failed to start vehicle")

          if confirmed[0] != 0:
            raise Exception("Failed to start vehicle")

def apply_controls():
  command = custom_capnp.JoystickCommand.new_message(
    roll = carState.roll,
    pitch = carState.pitch,
    yaw = carState.yaw,
    throttle = carState.throttle,
  )

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      print("sending control")
      s.connect(("127.0.0.1", 4069))
      s.send(bytes([2]))
      s.sendall(command.to_bytes_packed())

def control_loop():
  while not stop:
    apply_controls()
    time.sleep(0.1)

def read_keyboard():
  return sys.stdin.read(1)

def setup_terminal():
  attributes = termios.tcgetattr(sys.stdin)
  attributes[3] &= ~(termios.ECHO | termios.ICANON)
  termios.tcsetattr(sys.stdin, termios.TCSANOW, attributes)


def main():
  setup_terminal()
  command_thread = threading.Thread(target=control_loop)
  command_thread.start()
  while True:
    events = get_gamepad()
    for event in events:
      if event.code == "BTN_TL" and event.state == 1:
        start_vehicle()
      elif event.code == "BTN_TR" and event.state == 1:
        land_vehicle()
      elif event.code == "BTN_SOUTH" and event.state == 1:
        carState.yaw += 0.001
  command_thread.join()

if __name__ == "__main__":
  main()

