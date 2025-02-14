import sys
sys.path.append("/home/kevinm/Documents/projects/drone/openpilot/cereal")
import socket
import termios
import capnp
MAX_BUFFER_SIZE = 10*1024*1024  # 10MB
import time

custom_capnp = capnp.load("/home/eduardo/projects/uav/teleop/client/cereal/custom.capnp")

stop = False
class CarControl:
  def __init__(self, roll, pitch, yaw, throttle):
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw
    self.throttle = throttle

carState = CarControl(0, 0, 0, 0)

def apply_controls():

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.connect(("127.0.0.1", 4069))
      s.send(bytes([2]))
      s.sendall(command.to_bytes_packed())

def control_loop():
  while not stop:
    apply_controls()
    time.sleep(0.1)


def setup_terminal():
  attributes = termios.tcgetattr(sys.stdin)
  attributes[3] &= ~(termios.ECHO | termios.ICANON)
  termios.tcsetattr(sys.stdin, termios.TCSANOW, attributes)


def main():
  setup_terminal()

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
      s.connect(("127.0.0.1", 4069))
      s.send(bytes([2]))

      command = custom_capnp.JoystickCommand.new_message(
        roll = carState.roll,
        pitch = carState.pitch,
        yaw = carState.yaw,
        throttle = carState.throttle,
      )
      s.sendall(command.to_bytes_packed())

if __name__ == "__main__":
  main()


