import threading
import time
import serial


class ArduinoThread(threading.Thread):
    """Main class for controlling Arduino."""

    def __init__(self, xval, radius, colorval):
        threading.Thread.__init__(self)
        self.arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=0.1)
        self.isRunning = True
        self.colorval = colorval
        self.xval = xval
        self.radius = radius

    def run(self):
        """This method instructs boat to follow red/blue ball,
        turn lef/right, or keep going straight.

        If boat position is between radius 20 to 70 = follow ball.
        If boat position is under radius 20 = keep going straight.
        If boat position is over 70 radius = turn left/right.
        """
        if self.colorval == "Red":
            if 20 < self.radius < 70:
                self.arduino.write(str(self.xval).encode())
            elif self.radius <= 20:
                self.arduino.write(str("100").encode())
            elif self.radius >= 70:
                self.arduino.write("10".encode())
        else:
            if 20 < self.radius < 70:
                self.arduino.write(str(self.xval).encode())
            elif self.radius <= 20:
                self.arduino.write(str("100").encode())
            elif self.radius >= 70:
                self.arduino.write("10".encode())

        time.sleep(0.1)
