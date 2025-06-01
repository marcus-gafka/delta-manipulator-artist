import serial
import time

class SerialCommunicator:
    def __init__(self, port, baud_rate):
        print("SerialCommunicator initialized")
        self.ser = serial.Serial(port, baud_rate)
        time.sleep(2)
        self.ser.flush()

    def send_steps(self, steps1, steps2, interval):
        cmd = f"{steps1} {steps2} {interval}\n"
        self.ser.write(cmd.encode())

        while True:
            line = self.ser.readline().decode().strip()
            if line == "DONE":
                break
