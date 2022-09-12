import serial

# Global variables
arduino = "/dev/ttyACM0"
baud_rate = 9600
timeout = 1

# Initialize serial object
serial = serial.Serial(arduino, baud_rate, timeout=timeout)


def test_serial():
    serial.flush()

    while 1:
        line = serial.readline().decode("utf-8").rstrip()
        print(line)


if __name__ == "__main__":
    test_serial()
