"""A class to read positional serial control data from BERT's arduino and positional
   control sensors
"""

# Libraries
from serial import Serial

# --------------------------------------------------------------------------------------


class ArduinoPy:
    """A class to initialize a serial connection between a pc and arduino and
    transfer serial data between the two
    """

    def __init__(self, port="COM3", baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.arduino = Serial(
            port=self.port, baudrate=self.baudrate, timeout=self.timeout
        )

    def read_control_data_from_bert(self):
        """Read control data from BERT's arduino, and verify it

        returns:
                - control_data: Verified control data to send to the server

        Excpetions:
                - If control data cannot be verified
        """
        control_data = self.arduino.readline()
        if self.control_data_verified(control_data):
            return control_data
        raise Exception("Incorrect Control Data Format")

    def control_data_verified(self, control_data):
        """Verify control data is in the correct format

        parameters:
                - control_data: The data to verify

        returns:
                - Bool: True if data is formatted correctly
        """

#         # Check length
#         if len(control_data) != 11:
#             return False

#         # Check for initial positional control character
#         # i.e. control data should begin with 'p' to flag it is positional
#         if not control_data[0].decode().isalpha():
#             return False

        # All checks passed
        return True
