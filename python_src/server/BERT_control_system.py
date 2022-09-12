""" BERT's control system
"""

# Libraries
from BERT_client import BertClient
from arduino_serial import ArduinoPy

# --------------------------------------------------------------------------------------

# Initialize bert client to communicate with server
bert = BertClient()

# Initialize ardipy to read ad verify serial positional control data from BERT
ardipy = ArduinoPy()


def activate_bert():
    """Activate video and data transmission between BERT and server"""

    # Connect to server
    if bert.connect():
        print("Successfully connected to server!")
    else:
        raise Exception("Fatal Error: Could not establish connection with server.")

    # Read control data, if verified - send to server
    while True:
        try:
            control_data = ardipy.read_control_data_from_bert()
            bert.send_data_to_server(control_data)
        except Exception as error:
            print(error)


if __name__ == "__main__":
    activate_bert()
