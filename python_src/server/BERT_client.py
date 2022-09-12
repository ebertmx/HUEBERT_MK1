""" A class to setup a local client for BERT to communicate with the server

TODO add security best practices into server connection 
    1. OpenSSL to encrypt server-client connection
    2. Whitelist BERT's ip address so it is the only allowed connection
    3. Implement RSA key requirement so only a user with the correct private key
       can connect. 
"""

# Libraries
import socket

# --------------------------------------------------------------------------------------


class BertClient:
    """Connect, Disconnect and communicate with the raspberry pi server"""

    def __init__(self, host="192.168.1.147", port=1234, retry_attempts=10):
        self.host = host
        self.port = port
        self.retry_attempts = retry_attempts
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        """Attempt connecting to the server retry_attempts number of times

        returns:
            - Bool: True if connection successful
        """
        for attempt in range(self.retry_attempts):
            try:
                self.socket.connect((self.host, self.port))
                return True
            except Exception as error:
                print(error)
        return False

    def disconnect(self):
        """Disconnect from the server"""
        self.socket.close()

    def send_data_to_server(self, control_data):
        """Send control data to the server. If it fails, try to reconnect and resend"""
        try:
            self.socket.send(control_data)
        except Exception as error:
            print(error)
            self.socket.connect()
            self.socket.send(control_data)

    def receive_data_from_server(self, num_bytes=20):
        """Receive data from the server. If it fails, try to reconnect and resend"""
        try:
            return self.socket.recv(num_bytes)
        except Exception as error:
            print(error)
            self.socket.connect()
            return self.socket.recv(num_bytes)

    def print_server_data(self, data):
        """Print data coming from the server"""
        print(data.decode("utf-8"))
