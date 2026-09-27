import socket


HOST = '0.0.0.0' # Allows connections from any IP
PORT = 5500      # Expected port from embedded software


class ReceiverUDP:

    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((HOST, PORT))

    def receive_udp(self):
        data, addr = self.socket.recvfrom(4096)
        return data, addr

    def shutdown_udp(self):
        self.socket.close()

