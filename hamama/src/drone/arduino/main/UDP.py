import socket
import threading
import struct

FLOAT_SIZE = struct.calcsize('f')
INT_SIZE = struct.calcsize('i')

class UDPServer:
    def __init__(self, server_addr, client_addr):
        # Initialize server addresses
        self.server_addr = server_addr
        self.client_addr = client_addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Messages
        self.SYN = bytes([0x01, 0x01])
        self.SYNACK = bytes([0x01, 0x02])
        self.ACK = bytes([0x01, 0x03])
        self.PING = bytes([0x01, 0x10])
        self.PONG = bytes([0x01, 0x11])
        self.TERMINATE = bytes([0x01, 0xFF])

        # Data storage for IMUs and other data
        self.polulo_msg = [0.0] * 6
        self.euler_msg = [0.0] * 3
        self.mag_msg = [0.0] * 3 
        self.quaternion_msg = [0.0] * 4
        self.rc_ch_msg = [0] * 16

        # Start receiver thread
        threading.Thread(target=self.receiver).start()

    def receiver(self):
        self.sock.bind(self.server_addr)
        print("Server Started")

        # Set a timeout for handshake to avoid infinite loop
        self.sock.settimeout(5)  # 5 seconds timeout for receiving
        try:
            # Initiate SYN-ACK handshake
            self.sock.sendto(self.SYN, self.client_addr)
            retries = 0
            message, address = self.sock.recvfrom(1024)
            while message != self.SYNACK and retries < 5:
                message, address = self.sock.recvfrom(1024)
                retries += 1

            if message != self.SYNACK:
                print("FAILED SYN ACK after retries")
                return

            self.sock.sendto(self.ACK, self.client_addr)
            print("Created a connection")

        except socket.timeout:
            print("Connection timed out during handshake.")
            return

        self.sock.settimeout(None)  # Remove timeout after successful connection

        while True:
            message, address = self.sock.recvfrom(1024)
            if message[:2] == self.PING:
                pong_response = bytearray(self.PONG)
                pong_response.append(message[2])
                self.sock.sendto(bytes(pong_response), address)
            else:
                self.casting(message)

    def casting(self, message):
        # Casting the messages based on the type
        type = chr(message[0])
        message = message[1:]
        messages_struct_float = struct.unpack("f" * (len(message) // FLOAT_SIZE), message)
        message_struct_int = struct.unpack("i" * (len(message) // INT_SIZE), message)

        match type:
            case 'm':  # Magnetometer data
                self.mag_msg = messages_struct_float[:3]
                print(f"Magnetometer Data: {self.mag_msg}")

            case 'q':  # Quaternion data
                self.quaternion_msg = messages_struct_float[:4]
                print(f"Quaternion Data: {self.quaternion_msg}")

            case 'e':  # Euler angles data
                self.euler_msg = messages_struct_float[:3]
                print(f"Euler Angles Data: {self.euler_msg}")

            case 'p':  # Pololu IMU data
                self.polulo_msg = messages_struct_float[:6]
                print(f"Pololu IMU Data: {self.polulo_msg}")

            case 'r':  # RC channel data
                self.rc_ch_msg = message_struct_int[:16]
                print(f"RC Channel Data: {self.rc_ch_msg}")


def start_udp_server():
    # Create and start the UDP server
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.168.1.199', 8888)
    server = UDPServer(server_addr, client_addr)

if __name__ == "__main__":
    start_udp_server()
