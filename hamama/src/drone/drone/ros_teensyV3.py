import socket
import threading
import struct

FLOAT_SIZE = struct.calcsize('f')


class UDPServer:
    def __init__(self, server_addr, client_addr):
        self.server_addr = server_addr
        self.client_addr = client_addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.SYN = bytes([0x01, 0x01])
        self.SYNACK = bytes([0x01, 0x02])
        self.ACK = bytes([0x01, 0x03])
        self.PING = bytes([0x01, 0x10])
        self.PONG = bytes([0x01, 0x11])
        self.TERMINATE = bytes([0x01, 0xFF])


        self.mpu6050_msg = []*6
        self.polulo_msg = []*6
        self.fuse_imu = []*6
        self.qurtenion = []*4
        


    def receiver(self):
        while True:
            message, address = self.sock.recvfrom(1024)
            if message[:2] == self.PING:
                pong_response = bytearray(self.PONG)
                pong_response.append(message[2])
                self.sock.sendto(bytes(pong_response), address)
            else:
                self.casting(message)

    def start_server(self):
        self.sock.bind(self.server_addr)
        print("Server Started")

        # Initiate SYN-ACK handshake
        self.sock.sendto(self.SYN, self.client_addr)
        message, address = self.sock.recvfrom(1024)
        while message != self.SYNACK:
            message, address = self.sock.recvfrom(1024)

        if message != self.SYNACK:
            print("FAILED SYN ACK")
            return
        self.sock.sendto(self.ACK, self.client_addr)
        print("Created a connection")

        # Start receiver thread
        threading.Thread(target=self.receiver).start()

        while True:
            self.sock.sendto(input("Send $> ").encode(), self.client_addr)

    def casting(self, message):
        type = chr(message[0])
        message = message[1:]
        mssages_struct = struct.unpack("f" * (len(message) // FLOAT_SIZE), message)
        match type:
            case 'm':
                print(f"from mpu6050 {mssages_struct[5]}")


            case 'c':
                print(f"from combine {mssages_struct[12]}")


        return
        float_data = struct.unpack('ffff', message[1:])
        print(float_data)
        return None
        message = message.decode()  # Decode the message as a string
        print(f"Received message: {message}")
        
        if message[0] == 'c':  # Check if the message starts with 'c'
            try:
                c_message = float(message[1:])  # Extract the float part after 'c'
                print(f"New data: {c_message}")
            except ValueError:
                print("Invalid float format in message")
        else:
            print(f"Message does not start with 'c': {message}")


def start_udp_server():
    # Create and start the UDP server
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.168.1.199', 8888)
    server = UDPServer(server_addr, client_addr)

    threading.Thread(target=server.start_server).start()


if __name__ == "__main__":
    start_udp_server()
