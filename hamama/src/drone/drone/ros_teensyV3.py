import socket
import threading
from rclpy.executors import MultiThreadedExecutor
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion,Vector3
from std_msgs.msg import Int32MultiArray  
from sensor_msgs.msg import Joy




FLOAT_SIZE = struct.calcsize('f')
INT_SIZE = struct.calcsize('i')


class UDPServer(Node):
    def __init__(self, server_addr, client_addr):
        super().__init__('udp_server')
        
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
        self.mpu6050_msg = [0.0] * 6
        self.polulo_msg = [0.0] * 6
        self.euler_msg = [0.0] * 3
        self.quaternion_msg = [0.0] * 4
        self.fuse_imu_msg = [0.0] * 6
        self.rc_ch_msg = [0] * 16

        # ROS2 Publishers
        self.imu_pub_mpu6050 = self.create_publisher(Imu, 'mpu6050_imu', 10)
        self.imu_pub_pololu = self.create_publisher(Imu, 'pololu_imu', 10)
        self.imu_pub_fuse = self.create_publisher(Imu, 'fuse_imu', 10)
        self.quaternion_pub = self.create_publisher(Quaternion, 'quaternion', 10)
        self.euler_pub = self.create_publisher(Vector3, 'euler_angles', 10)
        self.rc_pub = self.create_publisher(Int32MultiArray, 'rc_channels', 10)  # New publisher for RC data


        # Start receiver thread
        threading.Thread(target=self.receiver).start()

    def receiver(self):
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

        while True:
            message, address = self.sock.recvfrom(1024)
            if message[:2] == self.PING:
                pong_response = bytearray(self.PONG)
                pong_response.append(message[2])
                self.sock.sendto(bytes(pong_response), address)
            else:
                self.casting(message)

    def casting(self, message):
        # Casting the messages before moving them to ROS topic 
        type = chr(message[0])
        message = message[1:]
        mssages_struct_float = struct.unpack("f" * (len(message) // FLOAT_SIZE), message)
        message_struct_int = struct.unpack("i" * (len(message) // INT_SIZE), message)
        
        match type:
            case 'm':
                self.mpu6050_msg = mssages_struct_float[:6]
                self.publish_imu(self.mpu6050_msg, self.imu_pub_mpu6050)
   
            case 'c':
                self.euler_msg = mssages_struct_float[:3]
                self.quaternion_msg = mssages_struct_float[3:7]
                self.fuse_imu_msg = mssages_struct_float[7:]
                self.publish_euler(self.euler_msg)
                self.publish_quaternion(self.quaternion_msg)
                self.publish_imu(self.fuse_imu_msg, self.imu_pub_fuse)
                
            case 'p':
                self.polulo_msg = mssages_struct_float[:6]
                self.publish_imu(self.polulo_msg, self.imu_pub_pololu)
                
            case 'r':
                self.rc_ch_msg = message_struct_int[:16]
                self.publish_rc(self.rc_ch_msg)


    def publish_imu(self, imu_data, publisher):
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = imu_data[0]
        imu_msg.linear_acceleration.y = imu_data[1]
        imu_msg.linear_acceleration.z = imu_data[2]
        imu_msg.angular_velocity.x = imu_data[3]
        imu_msg.angular_velocity.y = imu_data[4]
        imu_msg.angular_velocity.z = imu_data[5]
        publisher.publish(imu_msg)

    def publish_quaternion(self, quaternion_data):
        quat_msg = Quaternion()
        quat_msg.x = quaternion_data[0]
        quat_msg.y = quaternion_data[1]
        quat_msg.z = quaternion_data[2]
        quat_msg.w = quaternion_data[3]
        self.quaternion_pub.publish(quat_msg)

    def publish_euler(self, euler_data):
        euler_msg = Vector3()
        euler_msg.x = euler_data[0]
        euler_msg.y = euler_data[1]
        euler_msg.z = euler_data[2]
        self.euler_pub.publish(euler_msg)

    def publish_rc(self, rc_data):
        rc_msg = Int32MultiArray()
        rc_msg.data = rc_data  # Assign the list of 16 RC channel values
        self.rc_pub.publish(rc_msg)



class JoyListener(Node):
    def __init__(self, udp_server):
        super().__init__('joy_listener')
        self.udp_server = udp_server  # Store reference to UDPServer
        # Create a subscriber to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10  # QoS profile
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Listening to /joy topic...')

    def joy_callback(self, msg):
        # Process joystick data
        axes = msg.axes  # List of joystick axes values
        buttons = msg.buttons  # List of joystick button values

        # # Example of printing out joystick axes and button values
        # self.get_logger().info(f'Axes: {axes}')
        # self.get_logger().info(f'Buttons: {buttons}')

        axis_byte_array = b''.join([struct.pack('f', f) for f in axes])
        # print (axis_byte_array)
        self.udp_server.sock.sendto(axis_byte_array, self.udp_server.client_addr)
        


        
    #     # Optionally send joystick data via UDP
    #     self.send_joy_data(axes, buttons)

    # def send_joy_data(self, axes, buttons):
    #     data = struct.pack(f"{len(axes)}f{len(buttons)}i", *axes, *buttons)
    #     self.udp_server.sock.sendto(data, self.udp_server.client_addr)




def start_udp_server():
    rclpy.init()

    # Create and start the UDP server
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.168.1.199', 8888)
    server = UDPServer(server_addr, client_addr)

    # Create the JoyListener
    joy_listener = JoyListener(server)

    # Use MultiThreadedExecutor to manage both nodes
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(joy_listener)

    try:
        # Spin the executor
        executor.spin()
    finally:
        # Clean up on shutdown
        server.destroy_node()
        joy_listener.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    start_udp_server()