import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sympy.codegen.cnodes import sizeof
# import math
import socket 
import numpy as np



class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_mechine')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )



        self.axis_joy = [0.0] * 8  # Assuming the joystick has 8 axes
        self.buttons_joy = [0.0]*8
        self.IP= "192.168.1.177"
        self.PORT = 8888
        self.address = (self.IP,self.PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)







    def joy_callback(self, msg):
        if len(msg.axes) >= 8:
           for i in range(8):
                self.axis_joy[i] = msg.axes[i]
                self.buttons_joy[i] = msg.buttons[i]




        #print(self.combine_axis)

    def publish_combined_state(self):
        combined_state = self.axis_joy +self.buttons_joy
        print(combined_state)
        state_message = Float32MultiArray()
        state_message.data = combined_state
        np_array = np.array(combined_state, dtype=np.float32)
        bytes_data = np_array.tobytes()
        self.sock.sendto(bytes_data, self.address)  # Send the encoded message to the server



    def connect_msg(self) -> None:
        try:
            message = "5"  # Define the message to be sent
            self.sock.sendto(message.encode(), self.address)  # Send the encoded message to the server
        except Exception as e:
            self.get_logger().error(f"Failed to send connection message: {e}")






def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()

    try:
        node.connect_msg()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


