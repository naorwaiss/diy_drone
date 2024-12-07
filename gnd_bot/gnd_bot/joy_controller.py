import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from sympy.codegen.cnodes import sizeof
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


        self.publisher_ = self.create_publisher(Float32MultiArray, 'state_machine', 10)

        self.axis_joy = [0.0] * 8  # Assuming the joystick has 8 axes
        self.combine_axis = [0.0] * 4  # Adjust the size as needed
        self.sensitivity = 0.1

        self.IP= "192.168.1.177"
        self.PORT = 8888
        self.address = (self.IP,self.PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)





    def joy_callback(self, msg):
        if len(msg.axes) >= 8:
            self.axis_joy[0] = msg.axes[0]  # pitch
            self.axis_joy[1] = msg.axes[1]  # roll
            self.axis_joy[2] = msg.axes[2]  # thr
            self.axis_joy[3] = msg.axes[3]  # yaw
            self.axis_joy[4] = msg.axes[4]  # arm+mode + pit mode
            self.axis_joy[5] = msg.axes[5]  # flip
            self.axis_joy[6] = msg.axes[6]  # open capsule
            self.axis_joy[7] = msg.axes[7]  # free
        self.combine_callback()




    def publish_combined_state(self):
        combined_state = self.axis_joy 
        state_message = Float32MultiArray()
        state_message.data = combined_state
        self.publisher_.publish(state_message)
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



