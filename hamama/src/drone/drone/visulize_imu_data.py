import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import Imu
import numpy as np

class IMUListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            '/pololu_imu',  # Change to your actual topic name if different
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        
        # Setup the 3D plot for acceleration and angular velocity
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Draw the world axes
        self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
        self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.ax.set_zlim([-5, 5])
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

        plt.ion()  # Interactive mode for live updating
        plt.show()

    def listener_callback(self, msg):
        # Extract linear acceleration data (m/s^2)
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # Extract angular velocity data (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # Clear the current plot
        self.ax.cla()

        # Redraw the world axes
        self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
        self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

        # Draw the linear acceleration vector in red
        self.ax.quiver(0, 0, 0, accel_x, accel_y, accel_z, color='r', label='Accel')

        # Draw the angular velocity vector in green
        self.ax.quiver(0, 0, 0, gyro_x, gyro_y, gyro_z, color='g', label='Gyro')

        # Set limits and labels for the plot
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.ax.set_zlim([-5, 5])
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

        # Add legend for the vectors
        self.ax.legend()

        # Redraw the plot
        plt.draw()
        plt.pause(0.01)  # Pause to allow for live updating

def main(args=None):
    rclpy.init(args=args)
    imu_listener = IMUListener()
    rclpy.spin(imu_listener)

    # Shutdown after closing
    imu_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
