import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Quaternion
import numpy as np

def quaternion_to_rotation_matrix(q):
    # Convert a quaternion into a 3x3 rotation matrix
    q_w, q_x, q_y, q_z = q.w, q.x, q.y, q.z
    R = np.array([[1 - 2*q_y**2 - 2*q_z**2, 2*q_x*q_y - 2*q_z*q_w, 2*q_x*q_z + 2*q_y*q_w],
                  [2*q_x*q_y + 2*q_z*q_w, 1 - 2*q_x**2 - 2*q_z**2, 2*q_y*q_z - 2*q_x*q_w],
                  [2*q_x*q_z - 2*q_y*q_w, 2*q_y*q_z + 2*q_x*q_w, 1 - 2*q_x**2 - 2*q_y**2]])
    return R

class QuaternionListener(Node):
    def __init__(self):
        super().__init__('quaternion_listener')
        self.subscription = self.create_subscription(
            Quaternion,
            '/quaternion',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        
        # Setup the 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Draw the world axes
        self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
        self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

        plt.ion()  # Interactive mode for live updating
        plt.show()

    def listener_callback(self, msg):
        # Convert quaternion to rotation matrix
        R = quaternion_to_rotation_matrix(msg)

        # Define object axis in the local frame
        object_axes = np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0, 0, 1]])

        # Rotate object axis using rotation matrix
        transformed_axes = R.dot(object_axes)

        # Clear the current plot
        self.ax.cla()

        # Redraw the world axes
        self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
        self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

        # Draw the transformed object axes
        self.ax.quiver(0, 0, 0, transformed_axes[0, 0], transformed_axes[0, 1], transformed_axes[0, 2], color='r')
        self.ax.quiver(0, 0, 0, transformed_axes[1, 0], transformed_axes[1, 1], transformed_axes[1, 2], color='g')
        self.ax.quiver(0, 0, 0, transformed_axes[2, 0], transformed_axes[2, 1], transformed_axes[2, 2], color='b')

        # Set limits and labels again
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

        # Redraw the plot
        plt.draw()
        plt.pause(0.01)  # Pause to allow for live updating

def main(args=None):
    rclpy.init(args=args)
    quaternion_listener = QuaternionListener()
    rclpy.spin(quaternion_listener)

    # Shutdown after closing
    quaternion_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

