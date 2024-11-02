import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class PointCloudToLaserScan(Node):

    def __init__(self):
        super().__init__('pointcloud_to_laserscan')

        # Subscribe to the point cloud topic
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10)

        # Publisher for the laser scan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Set the number of laser scan points
        self.num_ranges = 360

    def pointcloud_callback(self, pointcloud_msg):
        # Initialize LaserScan message
        laser_scan = LaserScan()
        laser_scan.header = pointcloud_msg.header
        laser_scan.angle_min = -np.pi / 2  # 180-degree FOV
        laser_scan.angle_max = np.pi / 2
        laser_scan.angle_increment = (laser_scan.angle_max - laser_scan.angle_min) / self.num_ranges
        laser_scan.range_min = 0.05  # Minimum range in meters
        laser_scan.range_max = 20.0  # Maximum range in meters

        # Initialize ranges array with inf values
        ranges = [float('inf')] * self.num_ranges

        # Extract points from PointCloud2 message
        points = point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True)
        
        for point in points:
            x, y, z = point
            distance = np.sqrt(x**2 + y**2)  # Calculate 2D distance

            # Debugging: print point data
            self.get_logger().info(f'Point: x={x}, y={y}, z={z}, distance={distance}')
            
            if laser_scan.range_min < distance < laser_scan.range_max and x > 0:
                angle = np.arctan2(y, x)  # Calculate angle of the point
                # Check if the angle is within the valid range
                if laser_scan.angle_min <= angle <= laser_scan.angle_max:
                    # Convert angle to an index in the ranges array
                    index = int((angle - laser_scan.angle_min) / laser_scan.angle_increment)
                    # Update the range value if the new distance is closer
                    if ranges[index] == float('inf') or distance < ranges[index]:
                        ranges[index] = distance
                else:
                    self.get_logger().info(f'Point outside FOV: angle={angle}')

        # Fill the LaserScan message and publish
        laser_scan.ranges = ranges
        self.scan_pub.publish(laser_scan)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

