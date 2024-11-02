import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node')

        # Set up subscribers for the color and depth images
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

        self.bridge = CvBridge()
        self.prev_image = None

    def image_callback(self, data):
        current_image = self.bridge.imgmsg_to_cv2(data, "mono8")  # Using grayscale for simplicity
        current_image_np = np.array(current_image)

        if self.prev_image is not None:
            # Calculate visual odometry between prev_image and current_image
            self.calculate_visual_odometry(self.prev_image, current_image_np)
        
        self.prev_image = current_image_np

    def depth_callback(self, data):
        # Process depth data if needed for more accurate odometry
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32)

    def calculate_visual_odometry(self, prev_img, curr_img):
        """
        Simple feature-based odometry using corner detection (no OpenCV)
        """
        # Detect features using a basic Harris Corner Detection approach
        prev_features = self.harris_corner_detector(prev_img)
        curr_features = self.harris_corner_detector(curr_img)

        # Match features (simple nearest-neighbor)
        matched_features = self.match_features(prev_features, curr_features)

        # Compute basic transformation (this can be improved with more robust methods)
        if len(matched_features) > 0:
            self.estimate_motion(matched_features)

    def harris_corner_detector(self, image):
        """
        A basic corner detection algorithm that approximates feature detection.
        """
        # Compute the gradients in the x and y directions
        grad_x, grad_y = np.gradient(image)
        
        # Apply a threshold to detect significant gradients (features)
        threshold = 0.1
        features_x = np.where(grad_x > threshold)  # Features from x-gradient
        features_y = np.where(grad_y > threshold)  # Features from y-gradient

        # Combine the detected features from both directions
        features = np.hstack((features_x, features_y))

        return features

    def match_features(self, prev_features, curr_features):
        """
        A basic feature matching algorithm using nearest neighbors.
        """
        matches = []
        for pf in prev_features:
            best_match = min(curr_features, key=lambda cf: np.linalg.norm(pf - cf))
            matches.append((pf, best_match))
        return matches

    def estimate_motion(self, matched_features):
        """
        Estimate motion between two sets of matched features.
        """
        # Use basic transformation estimation (can be enhanced with more sophisticated methods)
        prev_points = np.array([m[0] for m in matched_features])
        curr_points = np.array([m[1] for m in matched_features])

        # Estimate translation and rotation (for simplicity, assuming 2D motion)
        translation = np.mean(curr_points - prev_points, axis=0)
        rotation = np.arctan2(curr_points[:, 1], curr_points[:, 0]) - np.arctan2(prev_points[:, 1], prev_points[:, 0])

        self.get_logger().info(f"Estimated motion: translation={translation}, rotation={rotation}")

def main(args=None):
    rclpy.init(args=args)
    vo_node = VisualOdometryNode()
    rclpy.spin(vo_node)
    vo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

