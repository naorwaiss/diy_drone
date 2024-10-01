import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.Qt3DCore import QEntity, QTransform
from PyQt5.Qt3DExtras import Qt3DWindow, QOrbitCameraController, QCylinderMesh, QPhongMaterial
from PyQt5.QtGui import QVector3D, QQuaternion
from sensor_msgs.msg import Imu


class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/mpu6050_imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.orientation = QQuaternion()

    def listener_callback(self, msg):
        # Get quaternion data from IMU message
        self.orientation = QQuaternion(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        )


class IMUVisualizer(QWidget):
    def __init__(self, imu_subscriber):
        super().__init__()
        self.imu_subscriber = imu_subscriber
        self.initUI()

    def initUI(self):
        # Set up the Qt3D window
        self.view = Qt3DWindow()
        container = self.createWindowContainer(self.view)
        layout = QVBoxLayout()
        layout.addWidget(container)
        self.setLayout(layout)

        # Root entity
        self.rootEntity = QEntity()

        # Create a cylinder to represent the rigid body
        self.cylinderEntity = QEntity(self.rootEntity)
        cylinderMesh = QCylinderMesh()
        cylinderMesh.setRadius(0.05)
        cylinderMesh.setLength(1.0)
        cylinderMesh.setRings(100)
        cylinderMesh.setSlices(20)

        cylinderMaterial = QPhongMaterial(self.rootEntity)
        self.cylinderTransform = QTransform()

        self.cylinderEntity.addComponent(cylinderMesh)
        self.cylinderEntity.addComponent(cylinderMaterial)
        self.cylinderEntity.addComponent(self.cylinderTransform)

        # Camera setup
        self.camera = self.view.camera()
        self.camera.lens().setPerspectiveProjection(45.0, 16.0 / 9.0, 0.1, 1000.0)
        self.camera.setPosition(QVector3D(0, 0, 5))
        self.camera.setViewCenter(QVector3D(0, 0, 0))

        # For controlling the camera with mouse
        camController = QOrbitCameraController(self.rootEntity)
        camController.setCamera(self.camera)

        # Set root object of the scene
        self.view.setRootEntity(self.rootEntity)

    def update_orientation(self):
        # Get the current quaternion from the IMU subscriber and apply it to the 3D model
        quat = self.imu_subscriber.orientation
        self.cylinderTransform.setRotation(quat)


def main(args=None):
    # Initialize ROS2 and the subscriber
    rclpy.init(args=args)
    imu_subscriber = IMUSubscriber()

    # Initialize the PyQt application
    app = QApplication(sys.argv)
    visualizer = IMUVisualizer(imu_subscriber)

    # Update the visualization periodically
    timer = visualizer.startTimer(50)  # 50ms = 20Hz refresh rate
    def update():
        visualizer.update_orientation()

    # Timer event to continuously update the orientation from the IMU
    visualizer.timerEvent = lambda event: update()

    visualizer.show()

    try:
        # Run the application
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
