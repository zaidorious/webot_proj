import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray
from controller import Robot
from core_modules.object_detection import ObjectDetection
from core_modules.localization import Localization
from core_modules.lidar_processing import LidarProcessing
from core_modules.longitudinal_control import LongitudinalControl
from sensor_msgs.msg import LaserScan, PointCloud2


class LocalizationAndDetectionNode(Node):
    def __init__(self, model_path, object_codes):
        super().__init__('localization_and_detection')

        self.localization_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose', 10)
        self.detection_publisher = self.create_publisher(
            Int32MultiArray, '/detected_objects', 10)
        self.lidar_publisher = self.create_publisher(
            LaserScan, '/lidar/scan', 10)
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, '/lidar/pointcloud', 10)

        self.robot = Robot()  
        self.timestep = int(self.robot.getBasicTimeStep())

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        self.lidar_processing = LidarProcessing(self.lidar)
        self.object_detection = ObjectDetection(model_path, object_codes)
        self.longitudinal_control = LongitudinalControl(self.robot, target_speed=5.0)

        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz

        self.start_time = self.robot.getTime()

    def publish_data(self):
        self.robot.step(self.timestep)

        current_time = self.robot.getTime() - self.start_time

        self.longitudinal_control.apply_longitudinal_control(current_time)

        localization_msg = Localization.get_pose(
            self.gps, self.imu, self.get_clock())
        self.localization_publisher.publish(localization_msg)

        detected_objects = self.object_detection.detect_objects(self.camera)
        msg = Int32MultiArray()
        msg.data = detected_objects
        self.detection_publisher.publish(msg)

        lidar_msg = self.lidar_processing.create_laserscan_msg(
            self.get_clock())
        self.lidar_publisher.publish(lidar_msg)

        pointcloud_msg = self.lidar_processing.create_pointcloud_msg(
            self.get_clock())
        if pointcloud_msg:
            self.pointcloud_publisher.publish(pointcloud_msg)

        self.get_logger().info("Data Published Successfully.")


def main():
    rclpy.init()
    model_path = '/home/zaid/webot_proj/controllers/camera_status_publisher/new_dataset/runs/detect/yolov8_custom/weights/best.pt'
    object_codes = {"car": 1, "cones": 2}

    node = LocalizationAndDetectionNode(model_path, object_codes)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
