import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray
from controller import Robot
from core_modules.object_detection import ObjectDetection  # Directly import the class
from core_modules.localization import Localization         # Directly import the class


class LocalizationAndDetectionNode(Node):
    def __init__(self, model_path, object_codes):
        super().__init__('localization_and_detection')

        self.localization_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose', 10)
        self.detection_publisher = self.create_publisher(
            Int32MultiArray, '/detected_objects', 10)

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(self.timestep)

        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)

        # Initialize ObjectDetection and Localization
        self.object_detection = ObjectDetection(model_path, object_codes)

        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz

    def publish_data(self):
        self.robot.step(self.timestep)

        # Publish localization data
        localization_msg = Localization.get_pose(self.gps, self.imu, self.get_clock())
        self.localization_publisher.publish(localization_msg)
        self.get_logger().info("Published Localization Data")

        # Publish object detection data
        detected_objects = self.object_detection.detect_objects(self.camera)
        msg = Int32MultiArray()
        msg.data = detected_objects
        self.detection_publisher.publish(msg)
        self.get_logger().info(f"Published Detections: {detected_objects}")


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
