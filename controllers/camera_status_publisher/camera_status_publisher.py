import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import NavSatFix
from controller import Robot
from tf_transformations import quaternion_from_euler
import cv2
import numpy as np
from ultralytics import YOLO


class LocalizationAndDetectionNode(Node):
    def __init__(self, model_path):
        super().__init__('localization_and_detection')

        # Publishers
        self.localization_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/localization/pose', 10)
        self.detection_publisher = self.create_publisher(
            Int32MultiArray, '/detected_objects', 10)

        # Webots Robot Initialization
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # GPS Initialization
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        # IMU Initialization
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(self.timestep)

        # Camera Initialization
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)

        # YOLOv8 Model Initialization
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("YOLOv8 model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            raise

        # Object code mapping
        self.object_codes = {
            "car": 1,
            "cones": 2
        }

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.publish_data)  # 10 Hz

    def publish_data(self):
        self.robot.step(self.timestep)

        # Publish localization data
        gps_values = self.gps.getValues()
        rpy = self.imu.getRollPitchYaw()
        self.publish_localization(gps_values, rpy)

        # Publish object detection data
        self.publish_detections()

    def publish_localization(self, gps_values, rpy):
        latitude, longitude, altitude = gps_values
        roll, pitch, yaw = rpy

        # Create PoseWithCovarianceStamped message
        localization_msg = PoseWithCovarianceStamped()
        localization_msg.header.stamp = self.get_clock().now().to_msg()
        localization_msg.header.frame_id = "map"

        # Position (from GPS)
        # Latitude (converted if needed)
        localization_msg.pose.pose.position.x = latitude
        # Longitude (converted if needed)
        localization_msg.pose.pose.position.y = longitude
        localization_msg.pose.pose.position.z = altitude   # Altitude

        # Orientation (from IMU)
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        localization_msg.pose.pose.orientation.x = quaternion[0]
        localization_msg.pose.pose.orientation.y = quaternion[1]
        localization_msg.pose.pose.orientation.z = quaternion[2]
        localization_msg.pose.pose.orientation.w = quaternion[3]

        # Placeholder covariance matrix (can be adjusted)
        localization_msg.pose.covariance = [0.0] * 36

        self.localization_publisher.publish(localization_msg)
        self.get_logger().info(
            f"Published Localization: Position ({latitude}, {longitude}, {altitude}), Orientation (roll={roll}, pitch={pitch}, yaw={yaw})")

    def publish_detections(self):
        # Capture camera image
        raw_image = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = np.frombuffer(raw_image, dtype=np.uint8).reshape(
            (height, width, 4))  # BGRA format
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)

        # Run YOLO inference
        results = self.model(image_rgb)
        detected_codes = []

        # Process YOLO results
        for result in results:
            if result.boxes:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    label = self.model.names[
                        class_id] if self.model.names else f"class_{class_id}"

                    if confidence > 0.5:  # Filter low-confidence detections
                        # Use -1 for unknown objects
                        code = self.object_codes.get(label, -1)
                        if code != -1:
                            detected_codes.append(code)

        # Publish detection results
        unique_codes = list(set(detected_codes))  # Remove duplicates
        msg = Int32MultiArray()
        msg.data = unique_codes
        self.detection_publisher.publish(msg)
        self.get_logger().info(f"Published Detections: {unique_codes}")


def main():
    rclpy.init()

    # Path to YOLOv8 model
    model_path = '/home/zaid/webot_proj/controllers/camera_status_publisher/new_dataset/runs/detect/yolov8_custom/weights/best.pt'

    # Initialize and run the node
    node = LocalizationAndDetectionNode(model_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
