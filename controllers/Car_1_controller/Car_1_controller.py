import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # ROS2 message type for integers
from controller import Robot

class CameraStatusPublisher(Node):
    def __init__(self):
        super().__init__('camera_status_publisher')

        # ROS2 publisher
        self.publisher_ = self.create_publisher(Int32, 'camera_status', 10)
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Camera initialization
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)

        self.timer = self.create_timer(0.1, self.publish_status)  # 10 Hz publishing

    def publish_status(self):
        # Check if the camera is working
        try:
            image = self.camera.getImage()  # Attempt to capture an image
            status = 1 if image else 0      # 1 if image is not None, else 0
        except:
            status = 0                      # If exception occurs, set status to 0

        # Publish the status
        msg = Int32()
        msg.data = status
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing camera status: {status}')

    def run(self):
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = CameraStatusPublisher()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # ROS2 message type for integers
from controller import Robot

class CameraStatusPublisher(Node):
    def __init__(self):
        super().__init__('camera_status_publisher')

        # ROS2 publisher
        self.publisher_ = self.create_publisher(Int32, 'camera_status', 10)
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Camera initialization
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)

        self.timer = self.create_timer(0.1, self.publish_status)  # 10 Hz publishing

    def publish_status(self):
        # Check if the camera is working
        try:
            image = self.camera.getImage()  # Attempt to capture an image
            status = 1 if image else 0      # 1 if image is not None, else 0
        except:
            status = 0                      # If exception occurs, set status to 0

        # Publish the status
        msg = Int32()
        msg.data = status
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing camera status: {status}')

    def run(self):
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = CameraStatusPublisher()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
