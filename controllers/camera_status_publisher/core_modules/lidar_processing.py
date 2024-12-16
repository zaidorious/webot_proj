from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class LidarProcessing:
    """
    A class to handle LiDAR data processing and publishing in ROS 2.
    """
    def __init__(self, lidar_device):
        if lidar_device is None:
            raise ValueError("LiDAR device is not initialized.")
        self.lidar = lidar_device
        self.fov = self.lidar.getFov()  
        self.min_range = self.lidar.getMinRange()  
        self.max_range = self.lidar.getMaxRange() 
        self.horizontal_resolution = self.lidar.getHorizontalResolution()  
        print(f"LiDAR initialized: FOV={self.fov}, Min={self.min_range}, Max={self.max_range}")
    def get_lidar_ranges(self):
        """
        Retrieves the LiDAR range image (distance data for each beam).

        Returns:
            List of distances to detected objects.
        """
        ranges = self.lidar.getRangeImage()
        if ranges is None or len(ranges) == 0:
            print("LiDAR data not available yet.") 
            return [float('inf')] * self.horizontal_resolution
        processed_ranges = [
            r if self.min_range <= r <= self.max_range else float('inf')
            for r in ranges
        ]
        print("Processed Ranges:", processed_ranges)  
        return processed_ranges

    def create_laserscan_msg(self, clock):
        """
        Creates a LaserScan message to publish LiDAR data.

        Args:
            clock: ROS 2 Clock instance to timestamp the message.

        Returns:
            A LaserScan message populated with processed LiDAR data.
        """
        ranges = self.get_lidar_ranges()

        scan_msg = LaserScan()
        scan_msg.header.stamp = clock.now().to_msg() 
        scan_msg.header.frame_id = "lidar_link"  

        scan_msg.angle_min = -self.fov / 2 
        scan_msg.angle_max = self.fov / 2   
        scan_msg.angle_increment = self.fov / self.horizontal_resolution  

        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range

        scan_msg.ranges = ranges

        scan_msg.time_increment = 0.0  
        scan_msg.scan_time = 0.1  

        return scan_msg
    

    def create_pointcloud_msg(self, clock):
        """
        Creates a PointCloud2 message from the LiDAR point cloud data.

        Args:
            clock: ROS 2 Clock instance to timestamp the message.

        Returns:
            A PointCloud2 message populated with LiDAR point cloud data.
        """
        point_cloud = self.lidar.getPointCloud()
        if not point_cloud:
            print("No point cloud data available.")  
            return None

        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = clock.now().to_msg()
        cloud_msg.header.frame_id = "lidar_link"

        cloud_msg.height = 1
        cloud_msg.width = len(point_cloud)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 4 bytes each for x, y, z
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True

        cloud_data = []
        for point in point_cloud:
            cloud_data.append(struct.pack('fff', point.x, point.y, point.z))  
        cloud_msg.data = b''.join(cloud_data)

        return cloud_msg

