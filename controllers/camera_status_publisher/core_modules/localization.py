from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler

class Localization:
    @staticmethod
    def get_pose(gps, imu, clock):
        gps_values = gps.getValues()
        rpy = imu.getRollPitchYaw()

        latitude, longitude, altitude = gps_values
        roll, pitch, yaw = rpy

        localization_msg = PoseWithCovarianceStamped()
        localization_msg.header.stamp = clock.now().to_msg()
        localization_msg.header.frame_id = "map"

        localization_msg.pose.pose.position.x = latitude
        localization_msg.pose.pose.position.y = longitude
        localization_msg.pose.pose.position.z = altitude

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        localization_msg.pose.pose.orientation.x = quaternion[0]
        localization_msg.pose.pose.orientation.y = quaternion[1]
        localization_msg.pose.pose.orientation.z = quaternion[2]
        localization_msg.pose.pose.orientation.w = quaternion[3]

        localization_msg.pose.covariance = [0.0] * 36

        return localization_msg
