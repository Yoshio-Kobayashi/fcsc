#! /usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

import sys

def yaml_to_CameraInfo(yaml_fname):

    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = calib_data["header"]["frame_id"]
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

if __name__ == "__main__":

    args = sys.argv

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(args[1])

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("my_calibration/camera/color/camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(30)

    # Run publisher
    while not rospy.is_shutdown():
        rate.sleep()
        camera_info_msg.header.stamp = rospy.Time.now()
        publisher.publish(camera_info_msg)
