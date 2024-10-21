#! /usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Lock
import time


class ImageRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        output_bag = "../data/aura32_lobby4.bag"
        self.bag = rosbag.Bag(output_bag, "w")
        self.lock = Lock()
        self.shutdown_flag = False
        # Subscribe to the aligned depth and RGB image topics
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

    def depth_callback(self, data):
        if not self.shutdown_flag:
            try:
                # Convert the depth image to OpenCV format
                depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                # Convert depth from mm to meters
                depth_image = depth_image.astype(np.float32) * 0.001

                # Convert back to ROS Image message
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")

                # Write the depth image to the bag
                with self.lock:
                    self.bag.write("/camera/depth/image", depth_msg)

            except Exception as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

    def rgb_callback(self, data):
        if not self.shutdown_flag:
            try:
                # Write the RGB image directly to the bag
                with self.lock:
                    self.bag.write("/camera/rgb/image_color", data)

            except Exception as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

    def camera_info_callback(self, data):
        if not self.shutdown_flag:
            with self.lock:
                try:
                    # Write the camera info message directly to the bag
                    self.bag.write("/camera/rgb/camera_info", data)
                except Exception as e:
                    rospy.logerr("Error writing camera info to bag: {0}".format(e))

    def shutdown(self):
        self.shutdown_flag = True
        time.sleep(0.1)
        self.bag.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("image_depth_recorder", anonymous=True)
    image_recorder = ImageRecorder()
    rospy.on_shutdown(image_recorder.shutdown)
    rospy.spin()
