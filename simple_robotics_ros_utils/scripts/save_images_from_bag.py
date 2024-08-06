#!/usr/bin/env python3
from simple_robotics_python_utils.common import io, logger
import sys
import argparse
import ros_io
import os
from cv_bridge import CvBridge
import cv2

def ask_path(item_name: str, data_path: str):
    cevab = io.ask_user(
        msg = f"Is the {item_name} path correct? If not, please come into the script and fix it. "
        f"Path: {data_path}",
        options=["yes", "no"]
    )

    if cevab == "no":
        print(f'Please come back after adjusting the {item_name} path.')
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--package_name", type=str, required=True)
    parser.add_argument("--bag_name", type=str, required=True)
    args = parser.parse_args()

    package_name = args.package_name
    package_path = ros_io.find_ros_package_path(package_name)
    data_path = os.path.join(package_path, "data/bag_images")
    
    bag_name = args.bag_name
    bag_path = os.path.join(package_path, "data", bag_name)
    
    ask_path(item_name="data saving", data_path=data_path) 
    ask_path(item_name="bag reading", data_path=bag_path) 
    
    l = logger.get_logger("bag_saver") 
    l.info(f"Saving bags to {data_path}")
    
    TOPIC = "/camera/rgb/image_color"
    msg_dict = ros_io.read_ros_bag(bag_path=bag_path, return_topics=TOPIC)
    ros_bridge = CvBridge()
    for idx, msg in enumerate(msg_dict[TOPIC]):
        cv_img = ros_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_file_path = os.path.join(data_path, f"frame_{idx:04d}.png")
        # Save the image as a PNG file
        cv2.imwrite(img_file_path, cv_img)

    
    