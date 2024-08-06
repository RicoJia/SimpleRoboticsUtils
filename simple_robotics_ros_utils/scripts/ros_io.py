import rospkg
import rosbag
import typing
from collections import defaultdict
from time import sleep

def find_ros_package_path(package_name):
    rospack = rospkg.RosPack()
    try:
        package_path = rospack.get_path(package_name)
        return package_path
    except rospkg.ResourceNotFound:
        return f"Package '{package_name}' not found"

def read_ros_bag(bag_path: str, return_topics: typing.List[str]) \
    -> typing.Dict[str, typing.List]:
    try:
        bag = rosbag.Bag(bag_path)
        print(f"Successfully opened bag file: {bag_path}")

        return_dict = defaultdict(list)
        # Iterate through the messages in the bag file
        for i, (topic, msg, t) in enumerate(bag.read_messages()):
            if topic in return_topics:
                return_dict[topic].append(msg)
            print(f'message num: {i}')
        bag.close()
    except rosbag.bag.ROSBagException as e:
        print(f"Error opening bag file: {e}")
    return return_dict