import tf_conversions.posemath
import numpy as np
from geometry_msgs.msg import Pose

def cv2_rvec_to_tf2_quat(cv, rvec):
    # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
    rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                dtype=float)
    rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

    # convert the matrix to a quaternion
    quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
    return quaternion

def cv2_transform_2_tf2(cv, rvec, tvec):
    quaternion = cv2_rvec_to_tf2_quat(rvec)
    return Pose(tvec, quaternion)

