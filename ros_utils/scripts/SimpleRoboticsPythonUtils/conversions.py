from tf.transformations import quaternion_matrix
import numpy as np
from geometry_msgs.msg import Pose, Transform, Quaternion, Point, Vector3
import tf
from typing import Union

def cv2_rvec_to_tf2_quat(cv, rvec):
    """Convert opencv rvec to tf2.Quaternion

    Args:
        cv (_type_): OpenCV Module
        rvec (3x1 CvMat): A Rodrigues rotation vector - unit rotation_axis * theta

    Returns:
        tf2.Quaternion
    """
    # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
    rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                dtype=float)
    rotation_matrix[:3, :3], _ = cv.Rodrigues(rvec)

    # convert the matrix to a quaternion
    quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
    return quaternion

def cv2_transform_2_tf2(cv, rvec, tvec):
    """Transform opencv's rvec, tvec to geometry_msgs.Pose

    Args:
        cv (): OpenCV Module
        rvec (3x1 CvMat): A Rodrigues rotation vector - unit rotation_axis * theta
        tvec (3x1 CvMat): A translation vector

    Returns:
        geometry_msgs.Pose: Pose with rvec, tvec
    """
    quaternion = cv2_rvec_to_tf2_quat(rvec)
    return Pose(tvec, quaternion)

def quat_2_np_matrix(q: Quaternion):
    q_arr = np.array([q.x, q.y, q.z, q.w])
    T = np.matrix(quaternion_matrix(q_arr))
    return T

def translation_like_2_np_array(trans: Union[Vector3, Point]):
    return np.array([trans.x, trans.y, trans.z])

def transform_to_cv2_rotation_tvec(cv, trans: Transform):
    T = quat_2_np_matrix(trans.rotation)
    t = translation_like_2_np_array(trans.translation)
    return T[:3, :3], t