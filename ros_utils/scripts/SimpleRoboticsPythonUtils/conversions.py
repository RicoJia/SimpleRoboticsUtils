from tf.transformations import quaternion_matrix
import numpy as np
from geometry_msgs.msg import Pose, Transform, Quaternion
import tf

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

def quat_2_np_array(q: Quaternion):
    return np.array([q.x, q.y, q.z, q.w])

def transform_to_cv2_rvec_tvec(cv, trans: Transform):
    q_arr = quat_2_np_array(trans.rotation)
    quat_mat = np.matrix(quaternion_matrix(q_arr))
    print(quat_mat)