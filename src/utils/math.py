from tf.transformations import euler_from_quaternion

def get_yaw(msg):
    """
    Computes yaw from a Odom message.
    Parameters:
        msg (nav_msgs.msg.Odometry): The message to extract yaw from.
    Returns:
        float: The yaw in radians.
    """
    return euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ])[2]