#! /bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64


def odom_noise ():
    # Given variances
    variance_position_x = 2.269 / 2
    variance_position_y = 1.649 / 2
    variance_yaw = 1.649 / 2

    position_x = rospy.get_param("/position_x")
    position_y = rospy.get_param("/position_y")
    yaw = rospy.get_param("/yaw")

    # Adding Gaussian noise
    position_x_noise = position_x + np.random.normal(0, np.sqrt(variance_position_x))
    position_y_noise = position_y + np.random.normal(0, np.sqrt(variance_position_y))
    yaw_noise = yaw + np.random.normal(0, np.sqrt(variance_yaw))

    rospy.set_param("/position_x_noise", position_x_noise)
    rospy.set_param("/position_y_noise", position_y_noise)
    rospy.set_param("/yaw_noise", yaw_noise)

if __name__ == "__main__":
    rospy.init_node("odom_noise")
    rospy.Publisher("/odom_noise",Float64, queue_size = 1)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        odom_noise()
        rate.sleep()
        
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")