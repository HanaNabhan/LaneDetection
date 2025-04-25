#!/usr/bin/env python3
# This line specifies the path to the Python 3 interpreter.

import rospy
from std_msgs.msg import Float64

# Main Track code
if __name__ == '__main__':

    # Initialize ROS node with the name "track_1"
    rospy.init_node("track_1")

    # Create a publisher node (if needed)
    track_1_pub = rospy.Publisher("/track_1", Float64, queue_size=1)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        # Get the "lane" parameter
        lane = rospy.get_param("lane")

        # Set parameters based on the lane value
        if lane == 0:
            rospy.set_param("padel", 0.2)
            rospy.set_param("breaking", 0.0)
        else:
            rospy.set_param("padel", 0.0)
            rospy.set_param("breaking", 0.6)

        # Sleep for a while to maintain the loop rate
        rate.sleep()
