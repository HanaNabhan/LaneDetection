#!/usr/bin/env python3
# This line specifies the path to the Python 3 interpreter.

import rospy
from std_msgs.msg import Float64
import csv
import os



# Main Track code
if __name__ == '_main_':

    # Initialize ROS node with the name "track_1"
    rospy.init_node("track_5")

    # Create a publisher node (if needed)
    track_1_pub = rospy.Publisher("/track_5", Float64, queue_size=1)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        # Get the "lane_1_clear_1_clear" parameter
        brakes = rospy.get_param("brakes")
        speed=rospy.get_param("speed")

      
        rospy.set_param("padel", speed)
        rospy.set_param("breaking", brakes)
            
        
        rate.sleep()