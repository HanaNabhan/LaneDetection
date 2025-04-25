#! /bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry

def odom_callBack (msg:Odometry):
    # Position of Vechile (x, y, z)
    rospy.set_param("/position_x", msg.pose.pose.position.x)
    rospy.set_param("/position_y", msg.pose.pose.position.y)
    rospy.set_param("/position_z", msg.pose.pose.position.z)

    # Velocity of Vechile (x, y, z)
    rospy.set_param("/velocity_x", msg.twist.twist.linear.x)
    rospy.set_param("/velocity_y", msg.twist.twist.linear.y)
    rospy.set_param("/velocity_z", msg.twist.twist.linear.z)

    # Angler of Vechile (x, y, z)
    rospy.set_param("/angular_x", msg.twist.twist.angular.x)
    rospy.set_param("/angular_y", msg.twist.twist.angular.y)
    rospy.set_param("/angular_z", msg.twist.twist.angular.z)

    # Linear acceleration of the car
    overall_velocity = math.sqrt((msg.twist.twist.linear.x)**2 + (msg.twist.twist.linear.y)**2 + (msg.twist.twist.linear.z)**2)
    rospy.set_param("/overall_velocity", overall_velocity)

if __name__ == "__main__":
    rospy.init_node("odom")
    rospy.Subscriber(f"/odom",Odometry,odom_callBack, queue_size = 1)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()
        
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")