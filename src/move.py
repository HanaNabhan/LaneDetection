#! /bin/env python3

import rospy
from std_msgs.msg import Float64

def move_car_with_padel(padel: float):
    cmd_vel_pub.publish(padel)

if __name__ == '__main__':
    rospy.init_node("move")
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Float64, queue_size=1)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        padel = rospy.get_param("/padel")
        move_car_with_padel(padel)
        rate.sleep()
    
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")