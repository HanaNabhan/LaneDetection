#! /bin/env python3

import rospy
from std_msgs.msg import Float64

rospy.init_node("break")
break_publisher = rospy.Publisher("/brakes", Float64, queue_size=1)

def breaks(breaking):
    break_publisher.publish(breaking)

if __name__ == '__main__':
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        breaking = rospy.get_param("/breaking")
        breaks(breaking)
        rate.sleep()
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")