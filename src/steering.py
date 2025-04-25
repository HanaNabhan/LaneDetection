#! /bin/env python3

import rospy
from std_msgs.msg import Float64

rospy.init_node("steering")
steering_publisher = rospy.Publisher("/SteeringAngle", Float64, queue_size=1)

def steer(steering_angle: float):
    steering_publisher.publish(steering_angle)

if __name__ == '__main__':
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        steering_angle = rospy.get_param("/steering_angle")
        steer(steering_angle)
        rate.sleep()
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")