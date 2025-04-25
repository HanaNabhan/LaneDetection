#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
import tf

def Imu_callBack(msg: Imu):
    # Angular Velocity of the car in 3D
    rospy.set_param("/angular_velocity_x", msg.angular_velocity.x)
    rospy.set_param("/angular_velocity_y", msg.angular_velocity.y)
    rospy.set_param("/angular_velocity_z", msg.angular_velocity.z)

    # Overall Angular Velocity of the car
    overall_angular_velocity = math.sqrt((msg.angular_velocity.x)**2 + (msg.angular_velocity.y)**2 + (msg.angular_velocity.z)**2)
    rospy.set_param("/overall_angular_velocity", overall_angular_velocity)

    # Linear acceleration of the car in 3D
    rospy.set_param("/linear_acceleration_x", msg.linear_acceleration.x)
    rospy.set_param("/linear_acceleration_y", msg.linear_acceleration.y)
    rospy.set_param("/linear_acceleration_z", msg.linear_acceleration.z)

    # Linear acceleration of the car
    overall_linear_acceleration = math.sqrt((msg.linear_acceleration.x)**2 + (msg.linear_acceleration.y)**2 + (msg.linear_acceleration.z)**2)
    rospy.set_param("/overall_linear_acceleration", overall_linear_acceleration)

    # Extract quaternion from the IMU message
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )

    # Convert quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # Extract the angular positions (roll, pitch, yaw)
    roll = euler[0] * 100
    pitch = euler[1] * 100
    yaw = euler[2] * 100

    # Set angular positions as ROS parameters
    if roll != 0 and pitch != 0 and yaw != 0:
        rospy.set_param("/roll", roll)
        rospy.set_param("/pitch", pitch)
        rospy.set_param("/yaw", yaw)

if __name__ == "__main__":
    rospy.init_node("Imu_node")
    rospy.Subscriber("/Imu", Imu, Imu_callBack, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()
        
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")
