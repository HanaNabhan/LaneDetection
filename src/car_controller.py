#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# Constants
MIN_DISTANCE_THRESHOLD = 1.0  # Minimum distance of 1 meter
FACTOR = 120 * 0.27778 * 2 / 0.68035

LATERAL_DEVIATION_THRESHOLD = 0.5  # Maximum lateral deviation for points to be considered "in front"
SAFE_DISTANCE = 7.0  # Safe distance in meters

class LidarFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.image_callback)
        self.safe_distance = SAFE_DISTANCE
        self.speed = 0.2  # Initial speed
        self.brakes = 0.0  # Initial brakes
        self.DIS_1 = 0
        self.DIS_2 = 0
        self.prev_car2_distance = 0.0
        self.prev_car2_time = rospy.get_time()

        # ROS subscribers and publishers
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)

        # Create a timer to calculate velocity every second
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def lidar_callback(self, msg):
        # Convert PointCloud2 data to a list of points
        point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = []

        # Read points from the point cloud data and filter by y-axis proximity (front of car 2)
        for point in point_cloud:
            if abs(point[0]) < LATERAL_DEVIATION_THRESHOLD and point[1] > MIN_DISTANCE_THRESHOLD:
                points_list.append([point[0], point[1], point[2]])

        points = np.array(points_list)

        # Ensure there are points to process
        if len(points) == 0:
            return

        # Calculate the distance to the nearest object in front (in y-axis)
        distances = points[:, 1]  # Use y-coordinate for distance in front
        car2_distance = np.min(distances)

        # Calculate velocity of car 2
        current_time = rospy.get_time()
        time_delta = current_time - self.prev_car2_time
        if time_delta > 0:
            car2_velocity = (car2_distance - self.prev_car2_distance) / time_delta
        else:
            car2_velocity = 0.0

        self.prev_car2_distance = car2_distance
        self.prev_car2_time = current_time

        # Update previous and current distances for car 1 (your car)
        self.DIS_1 = self.DIS_2
        self.DIS_2 = car2_distance

        # Adjust speed and brakes of your car (car 1) based on the distance and the speed of car 2
        if car2_distance < self.safe_distance:
            if car2_distance < 6.0 or car2_distance < 7.17:
                self.speed = max(0.0, self.speed - 0.08)  # Slow down
                self.brakes = 1.0
            elif car2_distance < 7.0 and (self.DIS_2 - self.DIS_1) > 1.0:
                self.speed = max(0.05, self.speed - 0.15)  # Slow down
                self.brakes = min(0.2, self.brakes + 0.1)  # Speed up brakes
            else:
                self.speed = max(0.08, self.speed - 0.005)  # Slow down
                self.brakes = min(0.2, self.brakes + 0.05)  # Speed up brakes
        else:
            self.speed = min(0.2, self.speed + 0.01)  # Speed up
            self.brakes = max(0.0, self.brakes - 0.5)  # Release brakes

        # Publish the new speed to the /cmd_vel topic
        self.cmd_vel_pub.publish(Float64(self.speed))
        self.brakes_pub.publish(Float64(self.brakes))

    def timer_callback(self, event):
        # Print the speed of your car (car 1)
        rospy.loginfo(f"Car 1 = {self.speed * FACTOR:.2f} m/s")

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV2 format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Display the image
        cv2.imshow("Vision Sensor View", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('lidar_follower')
    follower = LidarFollower()
    rospy.spin()
