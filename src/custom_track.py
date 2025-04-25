#! /bin/env python3
# This line specifies the path to the Python 3 interpreter.

import rospy
import math  # Added import math
from std_msgs.msg import Float64

def steer_right(steering_angle):
    rospy.loginfo("Steer Right")  
    start_yaw_filtered = rospy.get_param("/yaw_filtered")
    current_yaw_filtered = rospy.get_param("/yaw_filtered")
    # Steer until the car reaches the target x position
    while current_yaw_filtered - start_yaw_filtered > -25:  # Allow a small margin of error
        current_yaw_filtered = rospy.get_param("/yaw_filtered")
        rospy.set_param("/steering_angle", steering_angle)
        rospy.loginfo("Steering")
        rospy.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
        human_detected()

    # Adjust yaw_filtered to straighten the car
    current_yaw_filtered = rospy.get_param("/yaw_filtered")
    while current_yaw_filtered <= start_yaw_filtered:
        current_yaw_filtered = rospy.get_param("/yaw_filtered")
        rospy.set_param("/steering_angle", -steering_angle)
        rospy.loginfo("Adjusting")
        rospy.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
        human_detected()

    # Set steering angle to 0 to move in a straight line
    rospy.set_param("/steering_angle", 0.0)
    rospy.loginfo("Straight line")

def steer_left(steering_angle):
    rospy.loginfo("Steer Left") 
    start_yaw_filtered = rospy.get_param("/yaw_filtered")
    current_yaw_filtered = rospy.get_param("/yaw_filtered")
    # Steer until the car reaches the target x position
    while abs(current_yaw_filtered - start_yaw_filtered) < 25:  # Allow a small margin of error
        current_yaw_filtered = rospy.get_param("/yaw_filtered")
        rospy.set_param("/steering_angle", steering_angle)
        rospy.loginfo("Steering")
        rospy.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
        human_detected()

    # Adjust yaw_filtered to straighten the car
    current_yaw_filtered = rospy.get_param("/yaw_filtered")
    while current_yaw_filtered >= start_yaw_filtered:
        current_yaw_filtered = rospy.get_param("/yaw_filtered")
        rospy.set_param("/steering_angle", -steering_angle)
        rospy.loginfo("Adjusting")
        rospy.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
        human_detected()

    # Set steering angle to 0 to move in a straight line
    rospy.set_param("/steering_angle", 0.0)
    rospy.loginfo("Straight line")

def human_detected():
    if rospy.get_param("/Human_detected"):
            rospy.set_param("/padel", 0.0)
            rospy.set_param("/breaking", 1.0)
            Human_detected = rospy.get_param("/Human_detected")
            while Human_detected:
                 Human_detected = rospy.get_param("/Human_detected")
                 rospy.loginfo("Stopping Because of Human")
    else:
        rospy.set_param("/padel", 0.2)
        rospy.set_param("/breaking", 0.0)

def cone_detected(current_lane):
    state = current_lane
    if rospy.get_param("/Cone_detected") | rospy.get_param("/Car_detected"):
        # Determine the target position based on the current lane
        if state == 0:  # Car is on the right lane
            steer_left(2.7)
            state = 1
        else:  # Car is on the left lane
            # Steer the car to the target position
            steer_right(-2.7)
            state = 0
    return state

def steer_90_right(steering_angle):
    rospy.loginfo("Steer Right")  
    start_yaw_filtered = rospy.get_param("/yaw_filtered")
    current_yaw_filtered = rospy.get_param("/yaw_filtered")
    # Steer until the car reaches the target x position
    while current_yaw_filtered - start_yaw_filtered > -127:  # Allow a small margin of error
        current_yaw_filtered = rospy.get_param("/yaw_filtered")
        rospy.set_param("/steering_angle", steering_angle)
        rospy.loginfo("Steering")
        rospy.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
        human_detected()

    # Set steering angle to 0 to move in a straight line
    rospy.set_param("/steering_angle", 0.0)
    rospy.loginfo("Straight line")

def steer_90_left(steering_angle):
    rospy.loginfo("Steer Left") 
    start_yaw_filtered = rospy.get_param("/yaw_filtered")
    current_yaw_filtered = rospy.get_param("/yaw_filtered")
    # Steer until the car reaches the target x position
    while abs(current_yaw_filtered - start_yaw_filtered) < 127:  # Allow a small margin of error
        current_yaw_filtered = rospy.get_param("/yaw_filtered")
        rospy.set_param("/steering_angle", steering_angle)
        rospy.loginfo("Steering")
        rospy.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
        human_detected()

    # Set steering angle to 0 to move in a straight line
    rospy.set_param("/steering_angle", 0.0)
    rospy.loginfo("Straight line")

# Main Track code
if __name__ == '__main__':
    rospy.init_node("custom_track")
    distance_ = 0
    state = 0
    rospy.set_param("/padel", 0.2)
    rospy.set_param("/breaking", 0.0)

    # Create a publisher node
    track_1 = rospy.Publisher("/custom_track", Float64, queue_size=1)

    current_y = rospy.get_param("/position_y_filtered")  # Fixed variable name
    while (current_y < 196):
        current_y = rospy.get_param("/position_y_filtered")  # Fixed variable name
        human_detected()
        state = cone_detected(state)

    steer_90_right(-20)
    rospy.loginfo("Turn 1")

    current_x = rospy.get_param("/position_y_filtered")  # Fixed variable name
    while (current_x < 48):
        current_x = rospy.get_param("/position_y_filtered")  # Fixed variable name
        human_detected()
        state = cone_detected(state)

    steer_90_left(20)
    rospy.loginfo("Turn 2")

    current_y = rospy.get_param("/position_y_filtered")  # Fixed variable name
    while (current_y < 241):
        current_y = rospy.get_param("/position_y_filtered")  # Fixed variable name
        human_detected()
        state = cone_detected(state)

    steer_90_left(20)
    rospy.loginfo("Turn 3")

    current_x = rospy.get_param("/position_y_filtered")  # Fixed variable name
    while (current_x > -27):
        current_x = rospy.get_param("/position_y_filtered")  # Fixed variable name
        human_detected()
        state = cone_detected(state)
    
    steer_90_left(20)
    rospy.loginfo("Turn 4")

    current_y = rospy.get_param("/position_y_filtered")  # Fixed variable name
    while (current_y > 200):
        current_y = rospy.get_param("/position_y_filtered")  # Fixed variable name
        human_detected()
        state = cone_detected(state)

    # Set "gase padel" parameter to 0.0 to indicate no acceleration
    rospy.set_param("/padel", 0.0)          # No acceleration
    # Set "breaking" parameter to 0.2 to indicate moderate braking
    rospy.set_param("/breaking", 0.2)       # Gentle braking
    rospy.loginfo("Custom Track completed.")
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")