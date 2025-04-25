#! /bin/env python3
# This line specifies the path to the Python 3 interpreter.
import rospy
import math  # Added import math
from std_msgs.msg import Float64

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  # Fixed missing parenthesis

def human_detected():
    if rospy.get_param("/Human_detected"):
            rospy.set_param("/padel", 0.0)
            rospy.set_param("/breaking", 1.0)
            obj_Human_detected = rospy.get_param("/Human_detected")
            while obj_Human_detected:
                 obj_Human_detected = rospy.get_param("/Human_detected")
    else:
        rospy.set_param("/padel", 0.1)
        rospy.set_param("/breaking", 0.0)

x = -1

def cone_detected():
    if rospy.get_param("/Cone_detected"):
        # Adjust yaw to straighten the car
        start_yaw = rospy.get_param("/yaw_filtered")
        current_yaw = rospy.get_param("/yaw_filtered")

        while abs(current_yaw - start_yaw) < 9:
            current_yaw = rospy.get_param("/yaw_filtered")
            rospy.set_param("/steering_angle", 20)
            rospy.loginfo("steer")
            rospy.sleep(0.1)  # Add a small delay to avoid excessive CPU usage
            human_detected()

    rospy.loginfo("continue")
    rospy.set_param("/steering_angle", 13.0)

# Main Track code
if __name__ == '__main__':
    rospy.init_node("circular_path")
    # Create a publisher node
    track_1 = rospy.Publisher("/circular_path", Float64, queue_size=1)    

    rospy.set_param("/steering_angle", 13.0)

    while (rospy.get_param("/yaw_filtered") < 350):
        human_detected()
        cone_detected()

    # Set "gase padel" parameter to 0.0 to indicate no acceleration
    rospy.set_param("/padel", 0.0)          # No acceleration
    # Set "breaking" parameter to 0.2 to indicate moderate braking
    rospy.set_param("/breaking", 0.2)       # Gentle braking
    
    rospy.loginfo("Custom Track completed.")
    
    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")

