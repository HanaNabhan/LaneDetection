#! /bin/env python3

import time
import os
import csv
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

def write_to_csv(filename, data):
    # Create the directory if it does not exist
    directory = os.path.dirname(filename)
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Open the CSV file in write mode (append mode if the file already exists)
    with open(filename, mode='a', newline='') as csv_file:
        # Define the CSV file header
        fieldnames = [
            'Time', 'padel', 'breaking', 'steering_angle', 'position_x', 'position_y', 'position_z',
            'velocity_x', 'velocity_y', 'velocity_z', 'overall_velocity', 'angular_x', 'angular_y', 
            'angular_z', 'position_x_noise', 'position_y_noise', 'yaw_noise', 
            'position_x_filtered', 'position_y_filtered', 'yaw_filtered', 
            'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z', 
            'overall_linear_acceleration', 'roll', 'pitch', 'yaw'
        ]

        # Create a CSV writer object
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        # Write the header only if the file is empty
        if os.stat(filename).st_size == 0:
            writer.writeheader()

        # Write the data to the CSV file
        writer.writerow(data)

if __name__ == '__main__':
    # Initialize ROS node with the name "csv_gen"
    rospy.init_node("csv_gen")

    # File path for the CSV file on desktop
    desktop_path = os.path.join(os.path.expanduser("~"), "Desktop/Ever/Submission_3/Data_Analysis")
    filename = os.path.join(desktop_path, 'data_output.csv')
    rate = rospy.Rate(10)  # 10 Hz  
    
    rospy.wait_for_message("/Imu", Imu)
    # Save the start time
    start_time = time.time()
    while not rospy.is_shutdown():
        rate.sleep()
        # Get Data
        data = {
            'Time': (time.time() - start_time),
            'padel': rospy.get_param("/padel"),
            'breaking': rospy.get_param("/breaking"),
            'steering_angle': rospy.get_param("/steering_angle"),

            'position_x': rospy.get_param("/position_x"),
            'position_y': rospy.get_param("/position_y"),
            'position_z': rospy.get_param("/position_z"),

            'velocity_x': rospy.get_param("/velocity_x"),
            'velocity_y': rospy.get_param("/velocity_y"),
            'velocity_z': rospy.get_param("/velocity_z"),
            'overall_velocity': rospy.get_param("/overall_velocity"),

            'angular_x': rospy.get_param("/angular_x"),
            'angular_y': rospy.get_param("/angular_y"),
            'angular_z': rospy.get_param("/angular_z"),

            'position_x_noise': rospy.get_param("/position_x_noise"),
            'position_y_noise': rospy.get_param("/position_y_noise"),
            'yaw_noise': rospy.get_param("/yaw_noise"),

            'position_x_filtered': rospy.get_param("/position_x_filtered"),
            'position_y_filtered': rospy.get_param("/position_y_filtered"),
            'yaw_filtered': rospy.get_param("/yaw_filtered"),
            
            'linear_acceleration_x': rospy.get_param("/linear_acceleration_x"),
            'linear_acceleration_y': rospy.get_param("/linear_acceleration_y"),
            'linear_acceleration_z': rospy.get_param("/linear_acceleration_z"),
            
            'overall_linear_acceleration': rospy.get_param("/overall_linear_acceleration"),
            
            'roll': rospy.get_param("/roll"),
            'pitch': rospy.get_param("/pitch"),
            'yaw': rospy.get_param("/yaw")
        }
        
        # Write the variables to the CSV file
        write_to_csv(filename, data)

    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")

