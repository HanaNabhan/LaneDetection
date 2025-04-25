#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import numpy as np

# Define the Kalman Filter class
class KalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.state_estimate = initial_state
        self.covariance_estimate = initial_covariance
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self):
        # Predict the next state (no state transition model provided, so assume constant velocity model)
        self.state_estimate = self.state_estimate
        self.covariance_estimate = self.covariance_estimate + self.process_noise

    def update(self, measurement):
        # Calculate Kalman Gain
        kalman_gain = self.covariance_estimate / (self.covariance_estimate + self.measurement_noise)
        
        # Update the state estimate
        self.state_estimate = self.state_estimate + kalman_gain * (measurement - self.state_estimate)
        
        # Update the covariance estimate
        self.covariance_estimate = (1 - kalman_gain) * self.covariance_estimate

    def get_state(self):
        return self.state_estimate

# Variances for Gaussian noise
VARIANCE_X = 1.1345
VARIANCE_Y = 0.8245
VARIANCE_YAW = 0.8245

def init_kalman_filters():
    try:
        X_start_position = rospy.get_param("/position_x")
        Y_start_position = rospy.get_param("/position_y")
        YAW_start_position = rospy.get_param("/yaw")
        
        kf_x = KalmanFilter(initial_state=X_start_position, initial_covariance=VARIANCE_X, process_noise=VARIANCE_X, measurement_noise=VARIANCE_X)
        kf_y = KalmanFilter(initial_state=Y_start_position, initial_covariance=VARIANCE_Y, process_noise=VARIANCE_Y, measurement_noise=VARIANCE_Y)
        kf_yaw = KalmanFilter(initial_state=YAW_start_position, initial_covariance=VARIANCE_YAW, process_noise=VARIANCE_YAW, measurement_noise=VARIANCE_YAW)

        return kf_x, kf_y, kf_yaw
    except KeyError as e:
        rospy.logerr(f"Parameter {e} not found")
        rospy.signal_shutdown(f"Parameter {e} not found")
        return None, None, None

kf_x, kf_y, kf_yaw = init_kalman_filters()

# History of filtered states
history_length = 10
x_filtered_history = []
y_filtered_history = []
yaw_filtered_history = []

def odom_filtered():
    global x_filtered_history, y_filtered_history, yaw_filtered_history

    try:
        position_x_noise = rospy.get_param("/position_x_noise")
        position_y_noise = rospy.get_param("/position_y_noise")
        yaw_noise = rospy.get_param("/yaw_noise")

        # Apply the Kalman filter for each position
        kf_x.predict()
        kf_x.update(position_x_noise)
        position_x_filtered = kf_x.get_state()

        kf_y.predict()
        kf_y.update(position_y_noise)
        position_y_filtered = kf_y.get_state()

        kf_yaw.predict()
        kf_yaw.update(yaw_noise)
        yaw_filtered = kf_yaw.get_state()

        # Update the history
        x_filtered_history.append(position_x_filtered)
        y_filtered_history.append(position_y_filtered)
        yaw_filtered_history.append(yaw_filtered)

        # Maintain the history length
        if len(x_filtered_history) > history_length:
            x_filtered_history.pop(0)
        if len(y_filtered_history) > history_length:
            y_filtered_history.pop(0)
        if len(yaw_filtered_history) > history_length:
            yaw_filtered_history.pop(0)

        # Calculate the median of the history
        best_x = float(np.median(x_filtered_history))
        best_y = float(np.median(y_filtered_history))
        best_yaw = float(np.median(yaw_filtered_history))

        rospy.set_param("/position_x_filtered", best_x)
        rospy.set_param("/position_y_filtered", best_y)
        rospy.set_param("/yaw_filtered", best_yaw)

    except KeyError as e:
        rospy.logerr(f"Parameter {e} not found")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

if __name__ == "__main__":
    rospy.init_node("odom_filtered")
    rospy.Publisher("/odom_filtered", Float64, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        odom_filtered()
        rate.sleep()

    # Log a message indicating the shutdown of the node
    rospy.loginfo("Shutdown requested. Exiting...")
