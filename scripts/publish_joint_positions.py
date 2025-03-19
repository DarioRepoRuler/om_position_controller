#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

# Define parameters
num_timesteps = 500      # Total timesteps
total_time = 10.0        # Total trajectory duration (seconds)
dt = total_time / num_timesteps  # Time step interval
t = np.linspace(0, total_time, num_timesteps)  # Time vector

# Initialize trajectory with zeros
desired_positions = np.zeros((num_timesteps, 6))

# Sine wave properties
amplitude = 1.0  # Adjust amplitude if needed
period = 10.0    # Period in seconds
omega = 2 * np.pi / period  # Angular frequency

# Assign sine waves to the last three joints with different phase shifts
desired_positions[:, 3] = amplitude * np.sin(omega * t + 0)         # No delay
desired_positions[:, 4] = amplitude * np.sin(omega * t + np.pi / 3) # Phase shift π/3
desired_positions[:, 5] = amplitude * np.sin(omega * t + 2 * np.pi / 3) # Phase shift 2π/3

def publish_joint_positions():
    rospy.init_node('desired_joint_pos_publisher', anonymous=True)
    pub = rospy.Publisher('/desired_joint_pos', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(int(1/dt))  # 10 Hz (matching the trajectory steps)
    
    rospy.loginfo("Publishing desired joint positions...")

    for t_idx in range(num_timesteps):
        if rospy.is_shutdown():
            break

        msg = Float64MultiArray()
        msg.data = desired_positions[t_idx, :].tolist()  # Extract row as list

        rospy.loginfo("Timestep %d: %s", t_idx, msg.data)
        pub.publish(msg)

        rate.sleep()  # Maintain loop rate

    rospy.loginfo("Finished publishing all timesteps.")

if __name__ == '__main__':
    try:
        publish_joint_positions()
    except rospy.ROSInterruptException:
        pass
