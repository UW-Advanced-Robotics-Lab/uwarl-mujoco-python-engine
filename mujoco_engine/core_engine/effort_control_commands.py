#! /usr/bin/env python

# Subscribers for reading control commands from hw_interface and writing them into mj_data to be used next engine step
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class EffortControlCommand(object):

    def __init__(self,mj_data):
        # Wam position effort control
        # This topic broadcasts only those actuators which have a transmission associated with it in that robots' urdf-file.
        # So, other non-fixed joints that are not being controlled through a motor, like wheel joints, are not available here.
        self.sub_wam_pos = rospy.Subscriber("/mujoco/ros_control/effort_commands" ,JointState, self.effort_callback)

        # Create pointer to mujoco data
        # This contains references to joints which have actuators associated with them (they are mentioned in files similar to `include_summit_wam_bhand_actuators.xml`)
        self.mj_data_control = mj_data

        # Initialize PID variables for base velocity effort control
        self.Ix = 0.0
        self.ex_last = 0.0
        self.last_time_x = rospy.Time().now().to_time()

        self.Iy = 0.0
        self.ey_last = 0.0
        self.last_time_y = rospy.Time().now().to_time()

        self.Itheta = 0.0
        self.etheta_last = 0.0
        self.last_time_theta = rospy.Time().now().to_time()

        self.vel_summ_base = [0.0, 0.0, 0.0]
        self.vel_fetch_base = [0.0, 0.0, 0.0]

    # Callback for hw_sim_interface effort commands
    def effort_callback(self, msg):
        self.efforts = msg
        i = 0
        # Set control commands in mj_data
        for i in range(0, len(self.efforts.name)):

            # Add "/F" for arm force actuators. This is only for the sake of differentiating actuators which are only position-control, and those that can be (F)orce-controlled.
            if ("wam" in self.efforts.name[i]) or ("fetch_arm" in self.efforts.name[i]):
                # This topic broadcasts only those actuators which have a transmission associated with it in that robots' urdf-file.
                # So, other non-fixed joints that are not being controlled through a motor, like wheel joints, are not available here.
                self.mj_data_control.actuator(self.efforts.name[i]+'/F').ctrl = self.efforts.effort[i]
                # Why do this? Because we are using `/F` , appended onto the name of the joint, to dictate the motor name (include_summit_wam_bhand_actuators.xml)
                # that can be force-controlled (the rest will all be position controlled)
            else:
                self.mj_data_control.actuator(self.efforts.name[i]).ctrl = self.efforts.effort[i]

            i+=1
