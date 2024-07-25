#! /usr/bin/env python

# Subscribers for reading control commands from hw_interface and writing them into mj_data to be used next engine step
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class ControlCommand(object):

    def __init__(self,mj_data):
        # Base velocity effort control
        self.sub_vel_base = rospy.Subscriber("uwarl_a/robotnik_base_control/cmd_vel", Twist, self.vel_base_callback)
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

        self.vel_base = [0.0, 0.0, 0.0]


    def vel_base_callback(self, msg):
        # Store data in array to be used by PID control
        # New velocity commands are received at approximately 50 Hz, PID loop runs at 200 Hz
        self.vel_base = [msg.linear.x, msg.linear.y, msg.angular.z]


    # PID loop to control x velocity in map frame
    def velx_PID(self, Kp, Ki, Kd, CP):

        # Convert reference velocities into mapframe
        SP = self.vel_base[0]*cos(self.mj_data_control.joint('smt/orie/z').qpos[0])+self.vel_base[1]*-sin(self.mj_data_control.joint('smt/orie/z').qpos[0])

        t = rospy.Time().now().to_time()
        ex = float(SP - CP)

        # Compute PID variables
        P = ex*Kp
        self.Ix += Ki*ex*(t-self.last_time_x+0.0001) # Make sure it is never 0.0
        D = Kd*(ex-self.ex_last)/(t-self.last_time_x+0.0001) # Make sure it is never 0.0

        self.ex_last = ex
        self.last_time_x = t

        control = P+self.Ix+D

        # Set control commands in mj_data
        self.mj_data_control.actuator('smt/pose/x').ctrl = control


    # PID loop to control y velocity in map frame
    def vely_PID(self, Kp, Ki, Kd, CP):

        # Convert reference velocities into mapframe
        SP = self.vel_base[0]*sin(self.mj_data_control.joint('smt/orie/z').qpos[0])+self.vel_base[1]*cos(self.mj_data_control.joint('smt/orie/z').qpos[0])

        t = rospy.Time().now().to_time()
        ey = float(SP - CP)

        # Compute PID variables
        P = ey*Kp
        self.Iy += Ki*ey*(t-self.last_time_y+0.0001)
        D = Kd*(ey-self.ey_last)/(t-self.last_time_y+0.0001)

        self.ey_last = ey
        self.last_time_y = t

        control = P+self.Iy+D

        # Set control commands in mj_data
        self.mj_data_control.actuator('smt/pose/y').ctrl = control

    
    # PID loop to control yaw rate in map frame
    def veltheta_PID(self, Kp, Ki, Kd, CP):

        SP = self.vel_base[2]

        t = rospy.Time().now().to_time()
        etheta = float(SP - CP)

        # Compute PID variables
        P = etheta*Kp
        self.Itheta += Ki*etheta*(t-self.last_time_theta+0.0001)
        D = Kd*(etheta-self.etheta_last)/(t-self.last_time_theta+0.0001)

        self.etheta_last = etheta
        self.last_time_theta = t

        control = P+self.Itheta+D

        # Set control commands in mj_data
        self.mj_data_control.actuator('smt/orie/z').ctrl = control


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
