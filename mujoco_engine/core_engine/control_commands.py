#! /usr/bin/env python

# Subscribers for reading control commands from hw_interface and writing them into mj_data to be used next engine step
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class ControlCommand(object):

    def __init__(self,mj_data,cmd_vel_topic_name):
        # Base velocity effort control
        self.sub_vel_base_summ = rospy.Subscriber(cmd_vel_topic_name, Twist, self.vel_base_callback)

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

    # PID loop to control x velocity of non-holonomic-robot in map frame
    def velx_PID(self, Kp, Ki, Kd, CP, base_name):

        # Convert reference velocities into mapframe
        SP = self.vel_base[0]*cos(self.mj_data_control.joint(base_name+'/orie/z').qpos[0])+self.vel_base[1]*-sin(self.mj_data_control.joint(base_name+'/orie/z').qpos[0])

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
        self.mj_data_control.actuator(base_name+'/pose/x').ctrl = control


    # PID loop to control y velocity of holonomic-robot in map frame
    def vely_PID(self, Kp, Ki, Kd, CP, base_name):

        # Convert reference velocities into mapframe
        SP = self.vel_base[0]*sin(self.mj_data_control.joint(base_name+'/orie/z').qpos[0])+self.vel_base[1]*cos(self.mj_data_control.joint(base_name+'/orie/z').qpos[0])

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
        self.mj_data_control.actuator(base_name+'/pose/y').ctrl = control

    
    # PID loop to control yaw rate of holonomic-robot in map frame
    def veltheta_PID(self, Kp, Ki, Kd, CP,base_name):

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
        self.mj_data_control.actuator(base_name+'/orie/z').ctrl = control
