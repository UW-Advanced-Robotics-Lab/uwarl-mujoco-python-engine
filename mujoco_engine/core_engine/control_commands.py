#! /usr/bin/env python

# Subscribers for reading control commands from hw_interface and writing them into mj_data to be used next engine step
# Last version: Nov 28, 2023 Tim van Meijel

import numpy as np
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

        # Summit H-matrix
        gamma_smt = 45/180*np.pi
        dx_1_smt = 0.222
        dx_2_smt = 0.223
        dy_1_smt = 0.22
        dy_2_smt = 0.22
        whl_rad_smt = 0.127
        tan_gamma_smt = np.tan(gamma_smt)
        self.h_mat_smt = np.array([[1,  tan_gamma_smt, -dy_1_smt+dx_1_smt*tan_gamma_smt],
                                   [1, -tan_gamma_smt,  dy_2_smt-dx_1_smt*tan_gamma_smt],
                                   [1,  tan_gamma_smt,  dy_2_smt-dx_2_smt*tan_gamma_smt],
                                   [1, -tan_gamma_smt, -dy_1_smt+dx_2_smt*tan_gamma_smt]])/whl_rad_smt
        self.e_whl_last_smt = np.zeros((4,1))
        self.last_time_whl_smt = rospy.Time().now().to_time()
        self.I_whl_smt = np.zeros((4,1))


    def vel_base_callback(self, msg):
        # Store data in array to be used by PID control
        # New velocity commands are received at approximately 50 Hz, PID loop runs at 200 Hz
        self.vel_base = [msg.linear.x, msg.linear.y, msg.angular.z]

    # PID loop to control x velocity of non-holonomic-robot in map frame
    def velx_PID(self, Kp, Ki, Kd, CP, base_name):

        # Convert reference velocities into mapframe
        SP = self.vel_base[0]

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
        SP = self.vel_base[1]
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
    
    # PID loop to control wheel velocities
    def wheel_PID(self, Kp, Ki, Kd, CP, base_name):

        # Convert reference velocities into mapframe
        temp_mb_twist = np.array(self.vel_base)
        mb_twist = temp_mb_twist.reshape(3,1)

        # Wheel velocity: Set point
        whl_vel_SP = np.zeros((4,1))
        if(base_name is "smt"):
            whl_vel_SP = np.matmul(self.h_mat_smt,mb_twist)

        t = rospy.Time().now().to_time()
        # print("Wheel velocity set-point:")
        # print(whl_vel_SP)
        # print(CP)
        e_whl = whl_vel_SP - CP

        # Compute PID variables
        P = e_whl*Kp
        self.I_whl_smt += Ki*e_whl*(t-self.last_time_whl_smt+0.0001)
        D = Kd*(e_whl-self.e_whl_last_smt)/(t-self.last_time_whl_smt+0.0001)

        self.e_whl_last_smt = e_whl
        self.last_time_whl_smt = t

        control = P+self.I_whl_smt+D

        # print(P)
        # print(self.I_whl_smt)
        # print(D)
        # Set control commands in mj_data
        self.mj_data_control.actuator(base_name+'/whl_LF').ctrl = control[0,0]
        self.mj_data_control.actuator(base_name+'/whl_RF').ctrl = control[1,0]
        self.mj_data_control.actuator(base_name+'/whl_RR').ctrl = control[2,0]
        self.mj_data_control.actuator(base_name+'/whl_LR').ctrl = control[3,0]
