#! /usr/bin/env python

import rospy
from math import pi, sin, cos, acos, atan2
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, Quaternion
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Int64, Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from base_controller.msg import BaseControllerAction, BaseControllerFeedback, BaseControllerResult
from threading import Lock
from mujoco_engine.core_engine.wrapper.core import MjData, MjModel

class ControlCommand(object):

    def __init__(self):
        # Base velocity control
        self.sub_vel_base = rospy.Subscriber("uwarl_a/cmd_vel", Twist, self.vel_base_callback)
        # Wam position control
        self.sub_wam_pos = rospy.Subscriber("uwarl_a/wam_joints_control/command" , Float64MultiArray, self.wam_pos_callback)
        # Bhand position control
        self.sub_bhand_pos = rospy.Subscriber("/uwarl_a/bhand_joints_control/command" , Float64MultiArray, self.bhand_pos_callback)
        # Wam effort trajectory control
        self.sub_wam_eff = rospy.Subscriber("uwarl_a/eff_based_pos_traj_controller/command" , Float64MultiArray, self.wam_eff_callback)
        
        self.update_bhand_pos = False
        self.update_base = False
        self.update_wam_pos = False
        self.update_wam_eff = False
        # Wait for subscribers to be initialized
        rospy.sleep(0.5)

    def vel_base_callback(self, msg):
        # Store data in array
        self.update_base = True
        self.vel_base = [msg.linear.x, msg.linear.y, msg.angular.z]

    def wam_pos_callback(self, msg):
        # Store data in array
        self.update_wam_pos = True
        self.update_wam_eff = False
        self.wam_pos = msg.data

    def bhand_pos_callback(self, msg):
        # Store data in array
        self.update_bhand_pos = True
        self.bhand_pos = msg.data

    def wam_eff_callback(self, msg):
        # Store data in array
        self.update_wam_eff = True
        self.update_wam_pos = False
        self.wam_eff = msg.data

    def update_controls(self, mj_data):
        # Store data of all callbacks in one array

        # mj_data_copy = MujData.__copy__()

        # rospy.logwarn('Control commands:' + str(mj_data_copy.ctrl))
        # rospy.logwarn('Active controllers:' + str(mj_data_copy.act))

        if self.update_base:
            # print(self.vel_base[0])
            mj_data.actuator('smt/pose/x').ctrl = self.vel_base[0]
            mj_data.actuator('smt/pose/y').ctrl = self.vel_base[1]
            mj_data.actuator('smt/orie/z').ctrl = self.vel_base[2]


        if self.update_bhand_pos:
            # print(self.bhand_pos)
            mj_data.actuator('bhand/f1/prox').ctrl = self.bhand_pos[0]
            mj_data.actuator('bhand/f1/med').ctrl = self.bhand_pos[1]
            mj_data.actuator('bhand/f1/dist').ctrl = self.bhand_pos[2]
            mj_data.actuator('bhand/f2/prox').ctrl = self.bhand_pos[3]
            mj_data.actuator('bhand/f2/med').ctrl = self.bhand_pos[4]
            mj_data.actuator('bhand/f2/dist').ctrl = self.bhand_pos[5]
            mj_data.actuator('bhand/f3/med').ctrl = self.bhand_pos[6]
            mj_data.actuator('bhand/f3/dist').ctrl = self.bhand_pos[7]

        if self.update_wam_pos:
            # rospy.logwarn('Using position controller for wam')
            # print(self.wam_pos[0])
            mj_data.actuator('wam/J1/P').ctrl = self.wam_pos[0]
            mj_data.actuator('wam/J2/P').ctrl = self.wam_pos[1]
            mj_data.actuator('wam/J3/P').ctrl = self.wam_pos[2]
            mj_data.actuator('wam/J4/P').ctrl = self.wam_pos[3]
            mj_data.actuator('wam/J5/P').ctrl = self.wam_pos[4]
            mj_data.actuator('wam/J6/P').ctrl = self.wam_pos[5]
            mj_data.actuator('wam/J7/P').ctrl = self.wam_pos[6]

            mj_data.actuator('wam/J1/F').ctrl = 0
            mj_data.actuator('wam/J2/F').ctrl = 0
            mj_data.actuator('wam/J3/F').ctrl = 0
            mj_data.actuator('wam/J4/F').ctrl = 0
            mj_data.actuator('wam/J5/F').ctrl = 0
            mj_data.actuator('wam/J6/F').ctrl = 0
            mj_data.actuator('wam/J7/F').ctrl = 0
            

            # print(mj_data_copy.actuator('wam/J1/P').ctrl)


            # mj_data_copy.act = np.setdiff1d(mj_data_copy.act, ['wam/J1/F',
            #                                                    'wam/J2/F',
            #                                                    'wam/J3/F',
            #                                                    'wam/J4/F',
            #                                                    'wam/J5/F',
            #                                                    'wam/J6/F',
            #                                                    'wam/J7/F'])
            # De-activate effort controller
            # mj_data_copy.act.remove('wam/J1/F')
            # mj_data_copy.act.remove('wam/J2/F')
            # mj_data_copy.act.remove('wam/J3/F')
            # mj_data_copy.act.remove('wam/J4/F')
            # mj_data_copy.act.remove('wam/J5/F')
            # mj_data_copy.act.remove('wam/J6/F')
            # mj_data_copy.act.remove('wam/J7/F')

        if self.update_wam_eff:
            # rospy.logwarn('Using effort trajectory controller for wam')
            mj_data.actuator('wam/J1/F').ctrl = self.wam_eff[0]
            mj_data.actuator('wam/J2/F').ctrl = self.wam_eff[1]
            mj_data.actuator('wam/J3/F').ctrl = self.wam_eff[2]
            mj_data.actuator('wam/J4/F').ctrl = self.wam_eff[3]
            mj_data.actuator('wam/J5/F').ctrl = self.wam_eff[4]
            mj_data.actuator('wam/J6/F').ctrl = self.wam_eff[5]
            mj_data.actuator('wam/J7/F').ctrl = self.wam_eff[6]

            mj_data.actuator('wam/J1/P').ctrl = []
            mj_data.actuator('wam/J2/P').ctrl = []
            mj_data.actuator('wam/J3/P').ctrl = []
            mj_data.actuator('wam/J4/P').ctrl = []
            mj_data.actuator('wam/J5/P').ctrl = []
            mj_data.actuator('wam/J6/P').ctrl = []
            mj_data.actuator('wam/J7/P').ctrl = []

            # mj_data_copy.act = np.setdiff1d(mj_data_copy.act, ['wam/J1/P',
            #                                                    'wam/J2/P',
            #                                                    'wam/J3/P',
            #                                                    'wam/J4/P',
            #                                                    'wam/J5/P',
            #                                                    'wam/J6/P',
            #                                                    'wam/J7/P'])
            # De-activate position controller
            # mj_data_copy.act.remove('wam/J1/P')
            # mj_data_copy.act.remove('wam/J2/P')
            # mj_data_copy.act.remove('wam/J3/P')
            # mj_data_copy.act.remove('wam/J4/P')
            # mj_data_copy.act.remove('wam/J5/P')
            # mj_data_copy.act.remove('wam/J6/P')
            # mj_data_copy.act.remove('wam/J7/P')
        

        # return mj_data_copy