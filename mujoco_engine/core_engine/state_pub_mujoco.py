#! /usr/bin/env python

import rospy
from math import pi, sin, cos, acos, atan2
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, Quaternion
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from base_controller.msg import BaseControllerAction, BaseControllerFeedback, BaseControllerResult
from threading import Lock

from mujoco_engine.core_engine.wrapper.core import MjData


class StatePublisherMujoco(object):

    def __init__(self):
        self.pub_links = rospy.Publisher('/mujoco/link_states',LinkStates,queue_size=1)
        self.pub_joints = rospy.Publisher('/mujoco/joint_states',JointState,queue_size=1)

        self.jointlist =   ['bhand/f1/prox', 'bhand/f1/med', 'bhand/f1/dist', 'bhand/f2/prox', 'bhand/f2/med', 
                            'bhand/f2/dist', 'bhand/f3/med', 'bhand/f3/dist', 'smt/orie/z', 'smt/pose/x', 'smt/pose/y', 
                            'smt/whl_LF', 'smt/whl_LR', 'smt/whl_RF', 'smt/whl_RR', 'smt/world_x', 'smt/world_y', 'smt/world_z', 
                            'wagon/LF', 'wagon/LF/whl', 'wagon/LR/whl', 'wagon/RF', 'wagon/RF/whl', 'wagon/RR/whl', 'wagon/handle', 
                            'wagon/o/z', 'wagon/p/x', 'wagon/p/y', 'wagon/world_x', 'wagon/world_y', 'wagon/world_z', 'wam/J1', 
                            'wam/J2', 'wam/J3', 'wam/J4', 'wam/J5', 'wam/J6', 'wam/J7']
        
        self.linklist =  ['smt/base_link', 'smt/whl/LF_link', 'smt/whl/LR_link', 'smt/whl/RF_link', 'smt/whl/RR_link', 
                          'utility/wagon', 'wagon', 'wagon/LF', 'wagon/LF/whl', 'wagon/LR', 'wagon/LR/whl', 'wagon/RF', 
                          'wagon/RF/whl', 'wagon/RR', 'wagon/RR/whl', 'wagon/handle', 'wagon/pocket', 'wagon/wire_frame', 
                          'wam/base', 'wam/base_link', 'wam/bhand', 'wam/bhand/bhand_palm_link', 'wam/bhand/finger_1/dist_link', 
                          'wam/bhand/finger_1/med_link', 'wam/bhand/finger_1/prox_link', 'wam/bhand/finger_2/dist_link', 
                          'wam/bhand/finger_2/med_link', 'wam/bhand/finger_2/prox_link', 'wam/bhand/finger_3/dist_link', 
                          'wam/bhand/finger_3/med_link', 'wam/forearm_link', 'wam/sensor/intel', 'wam/sensor/zed', 
                          'wam/shoulder_pitch_link', 'wam/shoulder_yaw_link', 'wam/torque_sensor_link', 'wam/upper_arm_link', 
                          'wam/wrist_palm_link', 'wam/wrist_pitch_link', 'wam/wrist_yaw_link', 'wam_7dof_bhand', 'waterloo_steel', 'world']
        
        rospy.sleep(0.5)

    # Publish states joints: relative to initial state
    def pub_joint_states(self, MujData):
        self.joint_state = JointState()

        self.model, self.data = MujData.__getstate__()

        for name in self.jointlist:
            # print(name+" = "+str(self.data.joint(name).qpos[0]))
            
            # Maybe needs to be  updated since this is relative to initial position, use MjModel
            pos = self.data.joint(name).qpos[0]
            vel = self.data.joint(name).qvel[0]
            eff = self.data.joint(name).qfrc_actuator[0]

            self.joint_state.name.append(name)
            self.joint_state.position.append(pos)
            self.joint_state.velocity.append(vel)
            self.joint_state.effort.append(eff)
            
        self.joint_state.header.stamp = rospy.Time.now()

        self.pub_joints.publish(self.joint_state)



    # Publish states joints WAM: relative to initial state
    def pub_link_states(self, MujData):
        self.link_states = LinkStates()
        pos = Pose()
        orient = Quaternion()
        vel = Twist()

        self.model, self.data = MujData.__getstate__()

        for name in self.linklist:
            pos.position.x = self.data.body(name).xpos[0]
            pos.position.y = self.data.body(name).xpos[1]
            pos.position.z = self.data.body(name).xpos[2]

            orient.x = self.data.body(name).xquat[0]
            orient.y = self.data.body(name).xquat[1]
            orient.z = self.data.body(name).xquat[2]
            orient.w = self.data.body(name).xquat[3]

            pos.orientation = orient

            # Velocity based on COM!! Not the body frame!!
            vel.angular.x = self.data.body(name).cvel[0]
            vel.angular.y = self.data.body(name).cvel[1]
            vel.angular.z = self.data.body(name).cvel[2]
            vel.linear.x = self.data.body(name).cvel[3]
            vel.linear.y = self.data.body(name).cvel[4]
            vel.linear.z = self.data.body(name).cvel[5]

            self.link_states.name.append(name)
            self.link_states.pose.append(pos)
            self.link_states.twist.append(vel)

        self.pub_links.publish(self.link_states)

