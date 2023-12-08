#! /usr/bin/env python

# Publishers for publishing simulation states (link_states and joint_states)
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_inverse, quaternion_multiply


class StatePublisherMujoco(object):

    def __init__(self, Mujdata, Mujmodel):

        # Create publishers to publish mujoco link_states and joint_states
        self.pub_links = rospy.Publisher('/mujoco/link_states',LinkStates,queue_size=1)
        self.pub_joints = rospy.Publisher('/mujoco/joint_states',JointState,queue_size=1)
        
        # Use pointer to read data
        self.data = Mujdata
        self.model = Mujmodel

        # Define lists to publish
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


    # Publish states joints: relative to initial state (which is 0.0 for all joints)
    def pub_joint_states(self):

        # Initialize joint_state object
        self.joint_state = JointState()

        for name in self.jointlist:
            
            # Copy data from mj_data into joint_state object
            pos = self.data.joint(name).qpos[0]
            vel = self.data.joint(name).qvel[0]
            eff = self.data.joint(name).qfrc_actuator[0]

            # Add them in array with joint_states
            self.joint_state.name.append(name)
            self.joint_state.position.append(pos)
            self.joint_state.velocity.append(vel)
            self.joint_state.effort.append(eff)
            
        self.joint_state.header.stamp = rospy.Time.now()

        # Publish joint_states
        self.pub_joints.publish(self.joint_state)


    # Publish states links WAM: relative to initial state, therefore, transform summit parent base_link and wagon parent utility/wagon
    def pub_link_states(self):

        # Initialize link_states object
        self.link_states = LinkStates()

        for name in self.linklist:
            pos = Pose()
            orient = Quaternion()
            original_orient = Quaternion()
            vel = Twist()

            pos.position.x = self.data.body(name).xpos[0]
            pos.position.y = self.data.body(name).xpos[1]
            pos.position.z = self.data.body(name).xpos[2]

            orient.x = self.data.body(name).xquat[1]
            orient.y = self.data.body(name).xquat[2]
            orient.z = self.data.body(name).xquat[3]
            orient.w = self.data.body(name).xquat[0]

            # Set original orientation of spawned bodies to 0 for wagon and base
            if name == 'smt/base_link':
                id_new = self.model.name2id('waterloo_steel','body')
                original_orient.x = self.model.body_quat[id_new][1]
                original_orient.y = self.model.body_quat[id_new][2]
                original_orient.z = self.model.body_quat[id_new][3]
                original_orient.w = self.model.body_quat[id_new][0]

                # Rotate summit base_link back to 0.0 degrees
                orig_quat = [original_orient.x,original_orient.y,original_orient.z,original_orient.w]
                orie_quat = [orient.x,orient.y,orient.z,orient.w]
                inv_quat = quaternion_inverse(orig_quat)
                new_quat = quaternion_multiply(orie_quat,inv_quat)
                new_orient = Quaternion()
                new_orient.x = new_quat[0]
                new_orient.y = new_quat[1]
                new_orient.z = new_quat[2]
                new_orient.w = new_quat[3]
                pos.orientation = new_orient

            elif name == 'utility/wagon':
                id_new = self.model.name2id('wagon','body')
                original_orient.x = self.model.body_quat[id_new][1]
                original_orient.y = self.model.body_quat[id_new][2]
                original_orient.z = self.model.body_quat[id_new][3]
                original_orient.w = self.model.body_quat[id_new][0]

                # Rotate wagon utility/wagon back to 0.0 degrees
                orig_quat = [original_orient.x,original_orient.y,original_orient.z,original_orient.w]
                orie_quat = [orient.x,orient.y,orient.z,orient.w]
                inv_quat = quaternion_inverse(orig_quat)
                new_quat = quaternion_multiply(orie_quat,inv_quat)
                new_orient = Quaternion()
                new_orient.x = new_quat[0]
                new_orient.y = new_quat[1]
                new_orient.z = new_quat[2]
                new_orient.w = new_quat[3]
                pos.orientation = new_orient
   
            else:
                pos.orientation = orient

            # Velocity based on COM
            vel.angular.x = self.data.body(name).cvel[0]
            vel.angular.y = self.data.body(name).cvel[1]
            vel.angular.z = self.data.body(name).cvel[2]
            vel.linear.x = self.data.body(name).cvel[3]
            vel.linear.y = self.data.body(name).cvel[4]
            vel.linear.z = self.data.body(name).cvel[5]

            # Add them in array with link_states
            self.link_states.name.append(name)
            self.link_states.pose.append(pos)
            self.link_states.twist.append(vel)

        # Publish link_states
        self.pub_links.publish(self.link_states)

