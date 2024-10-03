#! /usr/bin/env python

# Publishers for publishing simulation states (link_states and joint_states) from MuJoCo to relevant ROS topics
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_inverse, quaternion_multiply


class StatePublisherMujoco(object):

    def __init__(self, Mujdata, Mujmodel, robot_list):

        # Create publishers to publish mujoco link_states and joint_states
        self.pub_links = rospy.Publisher('/mujoco/link_states',LinkStates,queue_size=1)
        self.pub_joints = rospy.Publisher('/mujoco/joint_states',JointState,queue_size=1)
        # Create publisher to publish mujoco sensor states
        # https://www.roboti.us/forum/index.php?threads/reading-sensor-values.3972/
        # Appropriate message-type in ROS
        # http://docs.ros.org/en/jade/api/gazebo_plugins/html/group__GazeboRosFTSensor.html
        self.pub_sensors = rospy.Publisher('/mujoco/sensor_states',WrenchStamped,queue_size=1)
        
        # Use pointer to read data
        self.data = Mujdata
        self.model = Mujmodel

        # Define lists to publish
        # These link and joint names are from the robot-kinematic files that are read by MuJoCo.
        # In order to move them using ROS, the corresponding joints of the corresponding robots' urdf must have the same names.
        summit_wam_joint_list = ['smt/orie/z', 'smt/pose/x', 'smt/pose/y', 'smt/whl_LF', 'smt/whl_LR', 'smt/whl_RF', 'smt/whl_RR', 
                                 'smt/world_x', 'smt/world_y', 'smt/world_z',
                                 'wam/J1','wam/J2', 'wam/J3', 'wam/J4', 'wam/J5', 'wam/J6', 'wam/J7',
                                 'bhand/f1/prox', 'bhand/f1/med', 'bhand/f1/dist', 'bhand/f2/prox', 'bhand/f2/med', 
                                 'bhand/f2/dist', 'bhand/f3/med', 'bhand/f3/dist']
        wagon_joint_list = ['wagon/LF', 'wagon/LF/whl', 'wagon/LR/whl', 'wagon/RF', 'wagon/RF/whl', 'wagon/RR/whl', 'wagon/handle','wagon/handle2',
                            'wagon/slide/world_x', 'wagon/slide/world_y', 'wagon/slide/world_z', 'wagon/hinge/world_y', 'wagon/hinge/world_z']
        fetch_joint_list = ['fetch/orie/z', 'fetch/pose/x', 'fetch/pose/y','fetch/world_x', 'fetch/world_y', 'fetch/world_z',
                            'fetch/R_whl','fetch/L_whl','fetch/torso_lift',
                            'fetch/head_pan','fetch/head_tilt',
                            'fetch_arm/shoulder_pan','fetch_arm/shoulder_lift','fetch_arm/upper_arm_roll','fetch_arm/elbow_flex','fetch_arm/fore_arm_roll','fetch_arm/wrist_flex','fetch_arm/wrist_roll',
                            'fetch_hand/right_gripper_finger','fetch_hand/left_gripper_finger']
        forklift_joint_list = ['fork_lift/pose/x','fork_lift/pose/y','fork_lift/orie/z',
                               'fork_lift/world_x','fork_lift/world_y','fork_lift/world_z']
        list_of_joint_lists = [summit_wam_joint_list,
                               fetch_joint_list,
                               forklift_joint_list,
                               wagon_joint_list]
        self.jointlist = []
        # Concatenate joint-lists
        counter = 0
        for _bool in robot_list:
            if _bool:
                self.jointlist += list_of_joint_lists[counter]
            counter +=1
        
        summit_wam_link_list = ['smt/base_link', 'smt/whl/LF_link', 'smt/whl/LR_link', 'smt/whl/RF_link', 'smt/whl/RR_link',
                                'smt/front/camera','smt/rear/camera',
                                'wam/base', 'wam/base_link', 'wam/bhand', 'wam/bhand/bhand_palm_link', 'wam/bhand/finger_1/dist_link', 
                                'wam/bhand/finger_1/med_link', 'wam/bhand/finger_1/prox_link', 'wam/bhand/finger_2/dist_link', 
                                'wam/bhand/finger_2/med_link', 'wam/bhand/finger_2/prox_link', 'wam/bhand/finger_3/dist_link', 
                                'wam/bhand/finger_3/med_link', 'wam/forearm_link', 'wam/camera', #'wam/sensor/zed', 
                                'wam/shoulder_pitch_link', 'wam/shoulder_yaw_link', 'wam/torque_sensor_link', 'wam/upper_arm_link', 
                                'wam/wrist_palm_link', 'wam/wrist_pitch_link', 'wam/wrist_yaw_link', 'wam_7dof_bhand', 'waterloo_steel']
        world_link_list = ['world']
        wagon_link_list = ['utility/wagon', 'wagon', 'wagon/LF', 'wagon/LF/whl', 'wagon/LR', 'wagon/LR/whl', 'wagon/RF', 
                           'wagon/RF/whl', 'wagon/RR', 'wagon/RR/whl', 'wagon/handle', 'wagon/pocket', 'wagon/wire_frame']
        fetch_link_list = ['fetch/base_link','fetch/whl/L_link','fetch/whl/R_link','fetch/laser_link','fetch/torso_fixed_link',
                           'fetch/torso_lift_link','fetch/bellows_link','fetch/bellows_2_link',
                           'fetch/head_pan_link','fetch/head_tilt_link',
                           'fetch/shoulder_pan_link','fetch/shoulder_lift_link','fetch/upper_arm_roll_link','fetch/elbow_flex_link',
                           'fetch/fore_arm_roll_link','fetch/wrist_flex_link','fetch/wrist_roll_link','fetch/gripper_link',
                           'fetch_hand/right_gripper_finger_link','fetch_hand/left_gripper_finger_link']
        forklift_link_list = ['fork_lift/base_link']
        list_of_link_lists = [summit_wam_link_list,
                              fetch_link_list,
                              forklift_link_list,
                              wagon_link_list]
        self.linklist =  world_link_list
        # Concatenate link-lists
        counter = 0
        for _bool in robot_list:
            if _bool:
                self.linklist += list_of_link_lists[counter]
            counter +=1

        # Sensor list
        self.sensor_list = ['force_sensor','torque_sensor']

    # Publish joint states: relative to initial state (which is 0.0 for all joints)
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


    # Publish link states: relative to initial state, therefore, transform summit parent base_link, wagon parent utility/wagon, and forklift parent forklift/base_link
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
            
            elif name == 'fork_lift/base_link':
                id_new = self.model.name2id('fork_lift_1','body')
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

    # Publish sensor states: relative to initial state (which is 0.0 for all sensors)
    def pub_sensor_states(self):

        # Initialize force-torque sensor-state object
        self.force_torque_state_stamped = WrenchStamped()
        # Sensor data list-form
        sensor_data = []
        for sensor_name in self.sensor_list:
            # https://www.roboti.us/forum/index.php?threads/reading-sensor-values.3972/#post-5368
            id_new = self.model.name2id(sensor_name,'sensor')
            # Sensor index
            sensor_index = self.model.sensor_adr[id_new]
            # Sensor dimension
            sensor_dim = self.model.sensor_dim[id_new]
            # Append sensor data
            sensor_data.append(self.data.sensordata[sensor_index:(sensor_index+sensor_dim)])
        
        # print(sensor_data[0][0])
        self.force_torque_state_stamped.header.stamp = rospy.Time.now()
        self.force_torque_state_stamped.wrench.force.x = sensor_data[0][0]
        self.force_torque_state_stamped.wrench.force.y = sensor_data[0][1]
        self.force_torque_state_stamped.wrench.force.z = sensor_data[0][2]
        self.force_torque_state_stamped.wrench.torque.x = sensor_data[1][0]
        self.force_torque_state_stamped.wrench.torque.y = sensor_data[1][1]
        self.force_torque_state_stamped.wrench.torque.z = sensor_data[1][2]
        # Publish force_torque_state
        self.pub_sensors.publish(self.force_torque_state_stamped)