#! /usr/bin/env python

# Publishers for publishing simulation states (link_states and joint_states) from MuJoCo to relevant ROS topics
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from geometry_msgs.msg import Accel, Twist, Pose, Quaternion, WrenchStamped
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from uwarl_mujoco_ros_msgs.msg import FTcompensation, JointStateArray, LinkStateArray
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
        self.pub_ft_sensor = rospy.Publisher('/mujoco/ft_sensor_states',WrenchStamped,queue_size=1)
        self.pub_ft_comp = rospy.Publisher('/mujoco/ft_comp',FTcompensation,queue_size=1)
        self.pub_link_state_sensor = rospy.Publisher('/mujoco/link_state_sensor_states',LinkStateArray,queue_size=1)
        self.pub_joint_state_sensor = rospy.Publisher('/mujoco/joint_state_sensor_states',JointStateArray,queue_size=1)
        
        # Initialize counter
        self.counter = 0
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
        self.sensor_list = ['accelerometer_mb','velocimeter_mb','gyroscope_mb','global_pos_mb','global_quat_mb','joint_vel_mb_lf','joint_vel_mb_rf','joint_vel_mb_rr','joint_vel_mb_lr',
                            'accelerometer_wam_base','velocimeter_wam_base','gyroscope_wam_base','global_pos_wam_base','global_quat_wam_base',
                            'accelerometer_wam_shoulder_yaw','velocimeter_wam_shoulder_yaw','gyroscope_wam_shoulder_yaw','global_pos_wam_shoulder_yaw','global_quat_wam_shoulder_yaw','joint_pos_wam_shoulder_yaw','joint_vel_wam_shoulder_yaw','joint_effort_wam_shoulder_yaw',
                            'accelerometer_wam_shoulder_pitch','velocimeter_wam_shoulder_pitch','gyroscope_wam_shoulder_pitch','global_pos_wam_shoulder_pitch','global_quat_wam_shoulder_pitch','joint_pos_wam_shoulder_pitch','joint_vel_wam_shoulder_pitch','joint_effort_wam_shoulder_pitch',
                            'accelerometer_wam_upper_arm','velocimeter_wam_upper_arm','gyroscope_wam_upper_arm','global_pos_wam_upper_arm','global_quat_wam_upper_arm','joint_pos_wam_upper_arm','joint_vel_wam_upper_arm','joint_effort_wam_upper_arm',
                            'accelerometer_wam_forearm','velocimeter_wam_forearm','gyroscope_wam_forearm','global_pos_wam_forearm','global_quat_wam_forearm','joint_pos_wam_forearm','joint_vel_wam_forearm','joint_effort_wam_forearm',
                            'accelerometer_wam_wrist_yaw','velocimeter_wam_wrist_yaw','gyroscope_wam_wrist_yaw','global_pos_wam_wrist_yaw','global_quat_wam_wrist_yaw','joint_pos_wam_wrist_yaw','joint_vel_wam_wrist_yaw','joint_effort_wam_wrist_yaw',
                            'accelerometer_wam_wrist_pitch','velocimeter_wam_wrist_pitch','gyroscope_wam_wrist_pitch','global_pos_wam_wrist_pitch','global_quat_wam_wrist_pitch','joint_pos_wam_wrist_pitch','joint_vel_wam_wrist_pitch','joint_effort_wam_wrist_pitch',
                            'accelerometer_wam_wrist_palm','velocimeter_wam_wrist_palm','gyroscope_wam_wrist_palm','global_pos_wam_wrist_palm','global_quat_wam_wrist_palm','joint_pos_wam_wrist_palm','joint_vel_wam_wrist_palm','joint_effort_wam_wrist_palm',
                            'force_sensor','torque_sensor',
                            'accelerometer_bhand_palm','velocimeter_bhand_palm','gyroscope_bhand_palm','global_pos_bhand_palm','global_quat_bhand_palm',
                            'accelerometer_bhand_finger_1_prox','velocimeter_bhand_finger_1_prox','gyroscope_bhand_finger_1_prox','global_pos_bhand_finger_1_prox','global_quat_bhand_finger_1_prox','joint_pos_bhand_finger_1_prox','joint_vel_bhand_finger_1_prox','joint_effort_bhand_finger_1_prox',
                            'accelerometer_bhand_finger_1_med','velocimeter_bhand_finger_1_med','gyroscope_bhand_finger_1_med','global_pos_bhand_finger_1_med','global_quat_bhand_finger_1_med','joint_pos_bhand_finger_1_med','joint_vel_bhand_finger_1_med','joint_effort_bhand_finger_1_med',
                            'accelerometer_bhand_finger_1_dist','velocimeter_bhand_finger_1_dist','gyroscope_bhand_finger_1_dist','global_pos_bhand_finger_1_dist','global_quat_bhand_finger_1_dist','joint_pos_bhand_finger_1_dist','joint_vel_bhand_finger_1_dist','joint_effort_bhand_finger_1_dist',
                            'accelerometer_bhand_finger_2_prox','velocimeter_bhand_finger_2_prox','gyroscope_bhand_finger_2_prox','global_pos_bhand_finger_2_prox','global_quat_bhand_finger_2_prox','joint_pos_bhand_finger_2_prox','joint_vel_bhand_finger_2_prox','joint_effort_bhand_finger_2_prox',
                            'accelerometer_bhand_finger_2_med','velocimeter_bhand_finger_2_med','gyroscope_bhand_finger_2_med','global_pos_bhand_finger_2_med','global_quat_bhand_finger_2_med','joint_pos_bhand_finger_2_med','joint_vel_bhand_finger_2_med','joint_effort_bhand_finger_2_med',
                            'accelerometer_bhand_finger_2_dist','velocimeter_bhand_finger_2_dist','gyroscope_bhand_finger_2_dist','global_pos_bhand_finger_2_dist','global_quat_bhand_finger_2_dist','joint_pos_bhand_finger_2_dist','joint_vel_bhand_finger_2_dist','joint_effort_bhand_finger_2_dist',
                            'accelerometer_bhand_finger_3_med','velocimeter_bhand_finger_3_med','gyroscope_bhand_finger_3_med','global_pos_bhand_finger_3_med','global_quat_bhand_finger_3_med','joint_pos_bhand_finger_3_med','joint_vel_bhand_finger_3_med','joint_effort_bhand_finger_3_med',
                            'accelerometer_bhand_finger_3_dist','velocimeter_bhand_finger_3_dist','gyroscope_bhand_finger_3_dist','global_pos_bhand_finger_3_dist','global_quat_bhand_finger_3_dist','joint_pos_bhand_finger_3_dist','joint_vel_bhand_finger_3_dist','joint_effort_bhand_finger_3_dist',
                            'accelerometer_1_cart','accelerometer_2_cart','velocimeter_cart','gyroscope_cart','global_pos_cart','global_quat_cart']

        # MB effort
        self.mb_effort = ['smt/pose/x','smt/pose/y','smt/orie/z']
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
            # if name == 'smt/base_link':
            #     id_new = self.model.name2id('waterloo_steel','body')
            #     original_orient.x = self.model.body_quat[id_new][1]
            #     original_orient.y = self.model.body_quat[id_new][2]
            #     original_orient.z = self.model.body_quat[id_new][3]
            #     original_orient.w = self.model.body_quat[id_new][0]

            #     # Rotate summit base_link back to 0.0 degrees
            #     orig_quat = [original_orient.x,original_orient.y,original_orient.z,original_orient.w]
            #     orie_quat = [orient.x,orient.y,orient.z,orient.w]
            #     inv_quat = quaternion_inverse(orig_quat)
            #     new_quat = quaternion_multiply(orie_quat,inv_quat)
            #     new_orient = Quaternion()
            #     new_orient.x = new_quat[0]
            #     new_orient.y = new_quat[1]
            #     new_orient.z = new_quat[2]
            #     new_orient.w = new_quat[3]
            #     pos.orientation = new_orient

            # elif name == 'utility/wagon':
            #     id_new = self.model.name2id('wagon','body')
            #     original_orient.x = self.model.body_quat[id_new][1]
            #     original_orient.y = self.model.body_quat[id_new][2]
            #     original_orient.z = self.model.body_quat[id_new][3]
            #     original_orient.w = self.model.body_quat[id_new][0]

            #     # Rotate wagon utility/wagon back to 0.0 degrees
            #     orig_quat = [original_orient.x,original_orient.y,original_orient.z,original_orient.w]
            #     orie_quat = [orient.x,orient.y,orient.z,orient.w]
            #     inv_quat = quaternion_inverse(orig_quat)
            #     new_quat = quaternion_multiply(orie_quat,inv_quat)
            #     new_orient = Quaternion()
            #     new_orient.x = new_quat[0]
            #     new_orient.y = new_quat[1]
            #     new_orient.z = new_quat[2]
            #     new_orient.w = new_quat[3]
            #     pos.orientation = new_orient
            
            # elif name == 'fork_lift/base_link':
            #     id_new = self.model.name2id('fork_lift_1','body')
            #     original_orient.x = self.model.body_quat[id_new][1]
            #     original_orient.y = self.model.body_quat[id_new][2]
            #     original_orient.z = self.model.body_quat[id_new][3]
            #     original_orient.w = self.model.body_quat[id_new][0]

            #     # Rotate wagon utility/wagon back to 0.0 degrees
            #     orig_quat = [original_orient.x,original_orient.y,original_orient.z,original_orient.w]
            #     orie_quat = [orient.x,orient.y,orient.z,orient.w]
            #     inv_quat = quaternion_inverse(orig_quat)
            #     new_quat = quaternion_multiply(orie_quat,inv_quat)
            #     new_orient = Quaternion()
            #     new_orient.x = new_quat[0]
            #     new_orient.y = new_quat[1]
            #     new_orient.z = new_quat[2]
            #     new_orient.w = new_quat[3]
            #     pos.orientation = new_orient
   
            # else:
            pos.orientation = orient

            # Velocity based on COM
            vel.angular.x = self.data.body(name).cvel[0]
            vel.angular.y = self.data.body(name).cvel[1]
            vel.angular.z = self.data.body(name).cvel[2]
            vel.linear.x = self.data.body(name).cvel[3]
            vel.linear.y = self.data.body(name).cvel[4]
            vel.linear.z = self.data.body(name).cvel[5]

            # if name == 'smt/base_link':
            #     print("MB yaw-rate error")
            #     print(self.data.body(name).cvel[2]-self.data.joint('smt/orie/z').qvel[0])
            # if name == 'wam/shoulder_yaw_link':
            #     print('Shoulder yaw yaw-rate error')
            #     print(self.data.body(name).cvel[2]-self.data.joint('smt/orie/z').qvel[0]-self.data.joint('wam/J1').qvel[0])

            # Add them in array with link_states
            self.link_states.name.append(name)
            self.link_states.pose.append(pos)
            self.link_states.twist.append(vel)

        # Publish link_states
        self.pub_links.publish(self.link_states)

    # Publish sensor states: relative to initial state (which is 0.0 for all sensors)
    def pub_sensor_states(self):

        # Initialize sensor objects:
        # Link States
        link_state_stamped = LinkStateArray()
        # Joint States
        joint_state_stamped = JointStateArray()
        # Force-torque sensor-state object
        force_torque_state_stamped = WrenchStamped()
        # Force-Torque compensation message collection
        force_torque_comp_stamped = FTcompensation()
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
        
        # List of actuator commands
        act_eff_data = []
        # Actuator effort command
        for act_name in self.mb_effort:
            act_eff_data.append(self.data.actuator(act_name).ctrl)
        
        # Current time
        curr_time = rospy.Time.now()
        link_state_stamped.header.stamp = curr_time
        joint_state_stamped.header.stamp = curr_time
        force_torque_state_stamped.header.stamp = curr_time
        force_torque_comp_stamped.header.stamp = curr_time
        link_state_stamped.header.seq = self.counter
        joint_state_stamped.header.seq = self.counter
        force_torque_state_stamped.header.seq = self.counter
        force_torque_comp_stamped.header.seq = self.counter

        # Mobile-base
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[0][0]
        temp_accel.linear.y = sensor_data[0][1]
        temp_accel.linear.z = sensor_data[0][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = self.data.joint("smt/orie/z").qacc[0]
        link_state_stamped.accel.append(temp_accel)
        force_torque_comp_stamped.mb_accel = temp_accel
        # Link name
        link_state_stamped.name.append("smt/base_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[1][0]
        temp_twist.linear.y = sensor_data[1][1]
        temp_twist.linear.z = sensor_data[1][2]
        # Angular component
        temp_twist.angular.x = sensor_data[2][0]
        temp_twist.angular.y = sensor_data[2][1]
        temp_twist.angular.z = sensor_data[2][2]
        link_state_stamped.twist.append(temp_twist)
        force_torque_comp_stamped.mb_twist = temp_twist
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[3][0]
        temp_pose.position.y = sensor_data[3][1]
        temp_pose.position.z = sensor_data[3][2]

        temp_pose.orientation.w = sensor_data[4][0]
        temp_pose.orientation.x = sensor_data[4][1]
        temp_pose.orientation.y = sensor_data[4][2]
        temp_pose.orientation.z = sensor_data[4][3]
        link_state_stamped.pose.append(temp_pose)
        force_torque_comp_stamped.mb_pose = temp_pose

        # Joint state
        joint_state_stamped.name.append("smt/pose/x")
        joint_state_stamped.position.append(0)
        joint_state_stamped.velocity.append(0)
        joint_state_stamped.acceleration.append(0)
        joint_state_stamped.effort.append(act_eff_data[0])

        force_torque_comp_stamped.mb_effort.append(act_eff_data[0])

        joint_state_stamped.name.append("smt/pose/y")
        joint_state_stamped.position.append(0)
        joint_state_stamped.velocity.append(0)
        joint_state_stamped.acceleration.append(0)
        joint_state_stamped.effort.append(act_eff_data[1])

        force_torque_comp_stamped.mb_effort.append(act_eff_data[1])

        joint_state_stamped.name.append("smt/orie/z")
        joint_state_stamped.position.append(0)
        joint_state_stamped.velocity.append(0)
        joint_state_stamped.acceleration.append(0)
        joint_state_stamped.effort.append(act_eff_data[2])

        force_torque_comp_stamped.mb_effort.append(act_eff_data[2])

        # WAM
        # Base Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[9][0]
        temp_accel.linear.y = sensor_data[9][1]
        temp_accel.linear.z = sensor_data[9][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = self.data.joint("smt/orie/z").qacc[0]
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/base_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[10][0]
        temp_twist.linear.y = sensor_data[10][1]
        temp_twist.linear.z = sensor_data[10][2]
        # Angular component
        temp_twist.angular.x = sensor_data[11][0]
        temp_twist.angular.y = sensor_data[11][1]
        temp_twist.angular.z = sensor_data[11][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[12][0]
        temp_pose.position.y = sensor_data[12][1]
        temp_pose.position.z = sensor_data[12][2]
        temp_pose.orientation.w = sensor_data[13][0]
        temp_pose.orientation.x = sensor_data[13][1]
        temp_pose.orientation.y = sensor_data[13][2]
        temp_pose.orientation.z = sensor_data[13][3]
        link_state_stamped.pose.append(temp_pose)

        # Shoulder Yaw Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[14][0]
        temp_accel.linear.y = sensor_data[14][1]
        temp_accel.linear.z = sensor_data[14][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = self.data.joint("wam/J1").qacc[0]
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/shoulder_yaw_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[15][0]
        temp_twist.linear.y = sensor_data[15][1]
        temp_twist.linear.z = sensor_data[15][2]
        # Angular component
        temp_twist.angular.x = sensor_data[16][0]
        temp_twist.angular.y = sensor_data[16][1]
        temp_twist.angular.z = sensor_data[16][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[17][0]
        temp_pose.position.y = sensor_data[17][1]
        temp_pose.position.z = sensor_data[17][2]
        temp_pose.orientation.w = sensor_data[18][0]
        temp_pose.orientation.x = sensor_data[18][1]
        temp_pose.orientation.y = sensor_data[18][2]
        temp_pose.orientation.z = sensor_data[18][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J1")
        joint_state_stamped.position.append(sensor_data[19][0])
        joint_state_stamped.velocity.append(sensor_data[20][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J1").qacc[0])
        joint_state_stamped.effort.append(sensor_data[21][0])

        force_torque_comp_stamped.joint_name.append("wam/J1")
        force_torque_comp_stamped.joint_position.append(sensor_data[19][0])
        force_torque_comp_stamped.joint_velocity.append(sensor_data[20][0])
        force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J1").qacc[0])
        force_torque_comp_stamped.joint_effort.append(sensor_data[21][0])

        # Shoulder Pitch Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[22][0]
        temp_accel.linear.y = sensor_data[22][1]
        temp_accel.linear.z = sensor_data[22][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/shoulder_pitch_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[23][0]
        temp_twist.linear.y = sensor_data[23][1]
        temp_twist.linear.z = sensor_data[23][2]
        # Angular component
        temp_twist.angular.x = sensor_data[24][0]
        temp_twist.angular.y = sensor_data[24][1]
        temp_twist.angular.z = sensor_data[24][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[25][0]
        temp_pose.position.y = sensor_data[25][1]
        temp_pose.position.z = sensor_data[25][2]
        temp_pose.orientation.w = sensor_data[26][0]
        temp_pose.orientation.x = sensor_data[26][1]
        temp_pose.orientation.y = sensor_data[26][2]
        temp_pose.orientation.z = sensor_data[26][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J2")
        joint_state_stamped.position.append(sensor_data[27][0])
        joint_state_stamped.velocity.append(sensor_data[28][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J2").qacc[0])
        joint_state_stamped.effort.append(sensor_data[29][0])

        force_torque_comp_stamped.joint_name.append("wam/J2")
        force_torque_comp_stamped.joint_position.append(sensor_data[27][0])
        force_torque_comp_stamped.joint_velocity.append(sensor_data[28][0])
        force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J2").qacc[0])
        force_torque_comp_stamped.joint_effort.append(sensor_data[29][0])

        # Upper Arm Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[30][0]
        temp_accel.linear.y = sensor_data[30][1]
        temp_accel.linear.z = sensor_data[30][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/upper_arm_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[31][0]
        temp_twist.linear.y = sensor_data[31][1]
        temp_twist.linear.z = sensor_data[31][2]
        # Angular component
        temp_twist.angular.x = sensor_data[32][0]
        temp_twist.angular.y = sensor_data[32][1]
        temp_twist.angular.z = sensor_data[32][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[33][0]
        temp_pose.position.y = sensor_data[33][1]
        temp_pose.position.z = sensor_data[33][2]
        temp_pose.orientation.w = sensor_data[34][0]
        temp_pose.orientation.x = sensor_data[34][1]
        temp_pose.orientation.y = sensor_data[34][2]
        temp_pose.orientation.z = sensor_data[34][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J3")
        joint_state_stamped.position.append(sensor_data[35][0])
        joint_state_stamped.velocity.append(sensor_data[36][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J3").qacc[0])
        joint_state_stamped.effort.append(sensor_data[37][0])

        force_torque_comp_stamped.joint_name.append("wam/J3")
        force_torque_comp_stamped.joint_position.append(sensor_data[35][0])
        force_torque_comp_stamped.joint_velocity.append(sensor_data[36][0])
        force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J3").qacc[0])
        force_torque_comp_stamped.joint_effort.append(sensor_data[37][0])

        # Forearm Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[38][0]
        temp_accel.linear.y = sensor_data[38][1]
        temp_accel.linear.z = sensor_data[38][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/forearm_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[39][0]
        temp_twist.linear.y = sensor_data[39][1]
        temp_twist.linear.z = sensor_data[39][2]
        # Angular component
        temp_twist.angular.x = sensor_data[40][0]
        temp_twist.angular.y = sensor_data[40][1]
        temp_twist.angular.z = sensor_data[40][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[41][0]
        temp_pose.position.y = sensor_data[41][1]
        temp_pose.position.z = sensor_data[41][2]
        temp_pose.orientation.w = sensor_data[42][0]
        temp_pose.orientation.x = sensor_data[42][1]
        temp_pose.orientation.y = sensor_data[42][2]
        temp_pose.orientation.z = sensor_data[42][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J4")
        joint_state_stamped.position.append(sensor_data[43][0])
        joint_state_stamped.velocity.append(sensor_data[44][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J4").qacc[0])
        joint_state_stamped.effort.append(sensor_data[45][0])

        force_torque_comp_stamped.joint_name.append("wam/J4")
        force_torque_comp_stamped.joint_position.append(sensor_data[43][0])
        force_torque_comp_stamped.joint_velocity.append(sensor_data[44][0])
        force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J4").qacc[0])
        force_torque_comp_stamped.joint_effort.append(sensor_data[45][0])

        # Wrist Yaw Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[46][0]
        temp_accel.linear.y = sensor_data[46][1]
        temp_accel.linear.z = sensor_data[46][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/wrist_yaw_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[47][0]
        temp_twist.linear.y = sensor_data[47][1]
        temp_twist.linear.z = sensor_data[47][2]
        # Angular component
        temp_twist.angular.x = sensor_data[48][0]
        temp_twist.angular.y = sensor_data[48][1]
        temp_twist.angular.z = sensor_data[48][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[49][0]
        temp_pose.position.y = sensor_data[49][1]
        temp_pose.position.z = sensor_data[49][2]
        temp_pose.orientation.w = sensor_data[50][0]
        temp_pose.orientation.x = sensor_data[50][1]
        temp_pose.orientation.y = sensor_data[50][2]
        temp_pose.orientation.z = sensor_data[50][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J5")
        joint_state_stamped.position.append(sensor_data[51][0])
        joint_state_stamped.velocity.append(sensor_data[52][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J5").qacc[0])
        joint_state_stamped.effort.append(sensor_data[53][0])

        force_torque_comp_stamped.joint_name.append("wam/J5")
        force_torque_comp_stamped.joint_position.append(sensor_data[51][0])
        force_torque_comp_stamped.joint_velocity.append(sensor_data[52][0])
        force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J5").qacc[0])
        force_torque_comp_stamped.joint_effort.append(sensor_data[53][0])

        # Wrist Pitch Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[54][0]
        temp_accel.linear.y = sensor_data[54][1]
        temp_accel.linear.z = sensor_data[54][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/wrist_pitch_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[55][0]
        temp_twist.linear.y = sensor_data[55][1]
        temp_twist.linear.z = sensor_data[55][2]
        # Angular component
        temp_twist.angular.x = sensor_data[56][0]
        temp_twist.angular.y = sensor_data[56][1]
        temp_twist.angular.z = sensor_data[56][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[57][0]
        temp_pose.position.y = sensor_data[57][1]
        temp_pose.position.z = sensor_data[57][2]
        temp_pose.orientation.w = sensor_data[58][0]
        temp_pose.orientation.x = sensor_data[58][1]
        temp_pose.orientation.y = sensor_data[58][2]
        temp_pose.orientation.z = sensor_data[58][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J6")
        joint_state_stamped.position.append(sensor_data[59][0])
        joint_state_stamped.velocity.append(sensor_data[60][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J6").qacc[0])
        joint_state_stamped.effort.append(sensor_data[61][0])

        force_torque_comp_stamped.joint_name.append("wam/J6")
        force_torque_comp_stamped.joint_position.append(sensor_data[59][0])
        force_torque_comp_stamped.joint_velocity.append(sensor_data[60][0])
        force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J6").qacc[0])
        force_torque_comp_stamped.joint_effort.append(sensor_data[61][0])

        # Wrist Palm Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[62][0]
        temp_accel.linear.y = sensor_data[62][1]
        temp_accel.linear.z = sensor_data[62][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/wrist_palm_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[63][0]
        temp_twist.linear.y = sensor_data[63][1]
        temp_twist.linear.z = sensor_data[63][2]
        # Angular component
        temp_twist.angular.x = sensor_data[64][0]
        temp_twist.angular.y = sensor_data[64][1]
        temp_twist.angular.z = sensor_data[64][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[65][0]
        temp_pose.position.y = sensor_data[65][1]
        temp_pose.position.z = sensor_data[65][2]
        temp_pose.orientation.w = sensor_data[66][0]
        temp_pose.orientation.x = sensor_data[66][1]
        temp_pose.orientation.y = sensor_data[66][2]
        temp_pose.orientation.z = sensor_data[66][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J7")
        joint_state_stamped.position.append(sensor_data[67][0])
        joint_state_stamped.velocity.append(sensor_data[68][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J7").qacc[0])
        joint_state_stamped.effort.append(sensor_data[69][0])

        force_torque_comp_stamped.joint_name.append("wam/J7")
        force_torque_comp_stamped.joint_position.append(sensor_data[67][0])
        force_torque_comp_stamped.joint_velocity.append(sensor_data[68][0])
        force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J7").qacc[0])
        force_torque_comp_stamped.joint_effort.append(sensor_data[69][0])

        # Force Torque Sensor
        force_torque_state_stamped.wrench.force.x = sensor_data[70][0]
        force_torque_state_stamped.wrench.force.y = sensor_data[70][1]
        force_torque_state_stamped.wrench.force.z = sensor_data[70][2]
        force_torque_state_stamped.wrench.torque.x = sensor_data[71][0]
        force_torque_state_stamped.wrench.torque.y = sensor_data[71][1]
        force_torque_state_stamped.wrench.torque.z = sensor_data[71][2]

        force_torque_comp_stamped.ft_sensor_wrench.force.x = sensor_data[70][0]
        force_torque_comp_stamped.ft_sensor_wrench.force.y = sensor_data[70][1]
        force_torque_comp_stamped.ft_sensor_wrench.force.z = sensor_data[70][2]
        force_torque_comp_stamped.ft_sensor_wrench.torque.x = sensor_data[71][0]
        force_torque_comp_stamped.ft_sensor_wrench.torque.y = sensor_data[71][1]
        force_torque_comp_stamped.ft_sensor_wrench.torque.z = sensor_data[71][2]

        # BHand Palm Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[72][0]
        temp_accel.linear.y = sensor_data[72][1]
        temp_accel.linear.z = sensor_data[72][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/bhand_palm_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[73][0]
        temp_twist.linear.y = sensor_data[73][1]
        temp_twist.linear.z = sensor_data[73][2]
        # Angular component
        temp_twist.angular.x = sensor_data[74][0]
        temp_twist.angular.y = sensor_data[74][1]
        temp_twist.angular.z = sensor_data[74][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[75][0]
        temp_pose.position.y = sensor_data[75][1]
        temp_pose.position.z = sensor_data[75][2]
        temp_pose.orientation.w = sensor_data[76][0]
        temp_pose.orientation.x = sensor_data[76][1]
        temp_pose.orientation.y = sensor_data[76][2]
        temp_pose.orientation.z = sensor_data[76][3]
        link_state_stamped.pose.append(temp_pose)

        # BHand Finger 1 Prox Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[77][0]
        temp_accel.linear.y = sensor_data[77][1]
        temp_accel.linear.z = sensor_data[77][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_1/prox_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[78][0]
        temp_twist.linear.y = sensor_data[78][1]
        temp_twist.linear.z = sensor_data[78][2]
        # Angular component
        temp_twist.angular.x = sensor_data[79][0]
        temp_twist.angular.y = sensor_data[79][1]
        temp_twist.angular.z = sensor_data[79][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[80][0]
        temp_pose.position.y = sensor_data[80][1]
        temp_pose.position.z = sensor_data[80][2]
        temp_pose.orientation.w = sensor_data[81][0]
        temp_pose.orientation.x = sensor_data[81][1]
        temp_pose.orientation.y = sensor_data[81][2]
        temp_pose.orientation.z = sensor_data[81][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f1/prox")
        joint_state_stamped.position.append(sensor_data[82][0])
        joint_state_stamped.velocity.append(sensor_data[83][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f1/prox").qacc[0])
        joint_state_stamped.effort.append(sensor_data[84][0])

        # BHand Finger 1 Med Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[85][0]
        temp_accel.linear.y = sensor_data[85][1]
        temp_accel.linear.z = sensor_data[85][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_1/med_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[86][0]
        temp_twist.linear.y = sensor_data[86][1]
        temp_twist.linear.z = sensor_data[86][2]
        # Angular component
        temp_twist.angular.x = sensor_data[87][0]
        temp_twist.angular.y = sensor_data[87][1]
        temp_twist.angular.z = sensor_data[87][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[88][0]
        temp_pose.position.y = sensor_data[88][1]
        temp_pose.position.z = sensor_data[88][2]
        temp_pose.orientation.w = sensor_data[89][0]
        temp_pose.orientation.x = sensor_data[89][1]
        temp_pose.orientation.y = sensor_data[89][2]
        temp_pose.orientation.z = sensor_data[89][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f1/med")
        joint_state_stamped.position.append(sensor_data[90][0])
        joint_state_stamped.velocity.append(sensor_data[91][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f1/med").qacc[0])
        joint_state_stamped.effort.append(sensor_data[92][0])

        # BHand Finger 1 Dist Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[93][0]
        temp_accel.linear.y = sensor_data[93][1]
        temp_accel.linear.z = sensor_data[93][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_1/dist_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[94][0]
        temp_twist.linear.y = sensor_data[94][1]
        temp_twist.linear.z = sensor_data[94][2]
        # Angular component
        temp_twist.angular.x = sensor_data[95][0]
        temp_twist.angular.y = sensor_data[95][1]
        temp_twist.angular.z = sensor_data[95][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[96][0]
        temp_pose.position.y = sensor_data[96][1]
        temp_pose.position.z = sensor_data[96][2]
        temp_pose.orientation.w = sensor_data[97][0]
        temp_pose.orientation.x = sensor_data[97][1]
        temp_pose.orientation.y = sensor_data[97][2]
        temp_pose.orientation.z = sensor_data[97][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f1/dist")
        joint_state_stamped.position.append(sensor_data[98][0])
        joint_state_stamped.velocity.append(sensor_data[99][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f1/dist").qacc[0])
        joint_state_stamped.effort.append(sensor_data[100][0])

        # BHand Finger 2 Prox Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[101][0]
        temp_accel.linear.y = sensor_data[101][1]
        temp_accel.linear.z = sensor_data[101][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_2/prox_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[102][0]
        temp_twist.linear.y = sensor_data[102][1]
        temp_twist.linear.z = sensor_data[102][2]
        # Angular component
        temp_twist.angular.x = sensor_data[103][0]
        temp_twist.angular.y = sensor_data[103][1]
        temp_twist.angular.z = sensor_data[103][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[104][0]
        temp_pose.position.y = sensor_data[104][1]
        temp_pose.position.z = sensor_data[104][2]
        temp_pose.orientation.w = sensor_data[105][0]
        temp_pose.orientation.x = sensor_data[105][1]
        temp_pose.orientation.y = sensor_data[105][2]
        temp_pose.orientation.z = sensor_data[105][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f2/prox")
        joint_state_stamped.position.append(sensor_data[106][0])
        joint_state_stamped.velocity.append(sensor_data[107][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f2/prox").qacc[0])
        joint_state_stamped.effort.append(sensor_data[108][0])

        # BHand Finger 2 Med Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[109][0]
        temp_accel.linear.y = sensor_data[109][1]
        temp_accel.linear.z = sensor_data[109][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_2/med_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[110][0]
        temp_twist.linear.y = sensor_data[110][1]
        temp_twist.linear.z = sensor_data[110][2]
        # Angular component
        temp_twist.angular.x = sensor_data[111][0]
        temp_twist.angular.y = sensor_data[111][1]
        temp_twist.angular.z = sensor_data[111][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[112][0]
        temp_pose.position.y = sensor_data[112][1]
        temp_pose.position.z = sensor_data[112][2]
        temp_pose.orientation.w = sensor_data[113][0]
        temp_pose.orientation.x = sensor_data[113][1]
        temp_pose.orientation.y = sensor_data[113][2]
        temp_pose.orientation.z = sensor_data[113][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f2/med")
        joint_state_stamped.position.append(sensor_data[114][0])
        joint_state_stamped.velocity.append(sensor_data[115][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f2/med").qacc[0])
        joint_state_stamped.effort.append(sensor_data[116][0])

        # BHand Finger 2 Dist Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[117][0]
        temp_accel.linear.y = sensor_data[117][1]
        temp_accel.linear.z = sensor_data[117][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_2/dist_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[118][0]
        temp_twist.linear.y = sensor_data[118][1]
        temp_twist.linear.z = sensor_data[118][2]
        # Angular component
        temp_twist.angular.x = sensor_data[119][0]
        temp_twist.angular.y = sensor_data[119][1]
        temp_twist.angular.z = sensor_data[119][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[120][0]
        temp_pose.position.y = sensor_data[120][1]
        temp_pose.position.z = sensor_data[120][2]
        temp_pose.orientation.w = sensor_data[121][0]
        temp_pose.orientation.x = sensor_data[121][1]
        temp_pose.orientation.y = sensor_data[121][2]
        temp_pose.orientation.z = sensor_data[121][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f2/dist")
        joint_state_stamped.position.append(sensor_data[122][0])
        joint_state_stamped.velocity.append(sensor_data[123][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f2/dist").qacc[0])
        joint_state_stamped.effort.append(sensor_data[124][0])

        # BHand Finger 3 Med Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[125][0]
        temp_accel.linear.y = sensor_data[125][1]
        temp_accel.linear.z = sensor_data[125][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_3/med_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[126][0]
        temp_twist.linear.y = sensor_data[126][1]
        temp_twist.linear.z = sensor_data[126][2]
        # Angular component
        temp_twist.angular.x = sensor_data[127][0]
        temp_twist.angular.y = sensor_data[127][1]
        temp_twist.angular.z = sensor_data[127][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[128][0]
        temp_pose.position.y = sensor_data[128][1]
        temp_pose.position.z = sensor_data[128][2]
        temp_pose.orientation.w = sensor_data[129][0]
        temp_pose.orientation.x = sensor_data[129][1]
        temp_pose.orientation.y = sensor_data[129][2]
        temp_pose.orientation.z = sensor_data[129][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f3/med")
        joint_state_stamped.position.append(sensor_data[130][0])
        joint_state_stamped.velocity.append(sensor_data[131][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f3/med").qacc[0])
        joint_state_stamped.effort.append(sensor_data[132][0])

        # BHand Finger 3 Dist Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[133][0]
        temp_accel.linear.y = sensor_data[133][1]
        temp_accel.linear.z = sensor_data[133][2]
        # Angular acceleration
        temp_accel.angular.x = 0
        temp_accel.angular.y = 0
        temp_accel.angular.z = 0
        link_state_stamped.accel.append(temp_accel)
        # Link name
        link_state_stamped.name.append("wam/bhand/finger_3/dist_link")
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[134][0]
        temp_twist.linear.y = sensor_data[134][1]
        temp_twist.linear.z = sensor_data[134][2]
        # Angular component
        temp_twist.angular.x = sensor_data[135][0]
        temp_twist.angular.y = sensor_data[135][1]
        temp_twist.angular.z = sensor_data[135][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[136][0]
        temp_pose.position.y = sensor_data[136][1]
        temp_pose.position.z = sensor_data[136][2]
        temp_pose.orientation.w = sensor_data[137][0]
        temp_pose.orientation.x = sensor_data[137][1]
        temp_pose.orientation.y = sensor_data[137][2]
        temp_pose.orientation.z = sensor_data[137][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("bhand/f3/dist")
        joint_state_stamped.position.append(sensor_data[138][0])
        joint_state_stamped.velocity.append(sensor_data[139][0])
        joint_state_stamped.acceleration.append(self.data.joint("bhand/f3/dist").qacc[0])
        joint_state_stamped.effort.append(sensor_data[140][0])

        # Cart
        # Linear acceleration
        temp_accel_1 = Accel()
        temp_accel_1.linear.x = sensor_data[141][0]
        temp_accel_1.linear.y = sensor_data[141][1]
        temp_accel_1.linear.z = sensor_data[141][2]
        # Angular acceleration
        temp_accel_1.angular.x = 0
        temp_accel_1.angular.y = 0
        temp_accel_1.angular.z = (sensor_data[142][1]-sensor_data[141][1])/0.1
        force_torque_comp_stamped.cart_accel_1 = temp_accel_1
        # Linear acceleration
        temp_accel_2 = Accel()
        temp_accel_2.linear.x = sensor_data[142][0]
        temp_accel_2.linear.y = sensor_data[142][1]
        temp_accel_2.linear.z = sensor_data[142][2]
        # Angular acceleration
        temp_accel_2.angular.x = 0
        temp_accel_2.angular.y = 0
        temp_accel_2.angular.z = (sensor_data[142][1]-sensor_data[141][1])/0.1
        force_torque_comp_stamped.cart_accel_2 = temp_accel_2
        # Twist
        temp_twist = Twist()
        # Linear component
        temp_twist.linear.x = sensor_data[143][0]
        temp_twist.linear.y = sensor_data[143][1]
        temp_twist.linear.z = sensor_data[143][2]
        # Angular component
        temp_twist.angular.x = sensor_data[144][0]
        temp_twist.angular.y = sensor_data[144][1]
        temp_twist.angular.z = sensor_data[144][2]
        force_torque_comp_stamped.cart_twist = temp_twist
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[145][0]
        temp_pose.position.y = sensor_data[145][1]
        temp_pose.position.z = sensor_data[145][2]
        temp_pose.orientation.w = sensor_data[146][0]
        temp_pose.orientation.x = sensor_data[146][1]
        temp_pose.orientation.y = sensor_data[146][2]
        temp_pose.orientation.z = sensor_data[146][3]
        force_torque_comp_stamped.cart_pose = temp_pose

        # Publish Link-states
        self.pub_link_state_sensor.publish(link_state_stamped)
        # Publish Joint-states
        self.pub_joint_state_sensor.publish(joint_state_stamped)
        # Publish force_torque_state
        self.pub_ft_sensor.publish(force_torque_state_stamped)
        # Publish FT Compensation
        self.pub_ft_comp.publish(force_torque_comp_stamped)
        # Increment counter
        self.counter +=1