#! /usr/bin/env python

# Publishers for publishing simulation states (link_states and joint_states) from MuJoCo to relevant ROS topics
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from geometry_msgs.msg import Accel, Twist, Pose, Quaternion, WrenchStamped
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from uwarl_mujoco_ros_msgs.msg import FTcompensation, JointStateArray, LinkStateArray
from tf.transformations import quaternion_inverse, quaternion_multiply

import time


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
        # List of robots
        self.robot_list = robot_list

        # Define lists to publish
        # These link and joint names are from the robot-kinematic files that are read by MuJoCo.
        # In order to move them using ROS, the corresponding joints of the corresponding robots' urdf must have the same names.
        summit_wam_joint_list = ['smt/orie/z', 'smt/pose/x', 'smt/pose/y', 'smt/whl_LF', 'smt/whl_LR', 'smt/whl_RF', 'smt/whl_RR', 
                                 'smt/world_x', 'smt/world_y', 'smt/world_z',
                                 'wam/J1','wam/J2', 'wam/J3', 'wam/J4', 'wam/J5', 'wam/J6', 'wam/J7',
                                 'bhand/f1/prox', 'bhand/f1/med', 'bhand/f1/dist', 'bhand/f2/prox', 'bhand/f2/med', 
                                 'bhand/f2/dist', 'bhand/f3/med', 'bhand/f3/dist']
        wagon_a_joint_list = ['wagon/LF', 'wagon/LF/whl', 'wagon/LR/whl', 'wagon/RF', 'wagon/RF/whl', 'wagon/RR/whl', 'wagon/handle','wagon/handle2',
                              'wagon/slide/world_x', 'wagon/slide/world_y', 'wagon/slide/world_z', 'wagon/hinge/world_y', 'wagon/hinge/world_z']
        wagon_b_joint_list = ['cart/C_swivel_hub', 'cart/FC_whl',
                              'cart/RL_whl', 'cart/RR_whl',
                              'cart/slide/world_x', 'cart/slide/world_y', 'cart/slide/world_z', 'cart/hinge/world_y', 'cart/hinge/world_z']
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
                               wagon_a_joint_list,
                               wagon_b_joint_list]
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
        wagon_a_link_list = ['utility/wagon', 'wagon', 'wagon/LF', 'wagon/LF/whl', 'wagon/LR', 'wagon/LR/whl', 'wagon/RF', 
                             'wagon/RF/whl', 'wagon/RR', 'wagon/RR/whl', 'wagon/handle', 'wagon/pocket', 'wagon/wire_frame']
        wagon_b_link_list = ['cart/base_link',
                             'cart/swivel_hub/C_link','cart/whl/FC_link',
                             'cart/static_hub/L_link','cart/whl/RL_link','cart/static_hub/R_link','cart/whl/RR_link']
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
                              wagon_a_link_list,
                              wagon_b_link_list]
        self.linklist =  world_link_list
        # Concatenate link-lists
        counter = 0
        for _bool in robot_list:
            if _bool:
                self.linklist += list_of_link_lists[counter]
            counter +=1

        # Sensor list
        summit_wam_sensor_list = ['accelerometer_mb','velocimeter_mb','gyroscope_mb','global_pos_mb','global_quat_mb','joint_vel_mb_lf','joint_vel_mb_rf','joint_vel_mb_rr','joint_vel_mb_lr',
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
                                  'accelerometer_bhand_finger_3_dist','velocimeter_bhand_finger_3_dist','gyroscope_bhand_finger_3_dist','global_pos_bhand_finger_3_dist','global_quat_bhand_finger_3_dist','joint_pos_bhand_finger_3_dist','joint_vel_bhand_finger_3_dist','joint_effort_bhand_finger_3_dist']
        fetch_sensor_list = ['velocimeter_fetch','gyroscope_fetch','global_pos_fetch','global_quat_fetch']
        forklift_sensor_list = ['velocimeter_forklift','gyroscope_forklift','global_pos_forklift','global_quat_forklift']
        wagon_a_sensor_list = ['velocimeter_cart','gyroscope_cart','global_pos_cart','global_quat_cart']
        wagon_b_sensor_list = ['accelerometer_1_cart','accelerometer_2_cart','velocimeter_cart','gyroscope_cart','global_pos_cart','global_quat_cart']
        list_of_sensor_lists = [summit_wam_sensor_list,
                                fetch_sensor_list,
                                forklift_sensor_list,
                                wagon_a_sensor_list,
                                wagon_b_sensor_list]
        self.list_of_total_sensor_num = [len(summit_wam_sensor_list),
                                         len(fetch_sensor_list),
                                         len(forklift_sensor_list),
                                         len(wagon_a_sensor_list),
                                         len(wagon_b_sensor_list)]
        self.sensor_list = []
        # Concatenate sensor-lists
        counter = 0
        for _bool in robot_list:
            if _bool:
                self.sensor_list += list_of_sensor_lists[counter]
            counter +=1
        # MB effort
        self.mb_effort = ['smt/pose/x','smt/pose/y','smt/orie/z']
    # Publish joint states: relative to initial state (which is 0.0 for all joints)
    def pub_joint_states(self, muj_time):

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

        self.joint_state.header.stamp = rospy.Time.from_sec(muj_time)

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
    def pub_sensor_states(self, muj_time):

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
        link_state_stamped.header.stamp = rospy.Time.from_sec(muj_time)
        joint_state_stamped.header.stamp = rospy.Time.from_sec(muj_time)
        force_torque_state_stamped.header.stamp = rospy.Time.from_sec(muj_time)
        force_torque_comp_stamped.header.stamp = rospy.Time.from_sec(muj_time)
        link_state_stamped.header.seq = self.counter
        joint_state_stamped.header.seq = self.counter
        force_torque_state_stamped.header.seq = self.counter
        force_torque_comp_stamped.header.seq = self.counter

        curr_time_1 = time.time()
        force_torque_comp_stamped.curr_time = curr_time_1
        link_state_stamped.curr_time = curr_time_1
        joint_state_stamped.curr_time = curr_time_1

        # Current cummulative sum of sensors so far
        cumm_sum_sensor_num_0 = 0
        curr_sensor_num_0 = 0
        if(self.robot_list[0]):
            curr_sensor_num_0 = self.list_of_total_sensor_num[0]
            # Add sensors corresponding to the Summit Wam system
            # Mobile-base
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+0][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+0][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+0][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+1][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+1][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+1][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+2][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+2][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+2][2]
            link_state_stamped.twist.append(temp_twist)
            force_torque_comp_stamped.mb_twist = temp_twist
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+3][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+3][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+3][2]

            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+4][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+4][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+4][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+4][3]
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
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+9][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+9][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+9][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+10][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+10][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+10][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+11][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+11][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+11][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+12][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+12][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+12][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+13][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+13][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+13][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+13][3]
            link_state_stamped.pose.append(temp_pose)

            # Shoulder Yaw Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+14][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+14][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+14][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+15][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+15][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+15][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+16][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+16][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+16][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+17][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+17][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+17][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+18][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+18][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+18][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+18][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("wam/J1")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+19][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+20][0])
            joint_state_stamped.acceleration.append(self.data.joint("wam/J1").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+21][0])

            force_torque_comp_stamped.joint_name.append("wam/J1")
            force_torque_comp_stamped.joint_position.append(sensor_data[cumm_sum_sensor_num_0+19][0])
            force_torque_comp_stamped.joint_velocity.append(sensor_data[cumm_sum_sensor_num_0+20][0])
            force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J1").qacc[0])
            force_torque_comp_stamped.joint_effort.append(sensor_data[cumm_sum_sensor_num_0+21][0])

            # Shoulder Pitch Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+22][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+22][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+22][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+23][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+23][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+23][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+24][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+24][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+24][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+25][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+25][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+25][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+26][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+26][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+26][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+26][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("wam/J2")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+27][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+28][0])
            joint_state_stamped.acceleration.append(self.data.joint("wam/J2").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+29][0])

            force_torque_comp_stamped.joint_name.append("wam/J2")
            force_torque_comp_stamped.joint_position.append(sensor_data[cumm_sum_sensor_num_0+27][0])
            force_torque_comp_stamped.joint_velocity.append(sensor_data[cumm_sum_sensor_num_0+28][0])
            force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J2").qacc[0])
            force_torque_comp_stamped.joint_effort.append(sensor_data[cumm_sum_sensor_num_0+29][0])

            # Upper Arm Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+30][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+30][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+30][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+31][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+31][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+31][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+32][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+32][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+32][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+33][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+33][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+33][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+34][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+34][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+34][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+34][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("wam/J3")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+35][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+36][0])
            joint_state_stamped.acceleration.append(self.data.joint("wam/J3").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+37][0])

            force_torque_comp_stamped.joint_name.append("wam/J3")
            force_torque_comp_stamped.joint_position.append(sensor_data[cumm_sum_sensor_num_0+35][0])
            force_torque_comp_stamped.joint_velocity.append(sensor_data[cumm_sum_sensor_num_0+36][0])
            force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J3").qacc[0])
            force_torque_comp_stamped.joint_effort.append(sensor_data[cumm_sum_sensor_num_0+37][0])

            # Forearm Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+38][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+38][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+38][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+39][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+39][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+39][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+40][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+40][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+40][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+41][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+41][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+41][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+42][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+42][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+42][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+42][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("wam/J4")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+43][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+44][0])
            joint_state_stamped.acceleration.append(self.data.joint("wam/J4").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+45][0])

            force_torque_comp_stamped.joint_name.append("wam/J4")
            force_torque_comp_stamped.joint_position.append(sensor_data[cumm_sum_sensor_num_0+43][0])
            force_torque_comp_stamped.joint_velocity.append(sensor_data[cumm_sum_sensor_num_0+44][0])
            force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J4").qacc[0])
            force_torque_comp_stamped.joint_effort.append(sensor_data[cumm_sum_sensor_num_0+45][0])

            # Wrist Yaw Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+46][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+46][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+46][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+47][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+47][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+47][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+48][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+48][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+48][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+49][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+49][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+49][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+50][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+50][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+50][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+50][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("wam/J5")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+51][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+52][0])
            joint_state_stamped.acceleration.append(self.data.joint("wam/J5").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+53][0])

            force_torque_comp_stamped.joint_name.append("wam/J5")
            force_torque_comp_stamped.joint_position.append(sensor_data[cumm_sum_sensor_num_0+51][0])
            force_torque_comp_stamped.joint_velocity.append(sensor_data[cumm_sum_sensor_num_0+52][0])
            force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J5").qacc[0])
            force_torque_comp_stamped.joint_effort.append(sensor_data[cumm_sum_sensor_num_0+53][0])

            # Wrist Pitch Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+54][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+54][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+54][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+55][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+55][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+55][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+56][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+56][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+56][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+57][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+57][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+57][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+58][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+58][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+58][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+58][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("wam/J6")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+59][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+60][0])
            joint_state_stamped.acceleration.append(self.data.joint("wam/J6").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+61][0])

            force_torque_comp_stamped.joint_name.append("wam/J6")
            force_torque_comp_stamped.joint_position.append(sensor_data[cumm_sum_sensor_num_0+59][0])
            force_torque_comp_stamped.joint_velocity.append(sensor_data[cumm_sum_sensor_num_0+60][0])
            force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J6").qacc[0])
            force_torque_comp_stamped.joint_effort.append(sensor_data[cumm_sum_sensor_num_0+61][0])

            # Wrist Palm Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+62][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+62][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+62][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+63][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+63][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+63][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+64][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+64][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+64][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+65][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+65][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+65][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+66][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+66][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+66][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+66][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("wam/J7")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+67][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+68][0])
            joint_state_stamped.acceleration.append(self.data.joint("wam/J7").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+69][0])

            force_torque_comp_stamped.joint_name.append("wam/J7")
            force_torque_comp_stamped.joint_position.append(sensor_data[cumm_sum_sensor_num_0+67][0])
            force_torque_comp_stamped.joint_velocity.append(sensor_data[cumm_sum_sensor_num_0+68][0])
            force_torque_comp_stamped.joint_acceleration.append(self.data.joint("wam/J7").qacc[0])
            force_torque_comp_stamped.joint_effort.append(sensor_data[cumm_sum_sensor_num_0+69][0])

            # Force Torque Sensor
            force_torque_state_stamped.wrench.force.x = sensor_data[cumm_sum_sensor_num_0+70][0]
            force_torque_state_stamped.wrench.force.y = sensor_data[cumm_sum_sensor_num_0+70][1]
            force_torque_state_stamped.wrench.force.z = sensor_data[cumm_sum_sensor_num_0+70][2]
            force_torque_state_stamped.wrench.torque.x = sensor_data[cumm_sum_sensor_num_0+71][0]
            force_torque_state_stamped.wrench.torque.y = sensor_data[cumm_sum_sensor_num_0+71][1]
            force_torque_state_stamped.wrench.torque.z = sensor_data[cumm_sum_sensor_num_0+71][2]

            force_torque_comp_stamped.ft_sensor_wrench.force.x = sensor_data[cumm_sum_sensor_num_0+70][0]
            force_torque_comp_stamped.ft_sensor_wrench.force.y = sensor_data[cumm_sum_sensor_num_0+70][1]
            force_torque_comp_stamped.ft_sensor_wrench.force.z = sensor_data[cumm_sum_sensor_num_0+70][2]
            force_torque_comp_stamped.ft_sensor_wrench.torque.x = sensor_data[cumm_sum_sensor_num_0+71][0]
            force_torque_comp_stamped.ft_sensor_wrench.torque.y = sensor_data[cumm_sum_sensor_num_0+71][1]
            force_torque_comp_stamped.ft_sensor_wrench.torque.z = sensor_data[cumm_sum_sensor_num_0+71][2]

            # BHand Palm Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+72][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+72][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+72][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+73][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+73][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+73][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+74][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+74][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+74][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+75][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+75][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+75][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+76][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+76][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+76][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+76][3]
            link_state_stamped.pose.append(temp_pose)

            # BHand Finger 1 Prox Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+77][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+77][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+77][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+78][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+78][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+78][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+79][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+79][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+79][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+80][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+80][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+80][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+81][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+81][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+81][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+81][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f1/prox")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+82][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+83][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f1/prox").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+84][0])

            # BHand Finger 1 Med Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+85][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+85][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+85][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+86][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+86][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+86][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+87][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+87][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+87][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+88][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+88][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+88][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+89][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+89][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+89][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+89][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f1/med")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+90][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+91][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f1/med").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+92][0])

            # BHand Finger 1 Dist Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+93][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+93][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+93][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+94][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+94][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+94][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+95][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+95][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+95][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+96][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+96][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+96][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+97][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+97][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+97][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+97][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f1/dist")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+98][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+99][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f1/dist").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+100][0])

            # BHand Finger 2 Prox Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+101][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+101][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+101][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+102][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+102][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+102][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+103][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+103][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+103][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+104][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+104][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+104][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+105][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+105][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+105][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+105][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f2/prox")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+106][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+107][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f2/prox").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+108][0])

            # BHand Finger 2 Med Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+109][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+109][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+109][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+110][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+110][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+110][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+111][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+111][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+111][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+112][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+112][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+112][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+113][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+113][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+113][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+113][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f2/med")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+114][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+115][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f2/med").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+116][0])

            # BHand Finger 2 Dist Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+117][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+117][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+117][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+118][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+118][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+118][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+119][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+119][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+119][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+120][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+120][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+120][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+121][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+121][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+121][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+121][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f2/dist")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+122][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+123][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f2/dist").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+124][0])

            # BHand Finger 3 Med Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+125][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+125][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+125][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+126][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+126][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+126][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+127][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+127][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+127][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+128][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+128][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+128][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+129][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+129][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+129][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+129][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f3/med")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+130][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+131][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f3/med").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+132][0])

            # BHand Finger 3 Dist Link
            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = sensor_data[cumm_sum_sensor_num_0+133][0]
            temp_accel.linear.y = sensor_data[cumm_sum_sensor_num_0+133][1]
            temp_accel.linear.z = sensor_data[cumm_sum_sensor_num_0+133][2]
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
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_0+134][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_0+134][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_0+134][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_0+135][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_0+135][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_0+135][2]
            link_state_stamped.twist.append(temp_twist)
            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_0+136][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_0+136][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_0+136][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_0+137][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_0+137][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_0+137][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_0+137][3]
            link_state_stamped.pose.append(temp_pose)
            # Joint state
            joint_state_stamped.name.append("bhand/f3/dist")
            joint_state_stamped.position.append(sensor_data[cumm_sum_sensor_num_0+138][0])
            joint_state_stamped.velocity.append(sensor_data[cumm_sum_sensor_num_0+139][0])
            joint_state_stamped.acceleration.append(self.data.joint("bhand/f3/dist").qacc[0])
            joint_state_stamped.effort.append(sensor_data[cumm_sum_sensor_num_0+140][0])

        cumm_sum_sensor_num_1 = cumm_sum_sensor_num_0+curr_sensor_num_0
        curr_sensor_num_1 = 0

        if(self.robot_list[1]):
            curr_sensor_num_1 = self.list_of_total_sensor_num[1]
            # Fetch

            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = 0
            temp_accel.linear.y = 0
            temp_accel.linear.z = 0
            # Angular acceleration
            temp_accel.angular.x = 0
            temp_accel.angular.y = 0
            temp_accel.angular.z = 0
            link_state_stamped.accel.append(temp_accel)
            # Link name
            link_state_stamped.name.append("fetch")

            # Twist
            temp_twist = Twist()
            # Linear component
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_1+0][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_1+0][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_1+0][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_1+1][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_1+1][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_1+1][2]
            link_state_stamped.twist.append(temp_twist)
            force_torque_comp_stamped.fetch_twist = temp_twist

            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_1+2][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_1+2][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_1+2][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_1+3][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_1+3][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_1+3][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_1+3][3]
            link_state_stamped.pose.append(temp_pose)
            force_torque_comp_stamped.fetch_pose = temp_pose
        
        cumm_sum_sensor_num_2 = cumm_sum_sensor_num_1+curr_sensor_num_1
        curr_sensor_num_2 = 0

        if(self.robot_list[2]):
            curr_sensor_num_2 = self.list_of_total_sensor_num[2]
            # Forklift

            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = 0
            temp_accel.linear.y = 0
            temp_accel.linear.z = 0
            # Angular acceleration
            temp_accel.angular.x = 0
            temp_accel.angular.y = 0
            temp_accel.angular.z = 0
            link_state_stamped.accel.append(temp_accel)
            # Link name
            link_state_stamped.name.append("fork_lift/base_link")

            # Twist
            temp_twist = Twist()
            # Linear component
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_2+0][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_2+0][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_2+0][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_2+1][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_2+1][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_2+1][2]
            link_state_stamped.twist.append(temp_twist)
            force_torque_comp_stamped.forklift_twist = temp_twist

            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_2+2][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_2+2][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_2+2][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_2+3][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_2+3][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_2+3][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_2+3][3]
            link_state_stamped.pose.append(temp_pose)
            force_torque_comp_stamped.forklift_pose = temp_pose
        
        cumm_sum_sensor_num_3 = cumm_sum_sensor_num_2+curr_sensor_num_2
        curr_sensor_num_3 = 0

        if(self.robot_list[3]):
            curr_sensor_num_3 = self.list_of_total_sensor_num[3]
            # Wagon

            # Linear acceleration
            temp_accel = Accel()
            temp_accel.linear.x = 0
            temp_accel.linear.y = 0
            temp_accel.linear.z = 0
            # Angular acceleration
            temp_accel.angular.x = 0
            temp_accel.angular.y = 0
            temp_accel.angular.z = 0
            link_state_stamped.accel.append(temp_accel)
            # Link name
            link_state_stamped.name.append("utility/wagon")

            # Twist
            temp_twist = Twist()
            # Linear component
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_3+0][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_3+0][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_3+0][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_3+1][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_3+1][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_3+1][2]
            link_state_stamped.twist.append(temp_twist)
            force_torque_comp_stamped.wagon_twist = temp_twist

            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_3+2][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_3+2][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_3+2][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_3+3][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_3+3][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_3+3][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_3+3][3]
            link_state_stamped.pose.append(temp_pose)
            force_torque_comp_stamped.wagon_pose = temp_pose

        cumm_sum_sensor_num_4 = cumm_sum_sensor_num_3+curr_sensor_num_3
        curr_sensor_num_4 = 0

        if(self.robot_list[4]):
            curr_sensor_num_4 = self.list_of_total_sensor_num[4]
            # Cart
            # Linear acceleration
            temp_accel_1 = Accel()
            temp_accel_1.linear.x = sensor_data[cumm_sum_sensor_num_4+0][0]
            temp_accel_1.linear.y = sensor_data[cumm_sum_sensor_num_4+0][1]
            temp_accel_1.linear.z = sensor_data[cumm_sum_sensor_num_4+0][2]
            # Angular acceleration
            temp_accel_1.angular.x = 0
            temp_accel_1.angular.y = 0
            temp_accel_1.angular.z = (sensor_data[cumm_sum_sensor_num_4+1][1]-sensor_data[cumm_sum_sensor_num_4+0][1])/0.1
            force_torque_comp_stamped.cart_accel_1 = temp_accel_1
            link_state_stamped.accel.append(temp_accel_1)
            # Linear acceleration
            temp_accel_2 = Accel()
            temp_accel_2.linear.x = sensor_data[cumm_sum_sensor_num_4+1][0]
            temp_accel_2.linear.y = sensor_data[cumm_sum_sensor_num_4+1][1]
            temp_accel_2.linear.z = sensor_data[cumm_sum_sensor_num_4+1][2]
            # Angular acceleration
            temp_accel_2.angular.x = 0
            temp_accel_2.angular.y = 0
            temp_accel_2.angular.z = (sensor_data[cumm_sum_sensor_num_4+1][1]-sensor_data[cumm_sum_sensor_num_4+0][1])/0.1
            force_torque_comp_stamped.cart_accel_2 = temp_accel_2

            # Link name
            link_state_stamped.name.append("cart")

            # Twist
            temp_twist = Twist()
            # Linear component
            temp_twist.linear.x = sensor_data[cumm_sum_sensor_num_4+2][0]
            temp_twist.linear.y = sensor_data[cumm_sum_sensor_num_4+2][1]
            temp_twist.linear.z = sensor_data[cumm_sum_sensor_num_4+2][2]
            # Angular component
            temp_twist.angular.x = sensor_data[cumm_sum_sensor_num_4+3][0]
            temp_twist.angular.y = sensor_data[cumm_sum_sensor_num_4+3][1]
            temp_twist.angular.z = sensor_data[cumm_sum_sensor_num_4+3][2]
            force_torque_comp_stamped.cart_twist = temp_twist
            link_state_stamped.twist.append(temp_twist)

            # Link pose
            temp_pose = Pose()
            temp_pose.position.x = sensor_data[cumm_sum_sensor_num_4+4][0]
            temp_pose.position.y = sensor_data[cumm_sum_sensor_num_4+4][1]
            temp_pose.position.z = sensor_data[cumm_sum_sensor_num_4+4][2]
            temp_pose.orientation.w = sensor_data[cumm_sum_sensor_num_4+5][0]
            temp_pose.orientation.x = sensor_data[cumm_sum_sensor_num_4+5][1]
            temp_pose.orientation.y = sensor_data[cumm_sum_sensor_num_4+5][2]
            temp_pose.orientation.z = sensor_data[cumm_sum_sensor_num_4+5][3]
            force_torque_comp_stamped.cart_pose = temp_pose
            link_state_stamped.pose.append(temp_pose)

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