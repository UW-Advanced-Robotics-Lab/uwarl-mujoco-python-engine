#! /usr/bin/env python

# Publishers for publishing simulation states (link_states and joint_states) from MuJoCo to relevant ROS topics
# Last version: Nov 28, 2023 Tim van Meijel

import rospy
from geometry_msgs.msg import Accel, Twist, Pose, Quaternion, WrenchStamped
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from uwarl_mujoco_ros_msgs.msg import JointStateArray, LinkStateArray
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
        self.sensor_list = ['accelerometer_mb','velocimeter_mb','gyroscope_mb','global_pos_mb','global_quat_mb',
                            'accelerometer_wam_base','velocimeter_wam_base','gyroscope_wam_base','global_pos_wam_base','global_quat_wam_base',
                            'accelerometer_wam_shoulder_yaw','velocimeter_wam_shoulder_yaw','gyroscope_wam_shoulder_yaw','global_pos_wam_shoulder_yaw','global_quat_wam_shoulder_yaw','joint_pos_wam_shoulder_yaw','joint_vel_wam_shoulder_yaw',
                            'accelerometer_wam_shoulder_pitch','velocimeter_wam_shoulder_pitch','gyroscope_wam_shoulder_pitch','global_pos_wam_shoulder_pitch','global_quat_wam_shoulder_pitch','joint_pos_wam_shoulder_pitch','joint_vel_wam_shoulder_pitch',
                            'accelerometer_wam_upper_arm','velocimeter_wam_upper_arm','gyroscope_wam_upper_arm','global_pos_wam_upper_arm','global_quat_wam_upper_arm','joint_pos_wam_upper_arm','joint_vel_wam_upper_arm',
                            'accelerometer_wam_forearm','velocimeter_wam_forearm','gyroscope_wam_forearm','global_pos_wam_forearm','global_quat_wam_forearm','joint_pos_wam_forearm','joint_vel_wam_forearm',
                            'accelerometer_wam_wrist_yaw','velocimeter_wam_wrist_yaw','gyroscope_wam_wrist_yaw','global_pos_wam_wrist_yaw','global_quat_wam_wrist_yaw','joint_pos_wam_wrist_yaw','joint_vel_wam_wrist_yaw',
                            'accelerometer_wam_wrist_pitch','velocimeter_wam_wrist_pitch','gyroscope_wam_wrist_pitch','global_pos_wam_wrist_pitch','global_quat_wam_wrist_pitch','joint_pos_wam_wrist_pitch','joint_vel_wam_wrist_pitch',
                            'accelerometer_wam_wrist_palm','velocimeter_wam_wrist_palm','gyroscope_wam_wrist_palm','global_pos_wam_wrist_palm','global_quat_wam_wrist_palm','joint_pos_wam_wrist_palm','joint_vel_wam_wrist_palm',
                            'force_sensor','torque_sensor']

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
        
        # Current time
        curr_time = rospy.Time.now()
        link_state_stamped.header.stamp = curr_time
        joint_state_stamped.header.stamp = curr_time
        link_state_stamped.header.seq = self.counter
        joint_state_stamped.header.seq = self.counter

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
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[3][0]
        temp_pose.position.y = sensor_data[3][1]
        temp_pose.position.z = sensor_data[3][2]

        # Correct mobile-base orientation
        original_orient = Quaternion()
        id_new = self.model.name2id('waterloo_steel','body')
        original_orient.x = self.model.body_quat[id_new][1]
        original_orient.y = self.model.body_quat[id_new][2]
        original_orient.z = self.model.body_quat[id_new][3]
        original_orient.w = self.model.body_quat[id_new][0]

        # Rotate summit base_link back to 0.0 degrees
        orig_quat = [original_orient.x,original_orient.y,original_orient.z,original_orient.w]
        orie_quat = [sensor_data[4][1],sensor_data[4][2],sensor_data[4][3],sensor_data[4][0]]
        inv_quat = quaternion_inverse(orig_quat)
        new_quat = quaternion_multiply(orie_quat,inv_quat)

        temp_pose.orientation.w = new_quat[3]
        temp_pose.orientation.x = new_quat[0]
        temp_pose.orientation.y = new_quat[1]
        temp_pose.orientation.z = new_quat[2]
        link_state_stamped.pose.append(temp_pose)

        # WAM
        # Base Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[5][0]
        temp_accel.linear.y = sensor_data[5][1]
        temp_accel.linear.z = sensor_data[5][2]
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
        temp_twist.linear.x = sensor_data[6][0]
        temp_twist.linear.y = sensor_data[6][1]
        temp_twist.linear.z = sensor_data[6][2]
        # Angular component
        temp_twist.angular.x = sensor_data[7][0]
        temp_twist.angular.y = sensor_data[7][1]
        temp_twist.angular.z = sensor_data[7][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[8][0]
        temp_pose.position.y = sensor_data[8][1]
        temp_pose.position.z = sensor_data[8][2]
        temp_pose.orientation.w = sensor_data[9][0]
        temp_pose.orientation.x = sensor_data[9][1]
        temp_pose.orientation.y = sensor_data[9][2]
        temp_pose.orientation.z = sensor_data[9][3]
        link_state_stamped.pose.append(temp_pose)

        # Shoulder Yaw Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[10][0]
        temp_accel.linear.y = sensor_data[10][1]
        temp_accel.linear.z = sensor_data[10][2]
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
        temp_twist.linear.x = sensor_data[11][0]
        temp_twist.linear.y = sensor_data[11][1]
        temp_twist.linear.z = sensor_data[11][2]
        # Angular component
        temp_twist.angular.x = sensor_data[12][0]
        temp_twist.angular.y = sensor_data[12][1]
        temp_twist.angular.z = sensor_data[12][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[13][0]
        temp_pose.position.y = sensor_data[13][1]
        temp_pose.position.z = sensor_data[13][2]
        temp_pose.orientation.w = sensor_data[14][0]
        temp_pose.orientation.x = sensor_data[14][1]
        temp_pose.orientation.y = sensor_data[14][2]
        temp_pose.orientation.z = sensor_data[14][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J1")
        joint_state_stamped.position.append(sensor_data[15][0])
        joint_state_stamped.velocity.append(sensor_data[16][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J1").qacc[0])

        # Shoulder Pitch Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[17][0]
        temp_accel.linear.y = sensor_data[17][1]
        temp_accel.linear.z = sensor_data[17][2]
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
        temp_twist.linear.x = sensor_data[18][0]
        temp_twist.linear.y = sensor_data[18][1]
        temp_twist.linear.z = sensor_data[18][2]
        # Angular component
        temp_twist.angular.x = sensor_data[19][0]
        temp_twist.angular.y = sensor_data[19][1]
        temp_twist.angular.z = sensor_data[19][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[20][0]
        temp_pose.position.y = sensor_data[20][1]
        temp_pose.position.z = sensor_data[20][2]
        temp_pose.orientation.w = sensor_data[21][0]
        temp_pose.orientation.x = sensor_data[21][1]
        temp_pose.orientation.y = sensor_data[21][2]
        temp_pose.orientation.z = sensor_data[21][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J2")
        joint_state_stamped.position.append(sensor_data[22][0])
        joint_state_stamped.velocity.append(sensor_data[23][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J2").qacc[0])

        # Upper Arm Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[24][0]
        temp_accel.linear.y = sensor_data[24][1]
        temp_accel.linear.z = sensor_data[24][2]
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
        temp_twist.linear.x = sensor_data[25][0]
        temp_twist.linear.y = sensor_data[25][1]
        temp_twist.linear.z = sensor_data[25][2]
        # Angular component
        temp_twist.angular.x = sensor_data[26][0]
        temp_twist.angular.y = sensor_data[26][1]
        temp_twist.angular.z = sensor_data[26][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[27][0]
        temp_pose.position.y = sensor_data[27][1]
        temp_pose.position.z = sensor_data[27][2]
        temp_pose.orientation.w = sensor_data[28][0]
        temp_pose.orientation.x = sensor_data[28][1]
        temp_pose.orientation.y = sensor_data[28][2]
        temp_pose.orientation.z = sensor_data[28][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J3")
        joint_state_stamped.position.append(sensor_data[29][0])
        joint_state_stamped.velocity.append(sensor_data[30][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J3").qacc[0])

        # Forearm Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[31][0]
        temp_accel.linear.y = sensor_data[31][1]
        temp_accel.linear.z = sensor_data[31][2]
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
        temp_twist.linear.x = sensor_data[32][0]
        temp_twist.linear.y = sensor_data[32][1]
        temp_twist.linear.z = sensor_data[32][2]
        # Angular component
        temp_twist.angular.x = sensor_data[33][0]
        temp_twist.angular.y = sensor_data[33][1]
        temp_twist.angular.z = sensor_data[33][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[34][0]
        temp_pose.position.y = sensor_data[34][1]
        temp_pose.position.z = sensor_data[34][2]
        temp_pose.orientation.w = sensor_data[35][0]
        temp_pose.orientation.x = sensor_data[35][1]
        temp_pose.orientation.y = sensor_data[35][2]
        temp_pose.orientation.z = sensor_data[35][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J4")
        joint_state_stamped.position.append(sensor_data[36][0])
        joint_state_stamped.velocity.append(sensor_data[37][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J4").qacc[0])

        # Wrist Yaw Link
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
        link_state_stamped.name.append("wam/wrist_yaw_link")
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
        joint_state_stamped.name.append("wam/J5")
        joint_state_stamped.position.append(sensor_data[43][0])
        joint_state_stamped.velocity.append(sensor_data[44][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J5").qacc[0])

        # Wrist Pitch Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[45][0]
        temp_accel.linear.y = sensor_data[45][1]
        temp_accel.linear.z = sensor_data[45][2]
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
        temp_twist.linear.x = sensor_data[46][0]
        temp_twist.linear.y = sensor_data[46][1]
        temp_twist.linear.z = sensor_data[46][2]
        # Angular component
        temp_twist.angular.x = sensor_data[47][0]
        temp_twist.angular.y = sensor_data[47][1]
        temp_twist.angular.z = sensor_data[47][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[48][0]
        temp_pose.position.y = sensor_data[48][1]
        temp_pose.position.z = sensor_data[48][2]
        temp_pose.orientation.w = sensor_data[49][0]
        temp_pose.orientation.x = sensor_data[49][1]
        temp_pose.orientation.y = sensor_data[49][2]
        temp_pose.orientation.z = sensor_data[49][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J6")
        joint_state_stamped.position.append(sensor_data[50][0])
        joint_state_stamped.velocity.append(sensor_data[51][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J6").qacc[0])

        # Wrist Palm Link
        # Linear acceleration
        temp_accel = Accel()
        temp_accel.linear.x = sensor_data[52][0]
        temp_accel.linear.y = sensor_data[52][1]
        temp_accel.linear.z = sensor_data[52][2]
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
        temp_twist.linear.x = sensor_data[53][0]
        temp_twist.linear.y = sensor_data[53][1]
        temp_twist.linear.z = sensor_data[53][2]
        # Angular component
        temp_twist.angular.x = sensor_data[54][0]
        temp_twist.angular.y = sensor_data[54][1]
        temp_twist.angular.z = sensor_data[54][2]
        link_state_stamped.twist.append(temp_twist)
        # Link pose
        temp_pose = Pose()
        temp_pose.position.x = sensor_data[55][0]
        temp_pose.position.y = sensor_data[55][1]
        temp_pose.position.z = sensor_data[55][2]
        temp_pose.orientation.w = sensor_data[56][0]
        temp_pose.orientation.x = sensor_data[56][1]
        temp_pose.orientation.y = sensor_data[56][2]
        temp_pose.orientation.z = sensor_data[56][3]
        link_state_stamped.pose.append(temp_pose)
        # Joint state
        joint_state_stamped.name.append("wam/J7")
        joint_state_stamped.position.append(sensor_data[57][0])
        joint_state_stamped.velocity.append(sensor_data[58][0])
        joint_state_stamped.acceleration.append(self.data.joint("wam/J7").qacc[0])

        # Publish Link-states
        self.pub_link_state_sensor.publish(link_state_stamped)
        # Publish Joint-states
        self.pub_joint_state_sensor.publish(joint_state_stamped)

        # Force Torque Sensor
        force_torque_state_stamped.header.stamp = curr_time
        force_torque_state_stamped.header.seq = self.counter
        force_torque_state_stamped.wrench.force.x = sensor_data[59][0]
        force_torque_state_stamped.wrench.force.y = sensor_data[59][1]
        force_torque_state_stamped.wrench.force.z = sensor_data[59][2]
        force_torque_state_stamped.wrench.torque.x = sensor_data[60][0]
        force_torque_state_stamped.wrench.torque.y = sensor_data[60][1]
        force_torque_state_stamped.wrench.torque.z = sensor_data[60][2]
        # Publish force_torque_state
        self.pub_ft_sensor.publish(force_torque_state_stamped)
        # Increment counter
        self.counter +=1