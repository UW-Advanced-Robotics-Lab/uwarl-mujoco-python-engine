#! /usr/bin/env python

""" `mujoco_engine.py`

    @author:  Jack (Jianxiang) Xu
        Contacts    : projectbyjx@gmail.com
        Last edits  : July 27, 2022

    @description:
        This library will initiate an Engine that handles updates and rendering
        Last edits  : Nov 28, 2023 (Tim van Meijel)
"""

#===================================#
#  I M P O R T - L I B R A R I E S  #
#===================================#

# python libraries:

import signal

# python 3rd party libraries:
import numpy as np
import mujoco
import cv2
import imageio
from cv_bridge import CvBridge, CvBridgeError

import rospy

from rosgraph_msgs.msg import Clock

# custom libraries:
import mujoco_viewer

# local libraries:
from mujoco_engine.core_engine.wrapper.core import MjData, MjModel

# import publisher and subscriber for migration with ros
from mujoco_engine.core_engine.state_pub_mujoco import StatePublisherMujoco
from mujoco_engine.core_engine.control_commands import ControlCommand
from mujoco_engine.core_engine.effort_control_commands import EffortControlCommand

# For running the MuJoCo viewer on a separate process
from multiprocessing import Process, Queue
import os

from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Float64
from sensor_msgs.msg import Image

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# #        ___           ___         ___          ___           ___           ___       # #
# #       /__/\         /__/\       /  /\        /  /\         /  /\         /  /\      # #
# #      |  |::\        \  \:\     /  /:/       /  /::\       /  /:/        /  /::\     # #
# #      |  |:|:\        \  \:\   /__/::\      /  /:/\:\     /  /:/        /  /:/\:\    # #
# #    __|__|:|\:\   ___  \  \:\  \__\/\:\    /  /:/  \:\   /  /:/  ___   /  /:/  \:\   # #
# #   /__/::::| \:\ /__/\  \__\:\    \  \:\  /__/:/ \__\:\ /__/:/  /  /\ /__/:/ \__\:\  # #
# #   \  \:\~~\__\/ \  \:\ /  /:/     \__\:\ \  \:\ /  /:/ \  \:\ /  /:/ \  \:\ /  /:/  # #
# #    \  \:\        \  \:\  /:/      /  /:/  \  \:\  /:/   \  \:\  /:/   \  \:\  /:/   # #
# #     \  \:\        \  \:\/:/      /__/:/    \  \:\/:/     \  \:\/:/     \  \:\/:/    # #
# #      \  \:\        \  \::/       \__\/      \  \::/       \  \::/       \  \::/     # #
# #       \__\/         \__\/                    \__\/         \__\/         \__\/      # #
# #                                                                                     # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

#=======================#
#  D E F I N I T I O N  #
#=======================#
class MuJoCo_Engine_InterruptException(Exception):
    pass

class Mujoco_Engine:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _camera_config = {
        # "camera/zed/L": {"width": 1280, "height":720, "fps": 60, "id":1},
        # "camera/zed/R": {"width": 1280, "height":720, "fps": 60, "id":0},
        "camera/intel/rgb": {"width": 1280, "height":720, "fps": 60, "id":0}
    }
    _camera_views = {}
    _IC_state = None
    _core = None
    
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self,  
        xml_path, rate_Hz, rate_scene,
        camera_config=None, 
        name="DEFAULT", 
        CAMERA_V_FACTOR=3,
        write_to = None,
        robot_list = None,
        if_camera_preview = False,
        if_viewport_preview = True
    ):
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
        ## Init Configs:
        if camera_config:
            self._camera_config = (camera_config) # override if given
        self.xml_path = xml_path
        self._name = name
        self._rate_Hz = rate_Hz
        self._rate_scene = rate_scene
        # Write camera images to a folder
        self._write_to = write_to
        # Which robot joints and bodies should be published
        self._robot_list = robot_list
        # Camera view
        self.if_camera_preview = if_camera_preview
        # Port view
        self.if_viewport_preview = if_viewport_preview

        # Create publishers to publish camera view
        self.pub_rear_cam = rospy.Publisher('/mujoco/camera',Image,queue_size=1)
        # To convert open  CV images to an ecoding that can be passed to ros message image.
        self.bridge = CvBridge()

        # Calculate rendering freq
        self.steps_per_render = round(float(self._rate_Hz)/float(self._rate_scene))
        self.i = 0
        self.last_render = rospy.Time.now()

        # Initial Summit-base yaw angular-displacement
        self.summit_initial_theta = 0.0
        # Initial Fetch-base yaw angular-displacement
        self.fetch_initial_theta = 0.0
        # Initial Forklift-base yaw angular-displacement
        self.forklift_initial_theta = 0.0

        ## Initiate MJ
        self.mj_model = MjModel.from_xml_path(xml_path=self.xml_path)
        self.mj_data = MjData(self.mj_model)
        self.state_pub = StatePublisherMujoco(self.mj_data, self.mj_model, self._robot_list)
        self.pub_time = rospy.Publisher('/simtime',Float64,queue_size=1)
        self.simtime = Float64()
        # Effort-controllers (manipulators like WAM)
        self.effort_control_commands = EffortControlCommand(self.mj_data)
        # Summit
        if (self._robot_list[0]):
            self.summit_base_name = "smt"
            self.summit_cmd_vel_topic_name = "uwarl/robotnik_base_control/cmd_vel"
            self.summit_control_commands = ControlCommand(self.mj_data,self.summit_cmd_vel_topic_name)
            # Get original pose of the MB
            id_new = self.mj_model.name2id('waterloo_steel','body')

            quaternion_vec = R.from_quat(np.array([self.mj_model.body_quat[id_new][1],
                                                   self.mj_model.body_quat[id_new][2],
                                                   self.mj_model.body_quat[id_new][3],
                                                   self.mj_model.body_quat[id_new][0]]))
            rot_mat = quaternion_vec.as_matrix()
            self.summit_initial_theta = np.arctan2(rot_mat[1,0],rot_mat[0,0])

        # Non-Holonomic bodies
        # Fetch
        if (self._robot_list[1]):
            self.fetch_base_name = "fetch"
            self.fetch_cmd_vel_topic_name = "fetch/fetch_base_control/cmd_vel"
            self.fetch_control_commands = ControlCommand(self.mj_data,self.fetch_cmd_vel_topic_name)
            # Get original pose of the Fetch
            id_new = self.mj_model.name2id('fetch_1','body')

            quaternion_vec = R.from_quat(np.array([self.mj_model.body_quat[id_new][1],
                                                   self.mj_model.body_quat[id_new][2],
                                                   self.mj_model.body_quat[id_new][3],
                                                   self.mj_model.body_quat[id_new][0]]))
            rot_mat = quaternion_vec.as_matrix()
            self.fetch_initial_theta = np.arctan2(rot_mat[1,0],rot_mat[0,0])

        # Forklift
        if (self._robot_list[2]):
            self.forklift_base_name = "fork_lift"
            self.forklift_cmd_vel_topic_name = "fork_lift/forklift_base_control/cmd_vel"
            self.forklift_control_commands = ControlCommand(self.mj_data,self.forklift_cmd_vel_topic_name)
            # Get original pose of the Forklift
            id_new = self.mj_model.name2id('fork_lift_1','body')

            quaternion_vec = R.from_quat(np.array([self.mj_model.body_quat[id_new][1],
                                                   self.mj_model.body_quat[id_new][2],
                                                   self.mj_model.body_quat[id_new][3],
                                                   self.mj_model.body_quat[id_new][0]]))
            rot_mat = quaternion_vec.as_matrix()
            self.forklift_initial_theta = np.arctan2(rot_mat[1,0],rot_mat[0,0])

        # Initialized current Summit-base twist
        self.summit_currentx_vel = 0.0
        self.summit_currenty_vel = 0.0
        self.summit_currenttheta_vel = 0.0
        self.summit_current_whl_vel = np.zeros((4,1))
        # Initialized current Fetch-base twist
        self.fetch_currentx_vel = 0.0
        self.fetch_currenty_vel = 0.0
        self.fetch_currenttheta_vel = 0.0
        # Initialized current Forklift-base twist
        self.forklift_currentx_vel = 0.0
        self.forklift_currenty_vel = 0.0
        self.forklift_currenttheta_vel = 0.0

        # Initialized current Summit-base yaw angular-displacement
        self.summit_current_theta = 0.0
        # Initialized current Fetch-base yaw angular-displacement
        self.fetch_current_theta = 0.0
        # Initialized current Forklift-base yaw angular-displacement
        self.forklift_current_theta = 0.0

        ## MJ Viewer:
        self.mj_viewer = mujoco_viewer.MujocoViewer(self.mj_model._model, self.mj_data._data, 
            title="Mujoco-Engine", 
            sensor_config=self._camera_config,
            window_size=(1280,720),
        )
        # if len(self._camera_config):
        #     self.mj_viewer_off = mujoco_viewer.MujocoViewer(self.mj_model, self.mj_data, width=800, height=800, title="camera-view")
        # self._t_update = time.time()
        
        # cv2 window
        cv2.startWindowThread()
        self.h_min = np.Infinity
        self.width = 0
        for camera, config in self._camera_config.items():
            self.h_min = int(min(config["height"]/CAMERA_V_FACTOR, self.h_min))
            # Sum up width
            self.width += config["width"]
        
        # Start video capture
        self.camera_video = cv2.VideoWriter(self._write_to+'/camera.avi',  
                                            cv2.VideoWriter_fourcc(*'MJPG'), 
                                            self._rate_scene, (self.width,self.h_min))
        self.viewport_video = cv2.VideoWriter(self._write_to+'/viewport.avi',  
                                              cv2.VideoWriter_fourcc(*'MJPG'), 
                                              self._rate_scene, (1280,720))

        # # Initialize the queues
        # # Queue of in-coming MuJoCo data
        # self.queue_muj_data = Queue()
        # # self.queue_muj_data.put(self.mj_data._data)
        
        # # Viewer
        # p_viewer = Process(target=self.muj_viewer, args=(self.queue_muj_data,))
        
        # # Start the process:
        # # To render
        # p_viewer.start()
        # rospy.loginfo("Start rendering ...")

        
    #==================================#
    #  P U B L I C    F U N C T I O N  #
    #==================================#
    def shutdown(self):
        print("[Job_Engine::{}] Program killed: running cleanup code".format(self._name))
        self.mj_viewer._on_terminate_safe()
        cv2.destroyAllWindows()
            
    def is_shutdown(self):
        try:
            return False
        except MuJoCo_Engine_InterruptException:
            self.shutdown()
            return True
    
    #====================================#
    #  P R I V A T E    F U N C T I O N  #
    #====================================#    
    def _signal_handler(self, signum, frame):
        raise MuJoCo_Engine_InterruptException
    
    def _internal_engine_update(self):
        self._update()

    def _update(self):

        # Get current velocity of base for PID control
        # For Summit
        if (self._robot_list[0]):
            # self.summit_currentx_vel = self.mj_data.body(self.summit_base_name+"/base_link").cvel[3]
            # self.summit_currenty_vel = self.mj_data.body(self.summit_base_name+"/base_link").cvel[4]
            # self.summit_currenttheta_vel = self.mj_data.body(self.summit_base_name+"/base_link").cvel[2]
            # https://www.roboti.us/forum/index.php?threads/reading-sensor-values.3972/#post-5368
            mb_velocimeter_id = self.mj_model.name2id("velocimeter_mb",'sensor')
            mb_gyroscope_id = self.mj_model.name2id("gyroscope_mb",'sensor')
            mb_quaternion_id = self.mj_model.name2id("global_quat_mb",'sensor')
            mb_whl_lf_id = self.mj_model.name2id("joint_vel_mb_lf",'sensor')
            mb_whl_rf_id = self.mj_model.name2id("joint_vel_mb_rf",'sensor')
            mb_whl_rr_id = self.mj_model.name2id("joint_vel_mb_rr",'sensor')
            mb_whl_lr_id = self.mj_model.name2id("joint_vel_mb_lr",'sensor')
            # Sensor index
            mb_velocimeter_sensor_index = self.mj_model.sensor_adr[mb_velocimeter_id]
            mb_gyroscope_sensor_index = self.mj_model.sensor_adr[mb_gyroscope_id]
            mb_quaternion_sensor_index = self.mj_model.sensor_adr[mb_quaternion_id]
            mb_whl_lf_sensor_index = self.mj_model.sensor_adr[mb_whl_lf_id]
            mb_whl_rf_sensor_index = self.mj_model.sensor_adr[mb_whl_rf_id]
            mb_whl_rr_sensor_index = self.mj_model.sensor_adr[mb_whl_rr_id]
            mb_whl_lr_sensor_index = self.mj_model.sensor_adr[mb_whl_lr_id]
            # Sensor dimension
            mb_velocimeter_sensor_dim = self.mj_model.sensor_dim[mb_velocimeter_id]
            mb_gyroscope_sensor_dim = self.mj_model.sensor_dim[mb_gyroscope_id]
            mb_quaternion_sensor_dim = self.mj_model.sensor_dim[mb_quaternion_id]
            mb_whl_lf_sensor_dim = self.mj_model.sensor_dim[mb_whl_lf_id]
            mb_whl_rf_sensor_dim = self.mj_model.sensor_dim[mb_whl_rf_id]
            mb_whl_rr_sensor_dim = self.mj_model.sensor_dim[mb_whl_rr_id]
            mb_whl_lr_sensor_dim = self.mj_model.sensor_dim[mb_whl_lr_id]
            # Append sensor data
            mb_velocimeter_sensor_data = self.mj_data.sensordata[mb_velocimeter_sensor_index:(mb_velocimeter_sensor_index+mb_velocimeter_sensor_dim)]
            mb_gyroscope_sensor_data = self.mj_data.sensordata[mb_gyroscope_sensor_index:(mb_gyroscope_sensor_index+mb_gyroscope_sensor_dim)]
            mb_quaternion_sensor_data = self.mj_data.sensordata[mb_quaternion_sensor_index:(mb_quaternion_sensor_index+mb_quaternion_sensor_dim)]
            mb_whl_lf_sensor_data = self.mj_data.sensordata[mb_whl_lf_sensor_index:(mb_whl_lf_sensor_index+mb_whl_lf_sensor_dim)]
            mb_whl_rf_sensor_data = self.mj_data.sensordata[mb_whl_rf_sensor_index:(mb_whl_rf_sensor_index+mb_whl_rf_sensor_dim)]
            mb_whl_rr_sensor_data = self.mj_data.sensordata[mb_whl_rr_sensor_index:(mb_whl_rr_sensor_index+mb_whl_rr_sensor_dim)]
            mb_whl_lr_sensor_data = self.mj_data.sensordata[mb_whl_lr_sensor_index:(mb_whl_lr_sensor_index+mb_whl_lr_sensor_dim)]

            self.summit_currentx_vel = mb_velocimeter_sensor_data[0]
            self.summit_currenty_vel = mb_velocimeter_sensor_data[1]
            self.summit_currenttheta_vel = mb_gyroscope_sensor_data[2]
            self.summit_current_whl_vel[0,0] = mb_whl_lf_sensor_data
            self.summit_current_whl_vel[1,0] = mb_whl_rf_sensor_data
            self.summit_current_whl_vel[2,0] = mb_whl_rr_sensor_data
            self.summit_current_whl_vel[3,0] = mb_whl_lr_sensor_data

            try:
                quaternion_vec = R.from_quat(np.array([mb_quaternion_sensor_data[1],
                                                       mb_quaternion_sensor_data[2],
                                                       mb_quaternion_sensor_data[3],
                                                       mb_quaternion_sensor_data[0]]))
                rot_mat = quaternion_vec.as_matrix()
                self.summit_current_theta = np.arctan2(rot_mat[1,0],rot_mat[0,0])
            except:
                # The initial query gives an incorrect quaternion (the norm of the quaternion-output is not ==1), so we try to catch it and throw out a deafult 0-value.
                self.summit_current_theta = 0

        # For Fetch
        if (self._robot_list[1]):
            # Place sensors on the Fetch. DO NOT USE cvel. (https://github.com/google-deepmind/mujoco/issues/2210#issuecomment-2476113820)
            fetch_velocimeter_id = self.mj_model.name2id("velocimeter_fetch",'sensor')
            fetch_gyroscope_id = self.mj_model.name2id("gyroscope_fetch",'sensor')
            fetch_quaternion_id = self.mj_model.name2id("global_quat_fetch",'sensor')
            # Sensor index
            fetch_velocimeter_sensor_index = self.mj_model.sensor_adr[fetch_velocimeter_id]
            fetch_gyroscope_sensor_index = self.mj_model.sensor_adr[fetch_gyroscope_id]
            fetch_quaternion_sensor_index = self.mj_model.sensor_adr[fetch_quaternion_id]
            # Sensor dimension
            fetch_velocimeter_sensor_dim = self.mj_model.sensor_dim[fetch_velocimeter_id]
            fetch_gyroscope_sensor_dim = self.mj_model.sensor_dim[fetch_gyroscope_id]
            fetch_quaternion_sensor_dim = self.mj_model.sensor_dim[fetch_quaternion_id]
            # Append sensor data
            fetch_velocimeter_sensor_data = self.mj_data.sensordata[fetch_velocimeter_sensor_index:(fetch_velocimeter_sensor_index+fetch_velocimeter_sensor_dim)]
            fetch_gyroscope_sensor_data = self.mj_data.sensordata[fetch_gyroscope_sensor_index:(fetch_gyroscope_sensor_index+fetch_gyroscope_sensor_dim)]
            fetch_quaternion_sensor_data = self.mj_data.sensordata[fetch_quaternion_sensor_index:(fetch_quaternion_sensor_index+fetch_quaternion_sensor_dim)]
            self.fetch_currentx_vel = fetch_velocimeter_sensor_data[0]
            self.fetch_currenty_vel = fetch_velocimeter_sensor_data[1]
            self.fetch_currenttheta_vel = fetch_gyroscope_sensor_data[2]

            try:
                quaternion_vec = R.from_quat(np.array([fetch_quaternion_sensor_data[1],
                                                       fetch_quaternion_sensor_data[2],
                                                       fetch_quaternion_sensor_data[3],
                                                       fetch_quaternion_sensor_data[0]]))
                rot_mat = quaternion_vec.as_matrix()
                self.fetch_current_theta = np.arctan2(rot_mat[1,0],rot_mat[0,0])
            except:
                # The initial query gives an incorrect quaternion (the norm of the quaternion-output is not ==1), so we try to catch it and throw out a deafult 0-value.
                self.fetch_current_theta = 0
        
        # For Forklift
        if (self._robot_list[2]):
            # Place sensors on the Fetch. DO NOT USE cvel. (https://github.com/google-deepmind/mujoco/issues/2210#issuecomment-2476113820)
            forklift_velocimeter_id = self.mj_model.name2id("velocimeter_forklift",'sensor')
            forklift_gyroscope_id = self.mj_model.name2id("gyroscope_forklift",'sensor')
            forklift_quaternion_id = self.mj_model.name2id("global_quat_forklift",'sensor')
            # Sensor index
            forklift_velocimeter_sensor_index = self.mj_model.sensor_adr[forklift_velocimeter_id]
            forklift_gyroscope_sensor_index = self.mj_model.sensor_adr[forklift_gyroscope_id]
            forklift_quaternion_sensor_index = self.mj_model.sensor_adr[forklift_quaternion_id]
            # Sensor dimension
            forklift_velocimeter_sensor_dim = self.mj_model.sensor_dim[forklift_velocimeter_id]
            forklift_gyroscope_sensor_dim = self.mj_model.sensor_dim[forklift_gyroscope_id]
            forklift_quaternion_sensor_dim = self.mj_model.sensor_dim[forklift_quaternion_id]
            # Append sensor data
            forklift_velocimeter_sensor_data = self.mj_data.sensordata[forklift_velocimeter_sensor_index:(forklift_velocimeter_sensor_index+forklift_velocimeter_sensor_dim)]
            forklift_gyroscope_sensor_data = self.mj_data.sensordata[forklift_gyroscope_sensor_index:(forklift_gyroscope_sensor_index+forklift_gyroscope_sensor_dim)]
            forklift_quaternion_sensor_data = self.mj_data.sensordata[forklift_quaternion_sensor_index:(forklift_quaternion_sensor_index+forklift_quaternion_sensor_dim)]
            self.forklift_currentx_vel = forklift_velocimeter_sensor_data[0]
            self.forklift_currenty_vel = forklift_velocimeter_sensor_data[1]
            self.forklift_currenttheta_vel = forklift_gyroscope_sensor_data[2]

            try:
                quaternion_vec = R.from_quat(np.array([forklift_quaternion_sensor_data[1],
                                                       forklift_quaternion_sensor_data[2],
                                                       forklift_quaternion_sensor_data[3],
                                                       forklift_quaternion_sensor_data[0]]))
                rot_mat = quaternion_vec.as_matrix()
                self.forklift_current_theta = np.arctan2(rot_mat[1,0],rot_mat[0,0])
            except:
                # The initial query gives an incorrect quaternion (the norm of the quaternion-output is not ==1), so we try to catch it and throw out a deafult 0-value.
                self.forklift_current_theta = 0

        # Set control commands by simple PID control defined in "control_commands.py"
        # For Summit
        if (self._robot_list[0]):
            self.summit_control_commands.vel_PID([25.0,25.0,12.0], 
                                                 [0.3,0.3,0.3], 
                                                 [1.3,1.3,0.3], 
                                                 [self.summit_currentx_vel,self.summit_currenty_vel,self.summit_currenttheta_vel],
                                                 self.summit_current_theta-self.summit_initial_theta,
                                                 self.summit_base_name)
            self.summit_control_commands.wheel_PID(20, 0.3, 0.01, self.summit_current_whl_vel,self.summit_base_name)

        # For Fetch
        if (self._robot_list[1]):
            self.fetch_control_commands.vel_PID([25.0,25.0,12.0], 
                                                [0.3,0.3,0.3], 
                                                [1.3,1.3,0], 
                                                [self.fetch_currentx_vel,self.fetch_currenty_vel,self.fetch_currenttheta_vel],
                                                self.fetch_current_theta-self.fetch_initial_theta,
                                                self.fetch_base_name)
            
        # For Forklift
        if (self._robot_list[2]):
            self.forklift_control_commands.vel_PID([25.0,25.0,12.0], 
                                                   [0.3,0.3,0.3], 
                                                   [1.3,1.3,0.3], 
                                                   [self.forklift_currentx_vel,self.forklift_currenty_vel,self.forklift_currenttheta_vel],
                                                   self.forklift_current_theta-self.forklift_initial_theta,
                                                   self.forklift_base_name)
        
        ################################################
        # In order to keep the MuJoCo viewer frame-rate in sync with the engine.
        # Don't want the viewer to lag behind the engine.

        if not self.mj_viewer.is_key_registered_to_pause_program_safe() or \
            self.mj_viewer.is_key_registered_to_step_to_next_safe():

            # - render current view:
            steps = round(1/self._rate_Hz/self.mj_model._model.opt.timestep)
            for i in range(steps):
                mujoco.mj_step(self.mj_model._model, self.mj_data._data)
            
            self.mj_viewer.reset_key_registered_to_step_to_next_safe()
        
        sim_time_msg = Clock()
        sim_time_msg.clock = rospy.Time.from_sec(self.mj_data.time)
        self.clock_pub.publish(sim_time_msg)

        # # Extract only what the viewer needs to render
        # state_to_send = {
        #     'qpos': self.mj_data.qpos.copy(),
        #     'qvel': self.mj_data.qvel.copy(),
        #     'time': self.mj_data.time
        # }
        # self.queue_muj_data.put(state_to_send)

        # process GUI interrupts
        self.mj_viewer.process_safe()

        # Update mj_viewer with specified frequency
        if self.i == 0:

            self.mj_viewer.update_safe()
            self.mj_viewer.render_safe()
            self.i=self.steps_per_render-1

            # Set "if_viewport_preview" to True (input "_update" function) when you want to plot the viewport's view
            if self.if_viewport_preview:
                # - capture view:
                viewport_data = self.mj_viewer.acquire_viewport_frames_safe()
                if( not (viewport_data["frame_buffer"] == [])):
                    # image = viewport_data["frame_buffer"]
                    # np.reshape(image, (-1,1))
                    # std_dev=np.std(image)
                    # print(std_dev)
                    img = cv2.cvtColor(viewport_data["frame_buffer"], cv2.COLOR_RGB2BGR)
                    # img = cv2.flip(img, 0)
                    img = cv2.resize(img, (1280, 720))
                    self.viewport_video.write(img)
                    cv2.waitKey(int(1000/self._rate_Hz))

            # Set "if_camera_preview" to True (input "_update" function) when you want to plot the cameras mounted on the WAM
            # Rendering of sensor cameras takes long!! Reduce update frequency to maintain real time simulation!
            if self.if_camera_preview:

                self.mj_viewer.render_sensor_cameras_safe()

                # - capture view:
                camera_sensor_data = self.mj_viewer.acquire_sensor_camera_frames_safe()

                # render captured views on cv2      
                cv2_capture_window = []
                for camera_buf, frame_time_stamp in zip(camera_sensor_data["frame_buffer"].items(),camera_sensor_data["frame_stamp"].items()):
                    img = cv2.cvtColor(camera_buf[1], cv2.COLOR_RGB2BGR)
                    img = cv2.flip(img, 0)
                    img = cv2.resize(img, (int(img.shape[1] * self.h_min / img.shape[0]), self.h_min))
                    # if self._write_to: #[NOT-USED: Implementation to save frames directly]
                    #     imageio.imwrite(
                    #         "{}/{}_{}.png".format(self._write_to, camera_buf[0].replace("/", "_"), frame_time_stamp[1]), 
                    #         img
                    #     )
                    cv2_capture_window.append(img)
                # Write depth-image (check if we have actually enabled depth-sensor capabilities in the camera-plugin used when defining a camera-sensor.)
                # if self._write_to: #[NOT-USED: Implementation to save frames directly]
                #     for camera_depth_buf, frame_time_stamp in zip(camera_sensor_data["depth_buffer"].items(),camera_sensor_data["frame_stamp"].items()):
                #         # Covert float-type gray-scale to uint8 (https://stackoverflow.com/a/60014123/19163020)
                #         uint_8_img = (camera_depth_buf[1]*255).astype(np.uint8)
                #         imageio.imwrite(
                #             "{}/{}_{}_gray.png".format(self._write_to, camera_depth_buf[0].replace("/", "_"), frame_time_stamp[1]), 
                #             uint_8_img
                #         )
                hoz_cat_img = cv2.hconcat(cv2_capture_window)
                cv2.imshow("camera views",hoz_cat_img)
                self.camera_video.write(hoz_cat_img)

                
                rear_cam = self.bridge.cv2_to_imgmsg(hoz_cat_img, "bgr8")
                rear_cam.header.frame_id = "rear"
                # Current time
                curr_time = rospy.Time.now()
                rear_cam.header.stamp = curr_time
                self.pub_rear_cam.publish(rear_cam)

                cv2.waitKey(int(1000/self._rate_Hz))

        self.i-=1

        # Publish link_states and joint_states
        self.state_pub.pub_joint_states(self.mj_data.time)
        self.state_pub.pub_link_states()
        self.state_pub.pub_sensor_states(self.mj_data.time)

        # Publish simulation time
        self.simtime.data = self.mj_data.time
        self.pub_time.publish(self.simtime)
    
    def muj_viewer(self,queue_muj_data):
        # --- CORE ISOLATION ---
        # Pin this viewer process exclusively to Core 7
        try:
            # '0' refers to the current process (the child process)
            os.sched_setaffinity(0, {8})
            print(f"[Viewer] Process started and pinned to CPU core(s): {os.sched_getaffinity(0)}")
        except Exception as e:
            print(f"[Viewer] Could not pin to core 8: {e}")
        # ----------------------
        
        # 1. Re-initialize the Model in the new process
        # We need the path, which you should store in self.xml_path
        local_model = MjModel.from_xml_path(self.xml_path)
        local_data = MjData(local_model)

        # Calculate rendering freq
        self.i = 0
        
        # 2. Re-initialize the viewer in the new process
        local_viewer = mujoco_viewer.MujocoViewer(
            local_model._model, local_data._data, 
            title="Mujoco-Engine-Viewer",
            sensor_config=self._camera_config,
            window_size=(1280, 720)
        )

        while True:
            # Wait for the data to be loaded (we can afford this because the dat will be made available at each tick)
            state = queue_muj_data.get() # Receive the arrays
            # Update the local viewer-process MjData instance
            np.copyto(local_data.qpos, state['qpos'])
            np.copyto(local_data.qvel, state['qvel'])
            local_data.time = state['time']

            # stepping if needed
            if not local_viewer.is_key_registered_to_pause_program_safe() or \
                local_viewer.is_key_registered_to_step_to_next_safe():

                # - render current view:
                steps = round(1/self._rate_Hz/self.mj_model._model.opt.timestep)
                for i in range(steps):
                    mujoco.mj_step(self.mj_model._model, local_data._data)
                
                local_viewer.reset_key_registered_to_step_to_next_safe()

            # process GUI interrupts
            local_viewer.process_safe()

            if self.i == 0:

                local_viewer.update_safe()
                local_viewer.render_safe()
                self.i=self.steps_per_render-1

                # Set "if_viewport_preview" to True (input "_update" function) when you want to plot the viewport's view
                if self.if_viewport_preview:
                    # - capture view:
                    viewport_data = local_viewer.acquire_viewport_frames_safe()
                    if( not (viewport_data["frame_buffer"] == [])):
                        # image = viewport_data["frame_buffer"]
                        # np.reshape(image, (-1,1))
                        # std_dev=np.std(image)
                        # print(std_dev)
                        img = cv2.cvtColor(viewport_data["frame_buffer"], cv2.COLOR_RGB2BGR)
                        # img = cv2.flip(img, 0)
                        img = cv2.resize(img, (1280, 720))
                        self.viewport_video.write(img)
                        cv2.waitKey(int(1000/self._rate_Hz))

                # Set "if_camera_preview" to True (input "_update" function) when you want to plot the cameras mounted on the WAM
                # Rendering of sensor cameras takes long!! Reduce update frequency to maintain real time simulation!
                if self.if_camera_preview:

                    local_viewer.render_sensor_cameras_safe()

                    # - capture view:
                    camera_sensor_data = local_viewer.acquire_sensor_camera_frames_safe()

                    # render captured views on cv2      
                    cv2_capture_window = []
                    for camera_buf, frame_time_stamp in zip(camera_sensor_data["frame_buffer"].items(),camera_sensor_data["frame_stamp"].items()):
                        img = cv2.cvtColor(camera_buf[1], cv2.COLOR_RGB2BGR)
                        img = cv2.flip(img, 0)
                        img = cv2.resize(img, (int(img.shape[1] * self.h_min / img.shape[0]), self.h_min))
                        # if self._write_to: #[NOT-USED: Implementation to save frames directly]
                        #     imageio.imwrite(
                        #         "{}/{}_{}.png".format(self._write_to, camera_buf[0].replace("/", "_"), frame_time_stamp[1]), 
                        #         img
                        #     )
                        cv2_capture_window.append(img)
                    # Write depth-image (check if we have actually enabled depth-sensor capabilities in the camera-plugin used when defining a camera-sensor.)
                    # if self._write_to: #[NOT-USED: Implementation to save frames directly]
                    #     for camera_depth_buf, frame_time_stamp in zip(camera_sensor_data["depth_buffer"].items(),camera_sensor_data["frame_stamp"].items()):
                    #         # Covert float-type gray-scale to uint8 (https://stackoverflow.com/a/60014123/19163020)
                    #         uint_8_img = (camera_depth_buf[1]*255).astype(np.uint8)
                    #         imageio.imwrite(
                    #             "{}/{}_{}_gray.png".format(self._write_to, camera_depth_buf[0].replace("/", "_"), frame_time_stamp[1]), 
                    #             uint_8_img
                    #         )
                    hoz_cat_img = cv2.hconcat(cv2_capture_window)
                    cv2.imshow("camera views",hoz_cat_img)
                    self.camera_video.write(hoz_cat_img)

                    
                    rear_cam = self.bridge.cv2_to_imgmsg(hoz_cat_img, "bgr8")
                    rear_cam.header.frame_id = "rear"
                    # Current time
                    curr_time = rospy.Time.now()
                    rear_cam.header.stamp = curr_time
                    self.pub_rear_cam.publish(rear_cam)

                    cv2.waitKey(int(1000/self._rate_Hz))
                
            self.i-=1
        
    
