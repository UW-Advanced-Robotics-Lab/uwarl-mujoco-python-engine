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

import rospy

# custom libraries:
import mujoco_viewer

# local libraries:
from mujoco_engine.core_engine.wrapper.core import MjData, MjModel

# import publisher and subscriber for migration with ros
from mujoco_engine.core_engine.state_pub_mujoco import StatePublisherMujoco
from mujoco_engine.core_engine.control_commands import ControlCommand
from mujoco_engine.core_engine.effort_control_commands import EffortControlCommand

from std_msgs.msg import Float64

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
        robot_list = None
    ):
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        ## Init Configs:
        if camera_config:
            self._camera_config = (camera_config) # override if given
        self._name = name
        self._rate_Hz = rate_Hz
        self._rate_scene = rate_scene
        # Write camera images to a folder
        self._write_to = write_to
        # Which robot joints and bodies should be published
        self._robot_list = robot_list

        # Calculate rendering freq
        self.steps_per_render = round(float(self._rate_Hz)/float(self._rate_scene))
        self.i = 0
        self.last_render = rospy.Time.now()

        ## Initiate MJ
        self.mj_model = MjModel.from_xml_path(xml_path=xml_path)
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
        # Non-Holonomic bodies
        # Fetch
        if (self._robot_list[1]):
            self.fetch_base_name = "fetch"
            self.fetch_cmd_vel_topic_name = "fetch/fetch_base_control/cmd_vel"
            self.fetch_control_commands = ControlCommand(self.mj_data,self.fetch_cmd_vel_topic_name)
        # Forklift
        if (self._robot_list[2]):
            self.forklift_base_name = "fork_lift"
            self.forklift_cmd_vel_topic_name = "fork_lift/forklift_base_control/cmd_vel"
            self.forklift_control_commands = ControlCommand(self.mj_data,self.forklift_cmd_vel_topic_name)

        # Initialized current Summit-base location
        self.summit_currentx = 0.0
        self.summit_currenty = 0.0
        self.summit_currenttheta = 0.0
        # Initialized current Fetch-base location
        self.fetch_currentx = 0.0
        self.fetch_currenty = 0.0
        self.fetch_currenttheta = 0.0
        # Initialized current Forklift-base location
        self.forklift_currentx = 0.0
        self.forklift_currenty = 0.0
        self.forklift_currenttheta = 0.0

        ## MJ Viewer:
        self.mj_viewer = mujoco_viewer.MujocoViewer(self.mj_model._model, self.mj_data._data, 
            title="Mujoco-Engine", 
            sensor_config=self._camera_config,
            window_size=(1920,1080),
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
        self.video = cv2.VideoWriter(self._write_to+'/filename.avi',  
                                    cv2.VideoWriter_fourcc(*'MJPG'), 
                                    10, (self.width,self.h_min)) 

        
    #==================================#
    #  P U B L I C    F U N C T I O N  #
    #==================================#
    def shutdown(self):
        print("[Job_Engine::{}] Program killed: running cleanup code".format(self._name))
        self.mj_viewer.terminate_safe()
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

    def _update(self, if_camera_preview=True):

        # Get current velocity of base for PID control
        # For Summit
        if (self._robot_list[0]):
            self.summit_currentx = self.mj_data.body(self.summit_base_name+"/base_link").cvel[3]
            self.summit_currenty = self.mj_data.body(self.summit_base_name+"/base_link").cvel[4]
            self.summit_currenttheta = self.mj_data.body(self.summit_base_name+"/base_link").cvel[2]
        
        # For Fetch
        if (self._robot_list[1]):
            self.fetch_currentx = self.mj_data.body(self.fetch_base_name+"/base_link").cvel[3]
            self.fetch_currenty = self.mj_data.body(self.fetch_base_name+"/base_link").cvel[4]
            self.fetch_currenttheta = self.mj_data.body(self.fetch_base_name+"/base_link").cvel[2]
        
        # For Forklift
        if (self._robot_list[2]):
            self.forklift_currentx = self.mj_data.body(self.forklift_base_name+"/base_link").cvel[3]
            self.forklift_currenty = self.mj_data.body(self.forklift_base_name+"/base_link").cvel[4]
            self.forklift_currenttheta = self.mj_data.body(self.forklift_base_name+"/base_link").cvel[2]

        # Set control commands by simple PID control defined in "control_commands.py"
        # For Summit
        if (self._robot_list[0]):
            self.summit_control_commands.velx_PID(25.0, 0.3, 1.3, self.summit_currentx,self.summit_base_name)   
            self.summit_control_commands.vely_PID(25.0, 0.3, 1.3, self.summit_currenty,self.summit_base_name)
            self.summit_control_commands.veltheta_PID(12.0, 0.3, 0.3, self.summit_currenttheta,self.summit_base_name)
        
        # For Fetch
        if (self._robot_list[1]):
            self.fetch_control_commands.velx_PID(25.0, 0.3, 1.3, self.fetch_currentx,self.fetch_base_name)   
            self.fetch_control_commands.vely_PID(25.0, 0.3, 1.3, self.fetch_currenty,self.fetch_base_name)
            self.fetch_control_commands.veltheta_PID(12.0, 0.3, 0.3, self.fetch_currenttheta,self.fetch_base_name)
        
        # For Forklift
        if (self._robot_list[2]):
            self.forklift_control_commands.velx_PID(25.0, 0.3, 1.3, self.forklift_currentx,self.forklift_base_name)   
            self.forklift_control_commands.vely_PID(25.0, 0.3, 1.3, self.forklift_currenty,self.forklift_base_name)
            self.forklift_control_commands.veltheta_PID(12.0, 0.3, 0.3, self.forklift_currenttheta,self.forklift_base_name)
        
        # stepping if needed
        if not self.mj_viewer.is_key_registered_to_pause_program_safe() or \
            self.mj_viewer.is_key_registered_to_step_to_next_safe():

            # - render current view:
            steps = round(1/self._rate_Hz/self.mj_model._model.opt.timestep)
            for i in range(steps):
                mujoco.mj_step(self.mj_model._model, self.mj_data._data)
            
            self.mj_viewer.reset_key_registered_to_step_to_next_safe()

        # process GUI interrupts
        self.mj_viewer.process_safe()

        # Update mj_viewer with specified frequency
        if self.i == 0:

            self.mj_viewer.update_safe()
            self.mj_viewer.render_safe()
            self.i=self.steps_per_render-1

            # Set "if_camera_preview" to True (input "_update" function) when you want to plot the cameras mounted on the WAM
            # Rendering of sensor cameras takes long!! Reduce update frequency to maintain real time simulation!
            if if_camera_preview:

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
                cv2.imshow("camera views", cv2.hconcat(cv2_capture_window))
                self.video.write(cv2.hconcat(cv2_capture_window))
                cv2.waitKey(int(1000/self._rate_Hz))

        self.i-=1

        # Publish link_states and joint_states
        self.state_pub.pub_joint_states()
        self.state_pub.pub_link_states()
        self.state_pub.pub_sensor_states()


        # Publish simulation time
        self.simtime.data = self.mj_data.time
        self.pub_time.publish(self.simtime)
        
    
