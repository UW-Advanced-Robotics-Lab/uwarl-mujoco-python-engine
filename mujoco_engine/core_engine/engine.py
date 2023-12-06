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

import rospy

# custom libraries:
import mujoco_viewer

# local libraries:
from mujoco_engine.core_engine.wrapper.core import MjData, MjModel

# import publisher and subscriber for migration with ros
from mujoco_engine.core_engine.state_pub_mujoco import StatePublisherMujoco
from mujoco_engine.core_engine.control_commands import ControlCommand

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
        "camera/zed/L": {"width": 1280, "height":720, "fps": 60, "id":1},
        "camera/zed/R": {"width": 1280, "height":720, "fps": 60, "id":0},
        "camera/intel/rgb": {"width": 1280, "height":720, "fps": 60, "id":2},
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
        CAMERA_V_FACTOR=3
    ):
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        ## Init Configs:
        if camera_config:
            self._camera_config = (camera_config) # override if given
        self._name = name
        self._rate_Hz = rate_Hz
        self._rate_scene = rate_scene

        # Calculate rendering freq
        self.steps_per_render = round(float(self._rate_Hz)/float(self._rate_scene))
        self.i = 0
        self.last_render = rospy.Time.now()

        ## Initiate MJ
        self.mj_model = MjModel.from_xml_path(xml_path=xml_path)
        self.mj_data = MjData(self.mj_model)
        self.state_pub = StatePublisherMujoco(self.mj_data, self.mj_model)
        self.control_commands = ControlCommand(self.mj_data)
        self.pub_time = rospy.Publisher('/simtime',Float64,queue_size=1)
        self.simtime = Float64()

        self.currentx = 0.0
        self.currenty = 0.0
        self.currenttheta = 0.0

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
        for camera, config in self._camera_config.items():
            self.h_min = int(min(config["height"]/CAMERA_V_FACTOR, self.h_min))

        
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

    def _update(self, if_camera_preview=False):

        # Get current velocity of base for PID control
        self.currentx = self.mj_data.body("smt/base_link").cvel[3]
        self.currenty = self.mj_data.body("smt/base_link").cvel[4]
        self.currenttheta = self.mj_data.body("smt/base_link").cvel[2]

        # Set control commands by simple PID control defined in "control_commands.py"
        self.control_commands.velx_PID(25.0, 0.3, 1.3, self.currentx)   
        self.control_commands.vely_PID(25.0, 0.3, 1.3, self.currenty)
        self.control_commands.veltheta_PID(12.0, 0.3, 0.3, self.currenttheta)
        
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
                for camera_name, camera_buf in camera_sensor_data["frame_buffer"].items():
                    img = cv2.cvtColor(camera_buf, cv2.COLOR_RGB2BGR)
                    img = cv2.flip(img, 0)
                    img = cv2.resize(img, (int(img.shape[1] * self.h_min / img.shape[0]), self.h_min))
                    cv2_capture_window.append(img)
                    
                cv2.imshow("camera views", cv2.hconcat(cv2_capture_window))

        self.i-=1

        # Publish link_states and joint_states
        self.state_pub.pub_joint_states()
        self.state_pub.pub_link_states()

        # Publish simulation time
        self.simtime.data = self.mj_data.time
        self.pub_time.publish(self.simtime)
        
    
