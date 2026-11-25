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
# 2026-11-14: simulation hold requested by a controller node ("/mujoco/hold", see "_hold_active")
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray   # 2026-11-23: the hold acknowledgement (/mujoco/hold_ack)
import os as _os
import time as _wall_time

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
from sensor_msgs.msg import CompressedImage

class _NoViewer:
    # 2026-11-19 (headless mode, see Mujoco_Engine.__init__ "headless"): stands in for the MuJoCo viewer when no window is opened. Every
    # viewer call the engine makes (process_safe, update_safe, render_safe, is_key_registered_to_pause_program_safe, ...) is a no-op that
    # returns False, so the engine never pauses on a key, never renders, and steps the physics exactly as with the viewer.
    def __getattr__(self, name):
        return lambda *args, **kwargs: False

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
    _viewport        = {"width": 1920, "height":1080}
    
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
        if_viewport_preview = True,
        viewport = None,
        headless = False
    ):
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
        # 2026-11-14: simulation hold. A controller node publishes True on "/mujoco/hold" when a result it needs at the current
        # simulated time (e.g. a Cart-MPC plan) has not arrived yet, and False when it has. While held, the engine does not step the
        # physics or run the base PID, but keeps publishing "/clock" (unchanged), so the ROS clock, which follows "/clock", is
        # paused too. Safety: a hold is released by the engine after "/mujoco/max_hold_wall_time" seconds of wall time
        # (default 30 s), so that a node that dies while holding cannot freeze the simulation. Nothing publishes on "/mujoco/hold"
        # unless a node is written to: older pipelines are unaffected.
        self._hold = False
        self._hold_since_wall = 0.0
        self._hold_timed_out = False
        self._max_hold_wall_time = float(rospy.get_param('/mujoco/max_hold_wall_time', 30.0))
        self._hold_sub = rospy.Subscriber('/mujoco/hold', Bool, self._hold_callback, queue_size=10)
        # 2026-11-23 (user: "Then we need a handshake. When a pause command is sent, a reply should be sent back saying that the pause
        # is in effect. For unpausing, a handshake may not be necessary." - step 1, measure only): when a hold request (True) arrives,
        # the simulated time of its arrival is noted; when the hold first takes effect in _update (the physics stops stepping), the
        # engine publishes "/mujoco/hold_ack" (std_msgs/Float64MultiArray) [n, t_request_arrived, t_hold_in_effect] (n = the hold's
        # number, 1, 2, ..; times = simulated seconds, mj_data.time) and appends the same row to "<ROS_LOG_DIR>/hold_ack.csv"
        # (/mujoco/hold_ack_log overrides the path; empty = no file). t_hold_in_effect - t_request_arrived = the simulated time that passed
        # between the request reaching the engine and the pause. NOTHING ELSE CHANGES: the physics, the hold itself and its release are
        # as before (no node waits for the acknowledgement yet). Example row: 17, 1.712, 1.713 (one 1 ms step stepped after the request).
        self._hold_n = 0
        self._hold_req_sim = None
        self._hold_ack_pub = rospy.Publisher('/mujoco/hold_ack', Float64MultiArray, queue_size=10)
        _ld = _os.environ.get('ROS_LOG_DIR', '')
        self._hold_ack_log = rospy.get_param('/mujoco/hold_ack_log', (_ld + '/hold_ack.csv') if _ld else '')
        if self._hold_ack_log:
            try:
                with open(self._hold_ack_log, 'w') as _f: _f.write('n,t_request_arrived,t_hold_in_effect\n')
            except Exception:
                self._hold_ack_log = ''
        ## Init Configs:
        if camera_config:
            self._camera_config = (camera_config) # override if given
        if viewport:
            self._viewport = viewport
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
        # 2026-11-19 (user: run several MuJoCo simulations at once; their rendering through one X display was the bottleneck: one run 0.3x
        # real time, three at once 0.1x each): headless = no viewer window, no viewport / camera capture, no video files, no image topics.
        # The physics, the controllers, "/clock" and the hold are unchanged. Off by default (existing launches as before); on with the
        # argument, or the rosparam "/mujoco/headless", or the environment variable MUJOCO_HEADLESS=1 (read in main.py).
        self.headless = bool(headless)
        if self.headless:
            self.if_viewport_preview = False
            self.if_camera_preview = False

        # Create publishers to publish camera view
        self.pub_rear_cam = rospy.Publisher('/mujoco/camera/compressed',CompressedImage,queue_size=1)
        # Create publishers to publish camera view
        self.pub_viewport = rospy.Publisher('/mujoco/viewport/compressed',CompressedImage,queue_size=1)
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
        # 2026-11-23 (step 14; user: caster start angles "60 and 120 deg sound reasonable" + "You also wanted 90 and 180 degs, right?"): the cart's
        # front casters can START swivelled. rosparam "/mujoco/caster_start_angle_deg" (degrees; unset or 0 = as before, both casters
        # trailing straight) sets the angle of both swivel joints "cart/C_swivel_hub" and "cart/C2_swivel_hub" (the jointpos sensors
        # "joint_pos_cart_swivel_C/C2"; theta relative to the cart, 0 = the wheel trailing behind its swivel axis, 180 = the wheel leading)
        # once, here, before the first physics step; mj_forward then makes positions, sensors and contacts consistent. E.g. 90: both wheels
        # sideways (+90 deg about the swivel axis, z up). Nothing else changes; the casters swivel freely from there.
        _ca = float(rospy.get_param('/mujoco/caster_start_angle_deg', 0.0) or 0.0)
        if _ca != 0.0:
            _m = self.mj_model._model; _d = self.mj_data._data
            for _jn in ('cart/C_swivel_hub', 'cart/C2_swivel_hub'):
                _j = mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_JOINT, _jn)
                if _j < 0: raise RuntimeError('MuJoCo engine: /mujoco/caster_start_angle_deg set but the model has no joint %s' % _jn)
                _d.qpos[_m.jnt_qposadr[_j]] = np.radians(_ca)
            mujoco.mj_forward(_m, _d)
            rospy.loginfo('MuJoCo engine: both front casters start at %.1f deg (swivel angle, 0 = trailing straight)' % _ca)
        # 2026-11-25 (user: "Go ahead." on putting the grip torque fitted on the robot into MuJoCo's cart): the tool holder's vertical swivel
        # "wam/tool/holder_yaw" (the two prongs cradling the handle turn freely about it; model: stiffness 0.5 N m/rad, damping 0.001) gets the
        # torsional stiffness and damping fitted on five robot turn-2 runs (cart_mpc step_tools/cart_yaw_replay_V3.py: the MPC's cart model
        # with the measured hand motion over-predicted the cart's rotation 1.3-1.5x; a torque -k (handle yaw - hand yaw) - c (rate difference)
        # brought it to 0.96-1.02 with k 40 N m/rad, c 5 N m s/rad). On the robot that spring is mostly the handle's flex (the handle end
        # bends back 2-3 cm in hard turns = 4-6 deg) plus the swivel's own stiffness; MuJoCo's handle is rigid, so all of it goes on the
        # swivel (springs in series with almost no mass between them). rosparam "/mujoco/grip_swivel" = [k (N m/rad), c (N m s/rad)]; unset =
        # the model's values, unchanged. The spring's rest angle stays the model's springref (0: prongs square to the wrist, as at the grasp).
        _gs = rospy.get_param('/mujoco/grip_swivel', None)
        if _gs:
            _m = self.mj_model._model
            _j = mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_JOINT, 'wam/tool/holder_yaw')
            if _j < 0: raise RuntimeError('MuJoCo engine: /mujoco/grip_swivel set but the model has no joint wam/tool/holder_yaw')
            _k, _c = float(_gs[0]), float(_gs[1])
            _old = (float(_m.jnt_stiffness[_j]), float(_m.dof_damping[_m.jnt_dofadr[_j]]))
            _m.jnt_stiffness[_j] = _k; _m.dof_damping[_m.jnt_dofadr[_j]] = _c
            rospy.loginfo('MuJoCo engine: grip swivel wam/tool/holder_yaw stiffness %.3f -> %.3f N m/rad, damping %.4f -> %.4f N m s/rad' % (_old[0], _k, _old[1], _c))
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
        if self.headless:   # 2026-11-19: no window; the stand-in answers every viewer call with a no-op
            self.mj_viewer = _NoViewer()
            rospy.loginfo("MuJoCo engine: headless (no viewer, no rendering)")
        else:
          self.mj_viewer = mujoco_viewer.MujocoViewer(self.mj_model._model, self.mj_data._data, 
            title="Mujoco-Engine", 
            sensor_config=self._camera_config,
            window_size=(self._viewport["width"], self._viewport["height"]),
        )
        # if len(self._camera_config):
        #     self.mj_viewer_off = mujoco_viewer.MujocoViewer(self.mj_model, self.mj_data, width=800, height=800, title="camera-view")
        # self._t_update = time.time()
        
        # cv2 window (2026-11-19: not in headless mode)
        if not self.headless:
            cv2.startWindowThread()
        self.h_min = np.Infinity
        self.width = 0
        for camera, config in self._camera_config.items():
            self.h_min = int(min(config["height"]/CAMERA_V_FACTOR, self.h_min))
            # Sum up width
            self.width += config["width"]
        
        # Start video capture (2026-11-19: not in headless mode, where nothing is captured; several engines at once would also all
        # write the same two files)
        if self.headless:
            self.camera_video = None
            self.viewport_video = None
        else:
            self.camera_video = cv2.VideoWriter(self._write_to+'/camera.avi',  
                                                cv2.VideoWriter_fourcc(*'MJPG'), 
                                                self._rate_scene, (self.width,self.h_min))
            self.viewport_video = cv2.VideoWriter(self._write_to+'/viewport.avi',  
                                                  cv2.VideoWriter_fourcc(*'MJPG'), 
                                                  self._rate_scene, (self._viewport["width"], self._viewport["height"]))

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

    def _apply_caster_scrub(self):
        # 2026-11-15 (user: the turning resistance measured on the hardware): scrub torque of the cart's swivel casters, fitted on the robot
        # (run 2026_11_04_0): M = M0 * w * a / sqrt((w a)^2 + v^2 + eps^2) per caster, w the caster hub's absolute yaw rate (rad/s), v its
        # wheel's rolling speed (m/s): full M0 at rest, fading while the wheel rolls. Applied as an external torque about the world vertical on
        # each hub body (as the ground's friction), opposing w. Active only when the model has the custom numerics "caster_scrub_M0" (N m per
        # caster), "caster_scrub_a" (m), "caster_scrub_eps" (m/s), written by "cart_mpc/script/mujoco_cart_V4.py"; hub / wheel bodies
        # "cart/swivel_hub/C_link" + "cart/whl/FC_link" and, if present, "cart/swivel_hub/C2_link" + "cart/whl/FC2_link".
        if not hasattr(self, '_scrub'):
            m = self.mj_model._model; self._scrub = None
            try:
                M0 = float(m.numeric('caster_scrub_M0').data[0]); a = float(m.numeric('caster_scrub_a').data[0]); eps = float(m.numeric('caster_scrub_eps').data[0])
            except Exception:
                return
            pairs = []
            for hub, whl in (('cart/swivel_hub/C_link', 'cart/whl/FC_link'), ('cart/swivel_hub/C2_link', 'cart/whl/FC2_link')):
                try: pairs.append((m.body(hub).id, m.body(whl).id))
                except Exception: pass
            # 2026-11-18 (user: step 12, "make the plant follow the fitted law"): optional "caster_scrub_law" = 1: the cart-rate law of the MPC
            # ("cart_mpc_V30"), one equivalent centred caster on the CART's yaw rate, applied as a yaw torque on "cart/base_link":
            # Where the caster trail d is used (2026-11-19, user: "the original cart model never cared about trail-length ... add the explanation
            # on where the trail-length value is being actively used: in deriving the caster's rate of rotation w.r.t the cart's yaw-rate"):
            # the caster acts on the cart only through the force at its pivot (massless, free swivel: no torque passes the swivel axis), of size
            # M / d (the scrub torque M at the contact balanced by the side force times the trail). So in the FORCE the trail only appears as the
            # ratio M0 / d, which a fit or an estimator cannot split: it identifies M0 / d (a force scale, N). The trail is ACTIVELY used in one place
            # only: the caster's rate of rotation relative to the cart, from the no-lateral-slip kinematics of the pivot A = (L, y_c):
            #     phi_dot = (w L cos(theta) - (v - w y_c) sin(theta)) / d   (the wheel's world heading rate),   theta_dot = phi_dot - w
            # (theta = the wheel's rolling direction relative to the cart's x axis). It sets how fast a caster realigns (e.g. after a reversal),
            # hence the direction of its force (perpendicular to the wheel) and its own swivel rate, which the per-caster scrub law uses.
            # In this cart-rate law d enters only as L / trail (M0 / trail). In the per-hub law (MuJoCo's swivel joints) the caster's own
            # swivel rate comes from the simulated caster itself, whose physical trail (the cart model's caster wheel offset) is what d acts through.
            #   M_cart = -M0 (L / trail) w a / sqrt((w a)^2 + v_x^2 + v_y^2 + (L w)^2 + eps^2),  w the cart's yaw rate, (v_x, v_y) its origin's
            #   velocity, L "caster_scrub_swivel_ahead" (m), trail "caster_scrub_trail" (m), M0 for both casters together. The hubs get no torque.
            # Missing or 0: the per-hub law above (the default, unchanged).
            law = 0; L = trail = 0.0; base = -1
            try: law = int(round(float(m.numeric('caster_scrub_law').data[0])))
            except Exception: law = 0
            if law == 1:
                L = float(m.numeric('caster_scrub_swivel_ahead').data[0]); trail = float(m.numeric('caster_scrub_trail').data[0]); base = m.body('cart/base_link').id
            self._scrub = (M0, a, eps, pairs, np.zeros(6), law, L, trail, base)
            if law == 1:
                rospy.loginfo("MuJoCo engine: caster scrub, cart-rate law: M0 %.3f N m (both casters), a %.3f m, eps %.4f m/s, swivel %.3f m ahead, trail %.3f m" % (M0, a, eps, L, trail))
            else:
                rospy.loginfo("MuJoCo engine: caster scrub M0 %.3f N m per caster, a %.3f m, eps %.4f m/s on %d casters" % (M0, a, eps, len(pairs)))
        if self._scrub is None:
            return
        M0, a, eps, pairs, v6, law, L, trail, base = self._scrub; m = self.mj_model._model; d = self.mj_data._data
        if law == 1:
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, base, v6, 0); w = v6[2]; vx, vy = v6[3], v6[4]
            d.xfrc_applied[base, 3:6] = [0.0, 0.0, -M0*(L/trail)*w*a/np.sqrt((w*a)**2 + vx**2 + vy**2 + (L*w)**2 + eps**2)]
            return
        fac = self._scrub_noise_factors(float(d.time), len(pairs))   # 2026-11-24: 1 for every caster unless /mujoco/caster_scrub_noise is set
        for i_c, (hb, wb) in enumerate(pairs):
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, hb, v6, 0); w = v6[2]
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, wb, v6, 0); v = np.hypot(v6[3], v6[4])
            d.xfrc_applied[hb, 3:6] = [0.0, 0.0, -M0*fac[i_c]*w*a/np.sqrt((w*a)**2 + v**2 + eps**2)]

    def _scrub_noise_factors(self, t, n):
        # 2026-11-24 (user: "Can we vary the sliding coefficient to encourage flipping the casters noisily? That might show which
        # stretch-strategy is robust." -> "Prioritize these trials"): optional NOISY CASTER SCRUB, in the plant only (the MPC's model keeps the
        # constant M0). rosparam "/mujoco/caster_scrub_noise" = [sigma, tau_s, seed]: each caster's scrub M0 is multiplied by
        # f_i(t) = max(0, 1 + n_i(t)), n_i an Ornstein-Uhlenbeck process (mean 0, standard deviation sigma, correlation time tau_s), independent
        # per caster, sampled on a 10 ms grid of simulated time from the controller's first hold request (= its first MPC call, as the base
        # stalls), drawn from numpy's default_rng(seed): the SAME seed gives the SAME disturbance in every run, whatever the controller does.
        # E.g. [0.5, 0.3, 1]: M0 varies by about +-50 % (clipped at 0), changing over about 0.3 s; a caster with f near 0 swivels freely, one
        # with f near 2 sticks, so the casters stick and flip unevenly. Unset (default) or before the first hold request: f = 1 (as before).
        if not hasattr(self, '_snoise'):
            self._snoise = None
            p_ = rospy.get_param('/mujoco/caster_scrub_noise', None)
            if p_:
                sig, tau, seed = float(p_[0]), float(p_[1]), int(p_[2]); dt = 0.01; a_ = float(np.exp(-dt / tau))
                self._snoise = dict(sig=sig, a=a_, b=sig*float(np.sqrt(1.0 - a_*a_)), dt=dt, rng=np.random.default_rng(seed),
                                    seq=[sig*np.random.default_rng(seed + 1000003).standard_normal(n)])   # n(0) from the stationary law
                rospy.loginfo("MuJoCo engine: noisy caster scrub: M0 x max(0, 1 + n(t)), sigma %.2f, correlation %.2f s, seed %d, %d casters, from the first hold request"
                              % (sig, tau, seed, n))
        S_ = self._snoise; t0 = getattr(self, '_noise_t0_sim', None)
        if S_ is None or t0 is None or t < t0:
            return np.ones(n)
        k = int((t - t0) / S_['dt'])
        while len(S_['seq']) <= k:
            S_['seq'].append(S_['a']*S_['seq'][-1] + S_['b']*S_['rng'].standard_normal(n))
        return np.maximum(0.0, 1.0 + S_['seq'][k])

    def _log_yaw_forces(self):
        # 2026-11-18 (user: "do it" = find what loads the base's yaw; "confirm that the prongs are free to rotate about its yaw-axis"): optional
        # log, OFF unless the environment variable MJ_YAW_FORCE_LOG names a CSV file. Every 10 physics steps (10 ms) one row:
        #   base yaw joint "smt/orie/z": rate, and its generalized force by source (N m): actuator, passive (damping), constraint (all
        #   constraint rows: contacts, joint friction, limits, equalities), bias (Coriolis, gravity), inertial (M qacc, all joints' coupling);
        #   the contacts' share split by what the robot touches OUTSIDE the yaw joint's subtree: torque about the yaw axis (through the joint's
        #   anchor) of the contact force on the robot side, for wheel-ground, tool-handle (geom names "tool/...") and any other pair;
        #   tool holder joint "wam/tool/holder_yaw": angle, rate, passive torque (spring + damping) and constraint torque (its frictionloss and
        #   any contact rows): what the free joint can transmit.
        # The names of the "other" contact pairs go to <file>.other.txt. The simulation is not changed (read-only).
        if not hasattr(self, '_yawlog'):
            self._yawlog = None
            path = os.environ.get('MJ_YAW_FORCE_LOG', '')
            if not path:
                return
            m = self.mj_model._model
            jz = m.joint('smt/orie/z').id; jh = m.joint('wam/tool/holder_yaw').id
            dz, dh = int(m.jnt_dofadr[jz]), int(m.jnt_dofadr[jh])
            bz = int(m.jnt_bodyid[jz]); sub = np.zeros(m.nbody, dtype=bool)
            for b in range(m.nbody):
                c = b
                while c > 0 and c != bz: c = int(m.body_parentid[c])
                sub[b] = (c == bz)
            f = open(path, 'w')
            f.write('t,yaw_rate,act,passive,constraint,bias,inertial,contact_wheels,contact_tool,contact_other,holder_q,holder_qd,holder_passive,holder_constraint\n')
            self._yawlog = dict(f=f, path=path, jz=jz, dz=dz, dh=dh, qh=int(m.jnt_qposadr[jh]), sub=sub, n=0, other={}, f6=np.zeros(6), mq=np.zeros(m.nv))
            rospy.loginfo("MuJoCo engine: base yaw force log -> %s" % path)
        L = self._yawlog
        if L is None:
            return
        L['n'] += 1
        if L['n'] % 10:
            return
        m = self.mj_model._model; d = self.mj_data._data; dz, dh = L['dz'], L['dh']
        anchor = d.xanchor[L['jz']]; axis = d.xaxis[L['jz']]
        tw = tt = to = 0.0
        for i in range(d.ncon):
            con = d.contact[i]
            b1, b2 = int(m.geom_bodyid[con.geom1]), int(m.geom_bodyid[con.geom2])
            s1, s2 = L['sub'][b1], L['sub'][b2]
            if s1 == s2:
                continue
            mujoco.mj_contactForce(m, d, i, L['f6'])
            Fw = con.frame.reshape(3, 3).T @ L['f6'][0:3]        # force of geom1 on geom2 (world), along the normal geom1 -> geom2
            F_robot = Fw if s2 else -Fw
            tz = float(np.dot(np.cross(con.pos-anchor, F_robot), axis))
            n1, n2 = m.geom(con.geom1).name, m.geom(con.geom2).name
            if 'whl' in n1 or 'whl' in n2: tw += tz
            elif n1.startswith('tool/') or n2.startswith('tool/'): tt += tz
            else:
                to += tz; key = n1+' | '+n2; L['other'][key] = L['other'].get(key, 0.0)+tz
        mujoco.mj_mulM(m, d, L['mq'], d.qacc)
        L['f'].write('%.4f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f\n' % (
            d.time, d.qvel[dz], d.qfrc_actuator[dz], d.qfrc_passive[dz], d.qfrc_constraint[dz], d.qfrc_bias[dz], L['mq'][dz], tw, tt, to,
            d.qpos[L['qh']], d.qvel[dh], d.qfrc_passive[dh], d.qfrc_constraint[dh]))
        if L['n'] % 10000 == 0:
            L['f'].flush()
            with open(L['path']+'.other.txt', 'w') as g:
                for k, v in sorted(L['other'].items(), key=lambda kv: -abs(kv[1])): g.write('%+.3f  %s\n' % (v*0.01, k))

    def _hold_callback(self, msg):
        # "/mujoco/hold" (std_msgs/Bool): True = pause the simulation, False = resume
        if msg.data and not self._hold:
            self._hold_since_wall = _wall_time.time()
            self._hold_timed_out = False
            self._hold_req_sim = float(self.mj_data.time)   # 2026-11-23: the acknowledgement's request time (simulated)
            # 2026-11-24: the first hold request (= the controller's first MPC call) is t = 0 of the simulated base stalls
            # (/mujoco/base_stall_intervals, control_commands.vel_PID)
            _cc = getattr(self, 'summit_control_commands', None)
            if _cc is not None and getattr(_cc, 'stall_t0', None) is None:
                _cc.stall_t0 = rospy.Time().now().to_time()
            if getattr(self, '_noise_t0_sim', None) is None:   # 2026-11-24: t = 0 of the noisy caster scrub (/mujoco/caster_scrub_noise)
                self._noise_t0_sim = float(self.mj_data.time)
        self._hold = bool(msg.data)

    def _hold_active(self):
        # True while a hold is requested and has not exceeded "/mujoco/max_hold_wall_time" (wall time)
        if not self._hold:
            return False
        if (_wall_time.time()-self._hold_since_wall) > self._max_hold_wall_time:
            if not self._hold_timed_out:
                rospy.logwarn("MuJoCo engine: hold requested on /mujoco/hold for more than %.1f s (wall time): released" % self._max_hold_wall_time)
                self._hold_timed_out = True
            return False
        return True

    def _update(self):

        # 2026-11-14: while a hold is active, neither step the physics nor run the controllers that integrate over time (base
        # velocity PID: its time-step would be ~0 with the clock paused); keep the GUI responsive and keep publishing "/clock".
        if self._hold_active():
            if self._hold_req_sim is not None:   # 2026-11-23: the first update with the hold in effect -> acknowledge it once
                self._hold_n += 1
                _row = [float(self._hold_n), self._hold_req_sim, float(self.mj_data.time)]
                self._hold_req_sim = None
                self._hold_ack_pub.publish(Float64MultiArray(data=_row))
                if self._hold_ack_log:
                    try:
                        with open(self._hold_ack_log, 'a') as _f: _f.write('%d,%.4f,%.4f\n' % tuple(_row))
                    except Exception:
                        pass
            self.mj_viewer.process_safe()
            sim_time_msg = Clock()
            sim_time_msg.clock = rospy.Time.from_sec(self.mj_data.time)
            self.clock_pub.publish(sim_time_msg)
            _wall_time.sleep(0.001)
            return

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
            # 2026-11-18 (user: "tune the MB PIDs if you need to"): gains from the rosparam "/mujoco/summit_vel_pid" ([Kp x, y, yaw, Ki x, y, yaw,
            # Kd x, y, yaw]), read once; missing = the gains below (unchanged). Why: with the yaw Kd 0.3 the yaw loop chatters at the actuator
            # limit (D = Kd (e - e_last) / dt at 1 kHz, gear 50, yaw inertia ~6 kg m^2: each step the D term answers a rate change with ~2.5
            # times it in the other direction -> +-500 N m square wave in 91 % of steps, run YL10M); the base then turns on the average of
            # that wave: gain ~0.75, lagging ("qpd/base_response_fit.py"). Offline ("yawload/pid_tune.py", the playground, the engine's PID on
            # the model's sensors): yaw Kp 12, Ki 10, Kd 0.05 -> no saturation, gain 0.997, 63 % rise 0.022 s, no overshoot. x / y unchanged
            # (stable: gain 0.995, rise 0.13 s).
            if not hasattr(self, '_summit_pid'):
                g = rospy.get_param('/mujoco/summit_vel_pid', None)
                self._summit_pid = ([25.0,25.0,12.0], [0.3,0.3,0.3], [1.3,1.3,0.3])
                if g is not None:
                    g = [float(v) for v in g]; self._summit_pid = (g[0:3], g[3:6], g[6:9])
                rospy.loginfo("MuJoCo engine: Summit velocity PID Kp %s Ki %s Kd %s%s" % (self._summit_pid + ('' if g is None else ' (/mujoco/summit_vel_pid)',)))
            self.summit_control_commands.vel_PID(self._summit_pid[0], 
                                                 self._summit_pid[1], 
                                                 self._summit_pid[2], 
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
                self._apply_caster_scrub()   # 2026-11-15: caster scrub torque (only if the model defines it)
                mujoco.mj_step(self.mj_model._model, self.mj_data._data)
                self._log_yaw_forces()   # 2026-11-18: optional base yaw force log (MJ_YAW_FORCE_LOG; off by default)
            
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
                    img = cv2.resize(img, (self._viewport["width"], self._viewport["height"]))
                    self.viewport_video.write(img)

                    # Prepare CompressedImage message
                    viewport_img_msg = CompressedImage()
                    viewport_img_msg.header.stamp = rospy.Time.now()
                    viewport_img_msg.header.frame_id = "camera_link"
                    viewport_img_msg.format = "jpeg"

                    # Compress the frame to JPEG (Quality 80 is usually the sweet spot)
                    # This turns the 6MB raw frame back into a small JPEG buffer
                    success, encoded_img = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    
                    if success:
                        viewport_img_msg.data = np.array(encoded_img).tobytes()
                        self.pub_viewport.publish(viewport_img_msg)

                    # viewport_cam = self.bridge.cv2_to_imgmsg(img, "bgr8")
                    # viewport_cam.header.frame_id = "rear"
                    # # Current time
                    # curr_time = rospy.Time.now()
                    # viewport_cam.header.stamp = curr_time
                    # self.pub_viewport.publish(viewport_cam)
                    
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

                # Prepare CompressedImage message
                camera_img_msg = CompressedImage()
                camera_img_msg.header.stamp = rospy.Time.now()
                camera_img_msg.header.frame_id = "camera_link"
                camera_img_msg.format = "jpeg"

                # Compress the frame to JPEG (Quality 80 is usually the sweet spot)
                # This turns the 6MB raw frame back into a small JPEG buffer
                success, encoded_img = cv2.imencode('.jpg', hoz_cat_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                
                if success:
                    camera_img_msg.data = np.array(encoded_img).tobytes()
                    self.pub_rear_cam.publish(camera_img_msg)

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
                        img = cv2.resize(img, (self._viewport["width"], self._viewport["height"]))
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
        
    
