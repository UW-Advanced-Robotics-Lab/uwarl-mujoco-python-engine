#! /usr/bin/env python

# Subscribers for reading control commands from hw_interface and writing them into mj_data to be used next engine step
# Last version: Nov 28, 2023 Tim van Meijel

import numpy as np
import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist
from uwarl_mujoco_ros_msgs.msg import MB_pid_effort
from sensor_msgs.msg import JointState


class ControlCommand(object):

    def __init__(self,mj_data,cmd_vel_topic_name):
        # Base velocity effort control
        self.sub_vel_base_summ = rospy.Subscriber(cmd_vel_topic_name, Twist, self.vel_base_callback)

        # Create pointer to mujoco data
        # This contains references to joints which have actuators associated with them (they are mentioned in files similar to `include_summit_wam_bhand_actuators.xml`)
        self.mj_data_control = mj_data

        # Initialize PID variables for base velocity effort control
        self.I_x = 0.0
        self.e_x_last = 0.0
        self.last_time_x = rospy.Time().now().to_time()

        self.I_y = 0.0
        self.e_y_last = 0.0
        self.last_time_y = rospy.Time().now().to_time()

        self.I_theta = 0.0
        self.e_theta_last = 0.0
        self.last_time_theta = rospy.Time().now().to_time()

        self.vel_base = [0.0, 0.0, 0.0]

        # Summit H-matrix
        gamma_smt = 45/180*np.pi
        dx_1_smt = 0.222
        dx_2_smt = 0.223
        dy_1_smt = 0.22
        dy_2_smt = 0.22
        whl_rad_smt = 0.127
        tan_gamma_smt = np.tan(gamma_smt)
        self.h_mat_smt = np.array([[1,  tan_gamma_smt, -dy_1_smt+dx_1_smt*tan_gamma_smt],
                                   [1, -tan_gamma_smt,  dy_2_smt-dx_1_smt*tan_gamma_smt],
                                   [1,  tan_gamma_smt,  dy_2_smt-dx_2_smt*tan_gamma_smt],
                                   [1, -tan_gamma_smt, -dy_1_smt+dx_2_smt*tan_gamma_smt]])/whl_rad_smt
        self.e_whl_last_smt = np.zeros((4,1))
        self.last_time_whl_smt = rospy.Time().now().to_time()
        self.I_whl_smt = np.zeros((4,1))

        # Record MB PID contribution
        self.mb_pid_contri = rospy.Publisher(
            '/mb_pid_contri',
            MB_pid_effort,
            queue_size=1
        )
        # Keep track of the sequence
        self.mb_pid_seq_ind = 0


    def vel_base_callback(self, msg):
        # Store data in array to be used by PID control
        # New velocity commands are received at approximately 50 Hz, PID loop runs at 200 Hz
        self.vel_base = [msg.linear.x, msg.linear.y, msg.angular.z]
    
    def vel_PID(self, Kp, Ki, Kd, CP, yaw_angle, base_name):

        # The MuJoCo-model of the mobile agents are not treated as a floating-base, but rather, as an actuated platform w.r.t
        # the world frame. This means that the actuators pose are w.r.t the world-frame, not the mobile base.

        # So, if the MB was initially tilted, then the actuators, which are fixed to the ground, are also tilted.
        # This initial-angle is subtracted from the current yaw-angle to get the above provided yaw-angle.

        # 2026-11-20 (user: "Yesterday, you had tuned the MB yaw-rate PIDs to match the hardware. Can you do the same for the MB lateral and
        # longitudinal velocities as well?" and "That should constrain what the MB can exert, and not just come from the OCPs limits."): two
        # optional features, read once from rosparams; both OFF when the parameters are missing (the behaviour before, unchanged):
        #   /mujoco/summit_accel_limit [a_x, a_y, a_yaw] (m/s^2, m/s^2, rad/s^2): the commanded base velocity (self.vel_base, base frame, from
        #       cmd_vel) is passed through a rate limiter per base axis before the PID, as the robot's base controller does (the real
        #       Summit speeds up at no more than ~0.6 m/s^2 in x and y: Vicon, runs 2026-11-02 .. 11-05). Each call the limited set-point
        #       moves towards the command by at most a * dt (dt: the time since the last call). E.g. a step 0 -> 0.3 m/s with 0.6 m/s^2
        #       becomes a 0.5 s ramp.
        #   /mujoco/summit_pid_base_frame (bool): the x / y PID acts in the BASE frame (errors rotated into the base frame, P / I / D per base
        #       axis, the two forces rotated back to the world-fixed actuators). Needed when x and y have different gains (Kp_y != Kp_x):
        #       in the world-aligned frame (the default) the gain would follow the world axes, so a base turned by 90 deg would get Kp_x on
        #       its lateral motion. With equal gains both frames give the same forces (P and D exactly; I up to its rotation history).
        if not hasattr(self, '_acc_lim'):
            try:
                a = rospy.get_param('/mujoco/summit_accel_limit', None); self._acc_lim = None if a is None else [float(v) for v in a]
            except Exception:
                self._acc_lim = None
            try: self._pid_base_frame = bool(rospy.get_param('/mujoco/summit_pid_base_frame', False))
            except Exception: self._pid_base_frame = False
            self._sp_lim = [0.0, 0.0, 0.0]; self._sp_t_last = None
            rospy.loginfo("MuJoCo engine: base set-point acceleration limit %s, PID in the %s frame" % (self._acc_lim, 'base' if self._pid_base_frame else 'world-aligned'))
        vel_cmd = list(self.vel_base)
        if self._acc_lim is not None:
            t_now = rospy.Time().now().to_time()
            dt_sp = 0.0 if self._sp_t_last is None else max(t_now - self._sp_t_last, 0.0); self._sp_t_last = t_now
            for k in range(3):
                # 2026-11-21 (step 13, user: "divide step 13 into that many parts ... add a switch"): an axis whose limit is <= 0 is NOT limited
                # (its set-point follows the command exactly), so each axis can be switched on alone, e.g. [0, 0, 1.5] = only the yaw ceiling.
                if self._acc_lim[k] <= 0.0:
                    self._sp_lim[k] = vel_cmd[k]
                    continue
                step = self._acc_lim[k]*dt_sp
                self._sp_lim[k] += min(max(vel_cmd[k] - self._sp_lim[k], -step), step)
            vel_cmd = list(self._sp_lim)
        # Convert the local velocities into the stationary initial base-frame.
        SP_x = vel_cmd[0]*cos(yaw_angle)-vel_cmd[1]*sin(yaw_angle)
        SP_y = vel_cmd[0]*sin(yaw_angle)+vel_cmd[1]*cos(yaw_angle)
        SP_theta = vel_cmd[2]

        # Current base velocity in initial base-frame.
        CP_x = CP[0]*cos(yaw_angle)-CP[1]*sin(yaw_angle)
        CP_y = CP[0]*sin(yaw_angle)+CP[1]*cos(yaw_angle)
        CP_theta = CP[2]

        t = rospy.Time().now().to_time()
        e_x = float(SP_x - CP_x)
        e_y = float(SP_y - CP_y)
        if self._pid_base_frame:
            # errors in the base frame (x forward, y left): the gains below act per base axis
            e_x, e_y = e_x*cos(yaw_angle)+e_y*sin(yaw_angle), -e_x*sin(yaw_angle)+e_y*cos(yaw_angle)
        e_theta = float(SP_theta - CP_theta)
        # if(base_name is "fetch"):
        #     print("SP theta vel: "+str(SP_theta))
        #     print("CP theta vel: "+str(CP_theta))


        # Compute PID variables
        P_x = e_x*Kp[0]
        P_y = e_y*Kp[1]
        P_theta = e_theta*Kp[2]

        delta_t = t-self.last_time_theta+0.0001

        self.I_x += Ki[0]*e_x*delta_t
        self.I_y += Ki[1]*e_y*delta_t
        self.I_theta += Ki[2]*e_theta*delta_t

        D_x = Kd[0]*(e_x-self.e_x_last)/delta_t
        D_y = Kd[1]*(e_y-self.e_y_last)/delta_t
        D_theta = Kd[2]*(e_theta-self.e_theta_last)/delta_t

        self.e_x_last = e_x
        self.e_y_last = e_y
        self.e_theta_last = e_theta

        self.last_time_theta = t

        control_x = P_x+self.I_x+D_x
        control_y = P_y+self.I_y+D_y
        if self._pid_base_frame:
            # the base-frame forces back to the world-fixed actuators
            control_x, control_y = control_x*cos(yaw_angle)-control_y*sin(yaw_angle), control_x*sin(yaw_angle)+control_y*cos(yaw_angle)
        control_theta = P_theta+self.I_theta+D_theta

        # Set control commands in mj_data
        self.mj_data_control.actuator(base_name+'/pose/x').ctrl = control_x
        self.mj_data_control.actuator(base_name+'/pose/y').ctrl = control_y
        self.mj_data_control.actuator(base_name+'/orie/z').ctrl = control_theta

        # Record the PID contributions
        mb_pid_effort_obj = MB_pid_effort()
        mb_pid_effort_obj.header.seq = self.mb_pid_seq_ind
        mb_pid_effort_obj.header.stamp = rospy.Time.now()
        mb_pid_effort_obj.header.frame_id = "global-frame"
        # Link name
        mb_pid_effort_obj.link_name = base_name
        # Current MB vel
        mb_pid_effort_obj.curr_mb_vel.append(CP_x)
        mb_pid_effort_obj.curr_mb_vel.append(CP_y)
        mb_pid_effort_obj.curr_mb_vel.append(CP_theta)
        # Set-point MB vel
        mb_pid_effort_obj.dem_mb_vel.append(SP_x)
        mb_pid_effort_obj.dem_mb_vel.append(SP_y)
        mb_pid_effort_obj.dem_mb_vel.append(SP_theta)
        # P-contribution
        mb_pid_effort_obj.p_contribution.append(P_x)
        mb_pid_effort_obj.p_contribution.append(P_y)
        mb_pid_effort_obj.p_contribution.append(P_theta)
        # I-contribution
        mb_pid_effort_obj.i_contribution.append(self.I_x)
        mb_pid_effort_obj.i_contribution.append(self.I_y)
        mb_pid_effort_obj.i_contribution.append(self.I_theta)
        # D-contribution
        mb_pid_effort_obj.d_contribution.append(D_x)
        mb_pid_effort_obj.d_contribution.append(D_y)
        mb_pid_effort_obj.d_contribution.append(D_theta)

        # Publish the qp inputs
        self.mb_pid_contri.publish(mb_pid_effort_obj)

        self.mb_pid_seq_ind +=1
    
    # PID loop to control wheel velocities
    def wheel_PID(self, Kp, Ki, Kd, CP, base_name):

        # Convert reference velocities into mapframe
        temp_mb_twist = np.array(self.vel_base)
        mb_twist = temp_mb_twist.reshape(3,1)

        # Wheel velocity: Set point
        whl_vel_SP = np.zeros((4,1))
        if(base_name == "smt"):
            whl_vel_SP = np.matmul(self.h_mat_smt,mb_twist)

        t = rospy.Time().now().to_time()
        # print("Wheel velocity set-point:")
        # print(whl_vel_SP)
        # print(CP)
        e_whl = whl_vel_SP - CP

        # Compute PID variables
        P = e_whl*Kp
        self.I_whl_smt += Ki*e_whl*(t-self.last_time_whl_smt+0.0001)
        D = Kd*(e_whl-self.e_whl_last_smt)/(t-self.last_time_whl_smt+0.0001)

        self.e_whl_last_smt = e_whl
        self.last_time_whl_smt = t

        control = P+self.I_whl_smt+D

        # print(P)
        # print(self.I_whl_smt)
        # print(D)
        # Set control commands in mj_data
        self.mj_data_control.actuator(base_name+'/whl_LF').ctrl = control[0,0]
        self.mj_data_control.actuator(base_name+'/whl_RF').ctrl = control[1,0]
        self.mj_data_control.actuator(base_name+'/whl_RR').ctrl = control[2,0]
        self.mj_data_control.actuator(base_name+'/whl_LR').ctrl = control[3,0]
