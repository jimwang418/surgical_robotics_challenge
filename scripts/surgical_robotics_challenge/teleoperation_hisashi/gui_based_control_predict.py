#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
import sys
from surgical_robotics_challenge.kinematics.psmIK import *
from ambf_client import Client
from surgical_robotics_challenge.psm_arm import PSM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from surgical_robotics_challenge.utils.obj_control_gui import ObjectGUI
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
from std_msgs.msg import Bool

### Prediction using KalmanFilter ###
import numpy as np
from pykalman import KalmanFilter


dt = 1e-10

class KFPredict:
    def __init__(self, observation):
        
        self.transition_matrix = [
            [1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0, 0],
            [0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt, 0],
            [0, 0, 1, 0, 0, dt, 0, 0, 0.5 * dt * dt],
            [0, 0, 0, 1, 0, 0, dt, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, dt, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]
        observation_matrix = [
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]
        initial_state_covariance = 0.01 * np.ones([9, 9])
        initial_observation_covariance = 0.01 * np.ones([9, 9])

        self.kf = KalmanFilter(
            transition_matrices=self.transition_matrix,
            observation_matrices=observation_matrix,
            initial_state_mean=observation,
            initial_state_covariance=initial_state_covariance,
            observation_covariance=initial_observation_covariance,
        )
    Closin
        mean, covariance = self.kf.filter_update(
            filtered_state_mean=state,
            filtered_state_covariance=cov,
            observation=self.transition_matrix @ state, 
        )
        test_observation_covariance = 0.1 * np.ones([9,9])
        #covariance = covariance + (1.0 - np.exp(-(t - t_change) / 10.0)) * test_observation_covariance
        covariance = covariance + (1.0 - np.exp(-(2.0) / 10.0)) * test_observation_covariance

        return mean, covariance


class PSMController:
    def __init__(self, gui_handle, arm, ghost=False):
        self.counter = 0
        self.GUI = gui_handle
        self.arm = arm

        self.ghost = ghost
        self.communication_loss = False
        self._T_t_b = Frame(Rotation.RPY(3.14, 0, 1.57079), Vector(0, 0, -1.0))

        self.pos = np.zeros([1,3]) #[0,0,0]
        self.pos_prev = np.zeros([1,3]) #[0,0,0]
        self.vel = np.zeros([1,3]) #[0,0,0]
        self.vel_prev = np.zeros([1,3]) #[0,0,0]
        self.acc = np.zeros([1,3]) #[0,0,0]

    def update_arm_pose(self):
        self.pos = np.array([self._T_t_b.p.x(),self._T_t_b.p.y(),self._T_t_b.p.z()])
        self.pos = self.pos.reshape(1,3)
        self.vel = (self.pos - self.pos_prev) / dt
        self.acc = (self.vel - self.vel_prev) / dt

        self.pos_prev = self.pos
        self.vel_prev = self.vel
        
        observation = np.hstack([self.pos,self.vel,self.acc]) #(1,9)
        

        kf = KFPredict(observation)

        if (self.ghost == True and self.communication_loss == True):
            # Predict using KF
            mean, cov = kf.predict(observation, 0.01 * np.ones([9, 9]))
            self._T_t_b = Frame(self._T_t_b.M, Vector(mean[0], mean[1], mean[2]))


        else:
            gui = self.GUI
            gui.App.update()
            self._T_t_b = Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
            self.arm.set_jaw_angle(gui.gr)
            self.arm.run_grasp_logic(gui.gr)
        self.arm.servo_cp(self._T_t_b)
        

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.arm.target_IK is not None:
            gui = self.GUI
            T_ik_w = self.arm.get_T_b_w() * Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
            self.arm.target_IK.set_pos(T_ik_w.p[0], T_ik_w.p[1], T_ik_w.p[2])
            self.arm.target_IK.set_rpy(T_ik_w.M.GetRPY()[0], T_ik_w.M.GetRPY()[1], T_ik_w.M.GetRPY()[2])
        if self.arm.target_FK is not None:
            ik_solution = self.arm.get_ik_solution()
            ik_solution = np.append(ik_solution, 0)
            T_t_b = convert_mat_to_frame(compute_FK(ik_solution))
            T_t_w = self.arm.get_T_b_w() * T_t_b
            self.arm.target_FK.set_pos(T_t_w.p[0], T_t_w.p[1], T_t_w.p[2])
            self.arm.target_FK.set_rpy(T_t_w.M.GetRPY()[0], T_t_w.M.GetRPY()[1], T_t_w.M.GetRPY()[2])

    def run(self):
            self.update_arm_pose()
            self.update_visual_markers()

            # subscribe the communication loss topic
            self.subscribe_communicationLoss()
    
    def communication_loss_callback(self, data):
        self.communication_loss = data.data

    def subscribe_communicationLoss(self):
        rospy.Subscriber("communication_loss", Bool, self.communication_loss_callback)
        # rate = rospy.Rate(100)
        # rate.sleep()


class ECMController:
    def __init__(self, gui, ecm):
        self.counter = 0
        self._ecm = ecm
        self._cam_gui = gui

    def update_camera_pose(self):
        self._cam_gui.App.update()
        self._ecm.servo_jp(self._cam_gui.jnt_cmds)

    def run(self):
        self.update_camera_pose()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='ambf_client')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=False)
    parser.add_argument('--ecm', action='store', dest='run_ecm', help='Control ECM', default=True)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    parsed_args.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)
    parsed_args.run_ecm = get_boolean_from_opt(parsed_args.run_ecm)

    c = Client(parsed_args.client_name)
    c.connect()

    time.sleep(0.5)
    controllers = []

    if parsed_args.run_psm_one is True:
        arm_name = 'psm1'
        psm = PSM(c, arm_name)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM1
            # init_xyz = [0.1, -0.85, -0.15]
            init_xyz = [0, 0, -1.0]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 3.0, 10.0, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)
        arm_name = 'psm1_ghost'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            init_xyz = [0, 0.0, -1.0]
            init_rpy = [3.14, 0.01, 1.57079]
            controller = PSMController(gui, psm, ghost=True)
            controllers.append(controller)

        

    if parsed_args.run_psm_two is True:
        arm_name = 'psm2'
        psm = PSM(c, arm_name)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -1.0]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 3.0, 12, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)
        
        arm_name = 'psm2_ghost'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False)
        if psm.is_present():
            init_xyz = [0, 0.0, -1.0]
            init_rpy = [3.14, 0.01, 1.57079]
            controller = PSMController(gui, psm, ghost=True)
            controllers.append(controller)

    if parsed_args.run_psm_three is True:
        arm_name = 'psm3'
        psm = PSM(c, arm_name)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -1.0]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 3.0, 3.14, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)



    if parsed_args.run_ecm is True:
        arm_name = 'CameraFrame'
        ecm = ECM(c, arm_name)
        gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"])
        controller = ECMController(gui, ecm)
        controllers.append(controller)



    if len(controllers) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            time.sleep(0.005)