"""
Created on Mon May 28 10:37:34 2018

@author: jack

"""

from mujoco_py import *

import time
import csv
import math
import sys
#import pybullet as p
sys.path.insert(1,"../Simulation_Pybullet/")
from PID import *
from Rotations import *
from RobotDataBaseClass import RobotDataBase
sys.path.insert(1,".")

class MUJOCO(object):
    def __init__(self,save_database = True ,visual_inspection = True):
        self._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID()]
        self._linearVelocity = [0,0,0,0,0,0,0,0,0]
        self._theta = [0,0,0,0,0,0,0,0,0]
        self._positions =[]
        self._convertdeg2rad = 57.295779578552
        self._timestep = 0.0001

        self._simulator = "Mujoco"
        self._physics_engine = ""
        self._experiment = ""
        self._current_iteration = 0


        #self._model = load_model_from_path("models-sergi/urdf/JACO3_URDF_V11_Mujoco.xml")
        self._model = load_model_from_path("models-sergi/urdf/Gen3Robotiq.xml")
        self._sim = MjSim(self._model)
        self._viewer = MjViewer(self._sim)
        self._sim.model.opt.timestep = self._timestep

        self.database_name = "Original"
        self.database_name_old = None
        self.database_list = []

        self.save_database = save_database
        self.visual_inspection = visual_inspection

    def set_experiment(self,experiment):
        self._experiment = experiment

    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep)

    def step_simulation(self):
        """Step simulation method"""
        self._sim.step()
        if self.visual_inspection:
            self._viewer.render()
            time.sleep(self._timestep)

        if self.save_database:
            self.record_database()

    def record_database(self):
        if(self.database_name != self.database_name_old):

            if(self.database_name_old != None):
                auxdatabase = self.database
                self.database_list.append(auxdatabase)

            self.database_name_old = self.database_name
            self.database = RobotDataBase(self.database_name,time_step = self._timestep)

        self.database.joints_angles_rad.append( self._theta.copy())
        """
        self.database.joints_angles_rad.append( [self._sim.data.sensordata[0],\
                                                self._sim.data.sensordata[1],\
                                                self._sim.data.sensordata[2],\
                                                self._sim.data.sensordata[3],\
                                                self._sim.data.sensordata[4],\
                                                self._sim.data.sensordata[5],\
                                                self._sim.data.sensordata[6])
        """
        self.database.joint_angles_vel_rad.append( self._sim.data.qvel)
        self.database.joint_torques.append( self._sim.data.qacc)

        #[tcp_position, tcp_orientation_q] = self.get_actual_tcp_pose()
        """
        print(self._sim.sensordata[7])
        print(self._sim.sensordata[8])
        self.database.tcp_position.append(self._sim.sensordata[7])
        self.database.tcp_orientation_q.append(self._sim.sensordata[8])
        self.database.tcp_orientation_e.append(p.getEulerFromQuaternion(self._sim.sensordata[8]))
        """
        self.database.save_time()

    def run_mujoco(self,joint = 1):
        self.set_num_steps()
        for simStep in range(self._num_steps):

            self._pid = set_target_thetas(self._num_steps, self._pid,self._experiment,self._simulator,simStep,joint)
            self.step_simulation()
            if simStep % 500 == 0:
                for jointNum in range(7):
                    self._theta[jointNum] = self._sim.data.sensordata[jointNum]
                    print(self._theta)
                    self._linearVelocity[jointNum] = self._pid[jointNum].get_velocity(math.degrees(self._theta[jointNum]))/self._convertdeg2rad
                    #print(self._linearVelocity)
                    self._sim.data.ctrl[jointNum] = self._linearVelocity[jointNum]


    #To run Rishabh code

    def ctrl_set_action(self, action):
        """For torque actuators it copies the action into mujoco ctrl field.
        For position actuators it sets the target relative to the current qpos.
        """
        if self._sim.model.nmocap > 0:
            _, action = np.split(action, (self._sim.model.nmocap * 7, ))
        # print(action)
        if self._sim.data.ctrl is not None:
            for i in range(action.shape[0]):
                if self._sim.model.actuator_biastype[i] == 0:#If tipe it's torque
                    self._sim.data.ctrl[i] = action[i]
                else:
                    print("Joint position control")
                    idx = self._sim.model.jnt_qposadr[self._sim.model.actuator_trnid[i, 0]]
                    self._sim.data.ctrl[i] = self._sim.data.qpos[idx] + action[i]

    def _set_action(self, action):
        assert action.shape == (4,)
        action = action.copy()  # ensure that we don't change the action outside of this scope
        pos_ctrl, gripper_ctrl = action[:3], action[3]

        pos_ctrl *= 0.05  # limit maximum change in position
        rot_ctrl = [1., 0., 1., 0.]  # fixed rotation of the end effector, expressed as a quaternion
        gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
        assert gripper_ctrl.shape == (2,)
        if self.block_gripper:
            print("Block the gripper position")
            gripper_ctrl = np.zeros_like(gripper_ctrl)
        action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])

        # Apply action to simulation.
        self.ctrl_set_action(action)
        #utils.mocap_set_action(self.sim, action)
