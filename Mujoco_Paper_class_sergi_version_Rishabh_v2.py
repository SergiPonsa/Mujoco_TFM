"""
Created on Mon May 28 10:37:34 2018

@author: jack

"""

from mujoco_py import *

import time
import csv
import math
import sys
import numpy as np
#import pybullet as p
sys.path.insert(1,"../Simulation_Pybullet/")
from PID import *
from Rotations import *
from RobotDataBaseClass import RobotDataBase
sys.path.insert(1,".")

class MUJOCO(object):
    def __init__(self,save_database = True ,visual_inspection = True, n_substeps=20,gripper_extra_height=0.2):
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
        self._model = load_model_from_path("models_rishabh/randomizedgen3.xml")
        self._sim = MjSim(self._model,nsubsteps=n_substeps)
        self._viewer = MjViewer(self._sim)
        self._sim.model.opt.timestep = self._timestep

        self.database_name = "Original"
        self.database_name_old = None
        self.database_list = []

        self.save_database = save_database
        self.visual_inspection = visual_inspection

        #Data for Rishabh
        self.gripper_extra_height = gripper_extra_height

    def set_experiment(self,experiment):
        self._experiment = experiment

    def set_num_steps(self):
        self._num_steps = simSteps(self._experiment,self._timestep)

    def step_simulation(self):
        """Step simulation method"""
        self._sim.step()
        if self.visual_inspection:
            #print("I render")
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

        #self.database.joints_angles_rad.append( self._theta.copy())
        self.database.joints_angles_rad.append( [self._sim.data.sensordata[0],\
                                                self._sim.data.sensordata[1],\
                                                self._sim.data.sensordata[2],\
                                                self._sim.data.sensordata[3],\
                                                self._sim.data.sensordata[4],\
                                                self._sim.data.sensordata[5],\
                                                self._sim.data.sensordata[6] ] )
        self.database.joint_angles_vel_rad.append( self._sim.data.qvel)
        self.database.joint_torques.append( self._sim.data.qacc)

        #print(self._sim.sensordata[7])
        #print(self._sim.sensordata[8])
        self.database.tcp_position.append([self._sim.data.sensordata[7],self._sim.data.sensordata[8],self._sim.data.sensordata[9],\
                                            self._sim.data.sensordata[10],self._sim.data.sensordata[11],self._sim.data.sensordata[12],self._sim.data.sensordata[13]\
                                            ])
        #self.database.tcp_orientation_q.append(self._sim.sensordata[8])
        #self.database.tcp_orientation_e.append(p.getEulerFromQuaternion(self._sim.sensordata[8]))
        #self.database.save_time()

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
    #def _env_setup(self, initial_qpos):
    def _env_setup(self,initial_qpos):
        for name, value in initial_qpos.items():
            self._sim.data.set_joint_qpos(name, value)
        self.reset_mocap_welds()
        self._sim.forward()

        # Move end effector into position.
        gripper_target = np.array([1.12810781, -0.59798289, -0.53003607 + self.gripper_extra_height]) #+ self._sim.data.get_site_xpos('robotiq_85_base_link')
        gripper_rotation = np.array([1., 0., 1., 0.])
        #self._sim.data.set_mocap_pos('robot1:mocap', gripper_target)
        #self._sim.data.set_mocap_quat('robot1:mocap', gripper_rotation)
        self._sim.data.set_mocap_pos('robot1:mocap', gripper_target)
        self._sim.data.set_mocap_quat('robot1:mocap', gripper_rotation)
        for k in range(10):
            print(k)
            self.step_simulation()
            #self._sim.step()
            time.sleep(1.0)
            print(self._sim.data.qpos)

        # Extract information for sampling goals.
        #self.initial_gripper_xpos = self.sim.data.get_site_xpos('robot1:grip').copy()
        #if self.has_object:
            #self.height_offset = self.sim.data.get_site_xpos('object0')[2]
    def reset_mocap_welds(self):
        """
        Resets the mocap welds that we use for actuation.
        """
        print("Hi")
        if self._sim.model.nmocap > 0 and self._sim.model.eq_data is not None:
            print("Some actuator it's created")
            for i in range(self._sim.model.eq_data.shape[0]):
                if self._sim.model.eq_type[i] == const.EQ_WELD:
                    self._sim.model.eq_data[i, :] = np.array([0., 0., 0., 1., 0., 0., 0.])
        self._sim.forward()

    def _set_action(self, action):
        assert action.shape == (4,)
        action = action.copy()  # ensure that we don't change the action outside of this scope
        pos_ctrl, gripper_ctrl = action[:3], action[3]

        pos_ctrl *= 0.05  # limit maximum change in position
        pos_ctrl *= 0.05  # limit maximum change in position
        rot_ctrl = [0., 1., 1., 0.]  # fixed rotation of the end effector, expressed as a quaternion {Vertical}
        #rot_ctrl = [1., 0., 1., 0.]  # fixed rotation of the end effector, expressed as a quaternion {Horizontal}
        gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
        assert gripper_ctrl.shape == (2,)
        #if self.block_gripper:
            #print("Block the gripper position")
            #gripper_ctrl = np.zeros_like(gripper_ctrl)
        action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])

        # Apply action to simulation.
        self.ctrl_set_action(action)
        self.mocap_set_action(action)

        self.step_simulation()

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

    def reset_mocap2body_xpos(self):
        """Resets the position and orientation of the mocap bodies to the same
        values as the bodies they're welded to.
        """

        if (self._sim.model.eq_type is None or
            self._sim.model.eq_obj1id is None or
            self._sim.model.eq_obj2id is None):
            print("there are no objects")
            return
        for eq_type, obj1_id, obj2_id in zip(self._sim.model.eq_type,
                                             self._sim.model.eq_obj1id,
                                             self._sim.model.eq_obj2id):
            print("there are objects")
            if eq_type != const.EQ_WELD:
                continue

            mocap_id = self._sim.model.body_mocapid[obj1_id]
            if mocap_id != -1:
                # obj1 is the mocap, obj2 is the welded body
                body_idx = obj2_id
            else:
                # obj2 is the mocap, obj1 is the welded body
                mocap_id = self._sim.model.body_mocapid[obj2_id]
                body_idx = obj1_id

            # assert (mocap_id != -1)
            self._sim.data.mocap_pos[mocap_id][:] = self._sim.data.body_xpos[body_idx]
            self._sim.data.mocap_quat[mocap_id][:] = self._sim.data.body_xquat[body_idx]

    def mocap_set_action(self, action):
        """The action controls the robot using mocaps. Specifically, bodies
        on the robot (for example the gripper wrist) is controlled with
        mocap bodies. In this case the action is the desired difference
        in position and orientation (quaternion), in world coordinates,
        of the of the target body. The mocap is positioned relative to
        the target body according to the delta, and the MuJoCo equality
        constraint optimizer tries to center the welded body on the mocap.
        """
        if self._sim.model.nmocap > 0:
            action, _ = np.split(action, (self._sim.model.nmocap * 7, ))
            action = action.reshape(self._sim.model.nmocap, 7)
            # print(action)

            pos_delta = action[:, :3]
            quat_delta = action[:, 3:]

            self.reset_mocap2body_xpos()
            print("Hi")
            self._sim.data.mocap_pos[:] = self._sim.data.mocap_pos + pos_delta
            self._sim.data.mocap_quat[:] = self._sim.data.mocap_quat + quat_delta
            print("bye")

    """
    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self._set_action(action)
        self.sim.step()
        self._step_callback()
        obs = self._get_obs()

        done = False
        info = {
            'is_success': self._is_success(obs['achieved_goal'], self.goal),
        }
        reward = self.compute_reward(obs['achieved_goal'], self.goal, info)
        #print("REWARD", reward ," and IS SUCCESS ", info['is_success'])
        return obs, reward, done, info
    """
