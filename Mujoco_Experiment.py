
from Mujoco_Paper_class_sergi_version import MUJOCO
import numpy as np
import pandas as pd
import sys
sys.path.insert(1,"../Simulation_Pybullet/")
from Rotations import *
#experiment = "Single"
#experiment = "Double"
experiment = "Sergi"
#experiment = "Cube"

repeats = 20
folder = "Experiments"
title = "Inertia_100"

robot = MUJOCO()

for iteration in range(repeats):
    robot.save_database = False
    robot.step_simulation()
    robot._sim.reset()
    robot.step_simulation()
    robot.save_database = False
    robot.set_experiment(experiment)
    robot.database_name = "Original_"+str(iteration)
    robot.run_mujoco()
