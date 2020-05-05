
from Mujoco_Paper_class_sergi_version import MUJOCO
import numpy as np
import pandas as pd
import sys
sys.path.insert(1,"../Simulation_Pybullet/")
from Rotations import *



def PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title,numberJoints):
    dataframe_list = []
    angles_list = []

    #Go through all the database classes created during the experiment
    for data in robot.database_list:

        #pass the data to a list to do the average
        angles_list.append(data.joints_angles_rad)

        print(data.name)
        print("\n")
        """
        aux_df = pd.DataFrame({ "joint_angles_rad": data.joints_angles_rad,\
                                "joint_angles_vel_rad": data.joint_angles_vel_rad,\
                                "joint_torques": data.joint_torques,\
                                "tcp_position": data.tcp_position,\
                                "tcp_orientation_q": data.tcp_orientation_q,\
                                "tcp_orientation_e": data.tcp_orientation_e,\
                                "time": data.time})
        """

        #create the data frame
        aux_df = pd.DataFrame({})
        #convert data to numpy
        joint_angles = np.array(data.joints_angles_rad)
        for i in range(numberJoints):
            column_name = "joint"+str(i)

            aux_df[column_name] = joint_angles [:,i]
        aux_df.index = data.time
        dataframe_list.append(aux_df)

    #create and excel with all the experiments
    # Create a Pandas Excel writer using XlsxWriter as the engine.
    print ("I arrive here")
    writer = pd.ExcelWriter(folder+"/"+title+"_all_experiments.xlsx", engine='xlsxwriter')
    print ("I create excel")
    for count in range ( len(dataframe_list) ):
        #dataframe_list[count].head
        dataframe_list[count].to_excel(writer, sheet_name='Sheet'+str(count))

    # Close the Pandas Excel writer and output the Excel file.
    writer.save()

    #compute the average, convert all the data to numpy to make it easier
    joint_angles_array = np.array(angles_list)
    print(joint_angles_array.shape)
    #dimensions of the numpy
    [experiments,steps,joints] = joint_angles_array.shape

    average_steps = []
    for stp in range(steps):
        average_joints = []
        for j in range(joints):
            average_value = np.average(joint_angles_array[:,stp,j])
            average_joints.append(average_value)
        average_steps.append(average_joints)

    #create the average data frame
    avg_df = pd.DataFrame({})

    joint_angles_average = np.array(average_steps)
    for i in range(numberJoints):
        column_name = "joint"+str(i)

        avg_df[column_name] = joint_angles_average [:,i]
    avg_df.index = data.time
    avg_df.to_excel(folder+"/"+title+"_average.xlsx", sheet_name='Sheet1')

#experiment = "Single"
#experiment = "Double"
#experiment = "Sergi"
#experiment = "Cube"
experiment = "Training1"

repeats = 20
folder = "Experiments"
title = "Original_Mujoco_"+str(experiment)
if (__name__ == "__main__"):

    robot = MUJOCO()
    #to don't render and go faster
    robot.visual_inspection =True
    if (experiment =="Single"):
        times=7
    else:
        times = 1
    for i in range(times):


        joint = i
        if (experiment =="Single"):
            robot.database_list =[]
            robot._pid = [PID(),PID(),PID(),PID(),PID(),PID(),PID()]
            title = "Single"+"_"+str(joint)

        for iteration in range(repeats):
            robot.save_database = False
            robot.step_simulation()
            robot._sim.reset()
            robot.step_simulation()
            robot.save_database = True

            robot.set_experiment(experiment)
            robot.database_name = "Original_"+str(iteration)
            robot.run_mujoco(joint)
            folder = "."

        robot.database_name = "Dummy"
        robot.run_mujoco()


        numberJoints = 7
        PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title,numberJoints)

        print("finished")
