
from Mujoco_Paper_class_sergi_version_Rishabh import MUJOCO
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
        aux_df.index = np.arange(0,0.0001*joint_angles.shape[0],0.0001)
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
    avg_df.index = np.arange(0,0.0001*joint_angles_average.shape[0],0.0001)
    avg_df.to_excel(folder+"/"+title+"_average.xlsx", sheet_name='Sheet1')



if (__name__ == "__main__"):

    robot = MUJOCO()
    robot.save_database = False
    Exp_Traj = np.load("acs.npy")
    Num = 19
    Exp_actions = Exp_Traj[Num,:,:]
    #to don't render and go faster

    #already set in the _env_setup
    #initial_qpos = [1.12810781, -0.59798289, -0.53003607]

    robot._env_setup()
    robot.visual_inspection =True
    robot.save_database = True
    for i in range(Exp_actions.shape[0]):
        print(i)
        actions = Exp_actions[i,:]
        robot._set_action(actions)
        for j in range(10**3):
            robot.step_simulation()

    robot.database_name = "Dummy"
    robot.step_simulation()

    print(robot.database_list)
    PassRobotDatabaseClass_2_excel_jointpos(robot,".","test",7)

    print("finished")
