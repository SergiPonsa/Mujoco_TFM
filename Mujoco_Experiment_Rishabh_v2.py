
from Mujoco_Paper_class_sergi_version_Rishabh_v2 import MUJOCO
import numpy as np
import pandas as pd
import sys
sys.path.insert(1,"../Simulation_Pybullet/")
from Rotations import *



def PassRobotDatabaseClass_2_excel_jointpos(robot,folder,title,numberJoints,n_substeps=20):
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
        aux_df.index = np.arange(0,0.0001*joint_angles.shape[0]*n_substeps,0.0001*n_substeps)
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
    avg_df.index = np.arange(0,0.0001*joint_angles_average.shape[0]*n_substeps,0.0001*n_substeps)
    avg_df.to_excel(folder+"/"+title+"_average.xlsx", sheet_name='Sheet1')

def PassRobotDatabaseClass_2_excel_tcppos(robot,folder,title,n_substeps=20):
    dataframe_list = []
    tcp_list = []

    #Go through all the database classes created during the experiment
    for data in robot.database_list:

        #pass the data to a list to do the average
        tcp_list.append(data.tcp_position)

        print(data.name)
        print("\n")

        #create the data frame
        aux_df = pd.DataFrame({})
        #convert data to numpy
        tcp_poses = np.array(data.tcp_position)
        print(tcp_poses)
        print(tcp_poses.shape)
        for i in range(3):
            column_name = "pos"+str(i)

            aux_df[column_name] = tcp_poses [:,i]
        aux_df.index = np.arange(0,0.0001*tcp_poses.shape[0]*n_substeps,0.0001*n_substeps)
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
    tcp_poses_array = np.array(tcp_list)
    print(tcp_poses_array.shape)
    #dimensions of the numpy
    [experiments,steps,joints] = tcp_poses_array.shape

    average_steps = []
    for stp in range(steps):
        average_tcp = []
        for j in range(joints):
            average_value = np.average(tcp_poses_array[:,stp,j])
            average_tcp.append(average_value)
        average_steps.append(average_tcp)

    #create the average data frame
    avg_df = pd.DataFrame({})

    tcp_poses_average = np.array(average_steps)
    for i in range(7):
        column_name = "pos"+str(i)

        avg_df[column_name] = tcp_poses_average [:,i]
    #avg_df["rx"] = [1.5707963267948966]*tcp_poses_array.shape[0]
    #avg_df["ry"] = [-0.0]*tcp_poses_array.shape[0]
    #avg_df["rz"] = [3.141592653589793]*tcp_poses_array.shape[0]

    avg_df.index = np.arange(0,0.0001*tcp_poses_average.shape[0]*n_substeps,0.0001*n_substeps)
    avg_df.to_excel(folder+"/"+title+"_average.xlsx", sheet_name='Sheet1')



if (__name__ == "__main__"):

    robot = MUJOCO()
    robot.save_database = False
    Exp_Traj = np.load("acs.npy")
    Num = 99
    Exp_actions = Exp_Traj[Num,:,:]
    #to don't render and go faster

    #already set in the _env_setup
    initial_qpos = {
            'robot1:Actuator1': 0.0,
            'robot1:Actuator2': 0.0,
            'robot1:Actuator3': 0.0,
            'robot1:Actuator4': 0.0,
            'robot1:Actuator5': 0.0,
            'robot1:Actuator6': 0.0,
            'robot1:Actuator7': 0.0,
        }

    robot._env_setup(initial_qpos)
    robot.visual_inspection =True
    robot.save_database = True
    for i in range(Exp_actions.shape[0]):
        print(i)
        actions = Exp_actions[i,:]
        robot._set_action(actions)
        #for j in range(20*10):
            #robot.step_simulation()

    robot.database_name = "Dummy"
    robot.step_simulation()

    print(robot.database_list)
    PassRobotDatabaseClass_2_excel_jointpos(robot,".","test_"+str(Num),7)
    PassRobotDatabaseClass_2_excel_tcppos(robot,".","test_tcp_"+str(Num))

    print("finished")
