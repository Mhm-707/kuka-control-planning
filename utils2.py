import pybullet as pb
import pybullet_data
import pybullet_utils.bullet_client as bc
import numpy as np
import time, sys
sys.path.append('/home/m/ompl/py-bindings/')

from kuka_dynamics import *
from kuka_kinematics import *
from prm_planning import *
from utils import *
import control


robot_id = -1
arm_joint_indices = [0, 1, 2, 3, 4, 5, 6]
end_effector_link_index = 7
constraint_id = -1
grasped_cube_id = -1
cube_indices = []
cube_colors = {}

tray_angles = [(-1.5645978492389427, -1.3851618568940318, 1.6489627020934818, 1.1722294995790654, 1.1838554672269421, 0.3346204738297698, -0.21318942799725152), (1.6977829677497298, 0.6880516244800711, -1.503711293770871, -0.46028024541041807, -2.112949651248695, -2.3136769471621874, 1.0536428742918122), (0.6997465787276819, 1.1413717388032398, -1.3414444422491136, -0.047574635252940776, -1.8202096287277014, -1.89140730254574, 0.6471320616120222), (0.4771489546429832, 0.6872408423105232, -1.4921132638427337, -0.6561548053225781, -2.2488766016090147, -2.1994619651733824, -0.36329706558605795)]
tray_angles = np.array(tray_angles)


def plan_controller(pb_client, time_step, robot_id, joint_angles, joint_velocities, K_1, K_2, joint_angles_desired):
    '''
    This function computes the joint torques required to track the desired joint angles.
    '''
    q_ref = np.array(joint_angles_desired)
          
    joint_actuator_torques = compute_joint_torques_waypoint_tracking(time_step / 240,
                                                                        K_1,
                                                                        K_2,
                                                                        joint_angles,
                                                                        joint_velocities,
                                                                        q_ref)
     ## Apply the joint torques      
    for i, joint_index in enumerate(arm_joint_indices):
        joint_torque_control(pb_client, robot_id, joint_index, joint_actuator_torques[i])
        
    return None


def go_to_location_plan(pb_client, pb_client_planning, planner_package , time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2, desired_angles):
    final_state = desired_angles
    initial_state = get_joint_angles(pb_client, robot_id)
    solution = plan(planner_package, pb_client_planning, robot_id_planning, initial_state, final_state)
    #draw_trajectory(pb_client, robot_id, solution)
    i = 0
    while i < len(solution):
        ## Set joint angles
        joint_angles = solution[i]
        current_angles = get_joint_angles(pb_client, robot_id)
        joint_velocities = get_joint_velocities(pb_client, robot_id)
        plan_controller(pb_client, time_step, robot_id, current_angles, joint_velocities, K_1, K_2, joint_angles)
            
        time.sleep(1/240)
        pb_client.stepSimulation()
        # Check if the robot has reached the goal, if yes, move on to the next goal state in the trajectory
        if np.linalg.norm(np.array(joint_angles) - np.array(current_angles)) < 0.25:
            i = i + 1
            
            
    grasped_flag = -1
       
    return grasped_flag


def go_to_red_plan(pb_client, pb_client_planning, planner_package , time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2):

    grasped_flag = go_to_location_plan(pb_client, pb_client_planning, planner_package , time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2, tray_angles[0])
        
    return grasped_flag

def go_to_green_plan(pb_client, pb_client_planning, planner_package , time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2):
       
    grasped_flag = go_to_location_plan(pb_client, pb_client_planning, planner_package , time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2, tray_angles[1])
        
    return grasped_flag

def go_to_blue_plan(pb_client, pb_client_planning, planner_package , time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2):
       
    grasped_flag = go_to_location_plan(pb_client, pb_client_planning, planner_package , time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2, tray_angles[2])
        
    return grasped_flag

