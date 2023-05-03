import os, time
import numpy as np

import pybullet as pb
import pybullet_data
import pybullet_utils.bullet_client as bc
from prm_planning import *

from utils import *
from utils2 import *

arm_joint_indices = [0, 1, 2, 3, 4, 5, 6]
robot_base_location = [.5, 0, 0]
tray_red_location = [-.9,.7,.3]
tray_green_location = [0,.7,.3]
tray_blue_location = [.9,.7,.3]
tray_width = 1.5*.6
K_1, K_2 = compute_control_gain()
# Edited tray locations to make sure that the cubes are placed on the tray properly.
tray_red_location_edited = [x + y for x, y in zip(tray_red_location, [-0.05, -0.2, 0.95])]
tray_green_location_edited = [x + y for x, y in zip(tray_green_location, [0.1, -0.2, 0.95])] 
tray_blue_location_edited = [x + y for x, y in zip(tray_blue_location, [0.1, 0.1, 0.95])]
# Defining a list of target positions to calculate the inverse kinematics earlier.
target_positions = [[tray_red_location_edited], [tray_green_location_edited], [tray_blue_location_edited]]


if __name__ == '__main__':
    ## Initialize the simulator
    pb_client = initializeGUI(enable_gui=False)
    
    # Initilize the planning client
    pb_client_planning = bc.BulletClient(connection_mode=pb.DIRECT)
    pb_client_planning.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb_client_planning.setGravity(0,0,-10)

    ## Load urdf models
    plane_id = pb_client.loadURDF("plane.urdf")
    robot_id = pb_client.loadURDF('./robot_models/kuka_vacuum.urdf',
                                  [0, 0, .5], useFixedBase=True)
    
    free_joint_torques(pb_client, robot_id, arm_joint_indices)

    
    robot_id_planning = pb_client_planning.loadURDF('./robot_models/kuka_vacuum.urdf', robot_base_location, useFixedBase=True)
    plane_id_planning = pb_client_planning.loadURDF("plane.urdf")

    conveyor_id = pb_client.loadURDF('./robot_models/conveyor.urdf', robot_base_location, useFixedBase=True)
    conveyor_id_planning = pb_client_planning.loadURDF('./robot_models/conveyor.urdf', robot_base_location, useFixedBase=True)

    tray_red = pb_client.loadURDF("./robot_models/traybox_covered_red.urdf", tray_red_location, globalScaling=1.5)
    tray_green = pb_client.loadURDF("./robot_models/traybox_covered_green.urdf", tray_green_location, globalScaling=1.5)
    tray_blue = pb_client.loadURDF("./robot_models/traybox_covered_blue.urdf", tray_blue_location, globalScaling=1.5)
    
    tray_red_planning = pb_client_planning.loadURDF("./robot_models/traybox_covered_red.urdf", tray_red_location, globalScaling=1.5)
    tray_green_planning = pb_client_planning.loadURDF("./robot_models/traybox_covered_green.urdf", tray_green_location, globalScaling=1.5)
    tray_blue_planning = pb_client_planning.loadURDF("./robot_models/traybox_covered_blue.urdf", tray_blue_location, globalScaling=1.5)
    
    table_id = pb_client.loadURDF(os.path.join(pybullet_data.getDataPath(),"table/table.urdf"), [0,.65,-.85], globalScaling=1.8)
    table_id_planning = pb_client_planning.loadURDF(os.path.join(pybullet_data.getDataPath(),"table/table.urdf"), [0,.65,-.85], globalScaling=1.8)
    
    obstacle_ids = [plane_id_planning, conveyor_id_planning, tray_red_planning, tray_green_planning, tray_blue_planning, table_id_planning]
    
    planner_package = load_planner(pb_client=pb_client_planning,
                                   robot_id=robot_id,
                                   obstacle_ids=obstacle_ids)
    
    ## Draw a frame at the end-effector
    draw_frame(pb_client, robot_id, link_index=7, axis_length=.1, line_width=3)

    ## Debug texts
    position, orientation = get_end_effector_pose(pb_client,
                                                  robot_id)

    ## Main loop
    np.random.seed()
    roller_speed = 10 + np.random.random()
    
    # for target_position in target_positions:
    #     mark_point(pb_client, target_position[0], axis_length=.1, line_width=3)

    grasped_cube_info = -1
    grasped_flag = -1 # Flag used to determine if there's a grasped cube
    tray_angles = []
    for position in [tray_red_location_edited, tray_green_location_edited, tray_blue_location_edited]:
        tray_angles.append(inverse_kinematics(pb_client, robot_id, position, [np.pi, 0, 0]))
        
    
    for time_step in range(100000):

        ## Place new cube on the conveyor
        if ((time_step/240)%2 == 0):
            color_code = np.random.randint(0,3)
            dy_random = .4*(np.random.random(1)-.5) # Randomize the shift of y position of the cube between -.2 and .2

            place_object(pb_client, [1.5, -.5+dy_random, .8], color_code) # Place a new cube on the conveyor at y location between -.7 and -0.3
            
        ## Rotate roller at a constant speed
        for i in range(1,22):
            pb_client.setJointMotorControl2(bodyUniqueId=conveyor_id,
                                            jointIndex=i,
                                            targetVelocity=roller_speed,
                                            controlMode=pb.VELOCITY_CONTROL)

        ## Joint control
        cube_locations = get_cube_locations(pb_client)
        end_effector_pose = get_end_effector_pose(pb_client, robot_id)
        joint_angles = get_joint_angles(pb_client, robot_id)
        joint_velocities = get_joint_velocities(pb_client, robot_id)
        
        if grasped_flag == -1:
        
            if cube_locations != [] and cube_locations[0][0] < 0.7:
                go_to_location(pb_client, time_step, robot_id, joint_angles, joint_velocities, K_1, K_2, [x + y for x, y in zip(cube_locations[0], [-0.03, 0, 0.11])], end_effector_target_orientation)
            else:
                go_to_initial_position(pb_client, time_step, robot_id, joint_angles, joint_velocities, K_1, K_2)
            
            grasped_cube_info, grasped_flag = grasp_object(pb_client, robot_id)
            pb_client.stepSimulation()
            time.sleep(1/240) 
        else:
            grasped_cube_id = grasped_cube_info[0]
            grasped_cube_color = grasped_cube_info[1]
            
            time_sec = time_step / 240
            if grasped_cube_color == 0:
                grasped_flag = go_to_red_plan(pb_client, pb_client_planning, planner_package, time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2)
                if grasped_flag == -1:
                    release_fix(pb_client, robot_id, arm_joint_indices)
            elif grasped_cube_color == 1:
                grasped_flag = go_to_green_plan(pb_client, pb_client_planning, planner_package, time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2)
                if grasped_flag == -1:
                    release_fix(pb_client, robot_id, arm_joint_indices)
            else:
                grasped_flag = go_to_blue_plan(pb_client, pb_client_planning, planner_package, time_step, robot_id, robot_id_planning, joint_angles, joint_velocities, K_1, K_2)
                if grasped_flag == -1:
                    release_fix(pb_client, robot_id, arm_joint_indices)
                
                    
    pb.disconnect()
