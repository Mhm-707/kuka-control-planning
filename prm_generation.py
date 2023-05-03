import sys, pdb, argparse, time, pickle, os
import numpy as np
from numpy.linalg import norm
sys.path.append('/home/m/ompl/py-bindings/')


import pybullet as pb
import pybullet_utils.bullet_client as bc

from utils import *
from prm_planning import *

if (__name__ == "__main__"):
    
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Motion planning for pick and place using PRM planning.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=1.0, help=\
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')

    args = parser.parse_args()

    ## Initialize the simulator
    pb_client = bc.BulletClient(connection_mode=pb.DIRECT)
    pb_client.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    pb_client.setGravity(0,0,-10)

    ## Load urdf models
    plane_id = pb_client.loadURDF("plane.urdf")
    robot_id = pb_client.loadURDF('./robot_models/kuka_vacuum.urdf',
                                  [0, 0, .5], useFixedBase=True)
    conveyor_id = pb_client.loadURDF('./robot_models/conveyor.urdf', [.5, 0, 0], useFixedBase=True)
    tray_red = pb_client.loadURDF("./robot_models/traybox_covered_red.urdf", tray_red_location, globalScaling=1.5)
    tray_green = pb_client.loadURDF("./robot_models/traybox_covered_green.urdf", tray_green_location, globalScaling=1.5)
    tray_blue = pb_client.loadURDF("./robot_models/traybox_covered_blue.urdf", tray_blue_location, globalScaling=1.5)
    table_id = pb_client.loadURDF(os.path.join(pybullet_data.getDataPath(),"table/table.urdf"), [0,.65,-.85], globalScaling=1.8)
    obstacle_ids = [plane_id, conveyor_id, tray_red, tray_green, tray_blue, table_id]

    generate_planner(pb_client=pb_client,
                     robot_id=robot_id,
                     obstacle_ids=obstacle_ids,
                     runTime=args.runtime,
                     plannerType="prmstar")
    
    for _ in range(1000000):
        time.sleep(1/240)
        pb_client.stepSimulation()
    
    
