import sys, time, pdb
import numpy as np
from numpy.linalg import norm
sys.path.append('/home/m/ompl/py-bindings/')

import pybullet as pb

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og

from utils import *

def get_contact_points(pb_client, robot_id, obstacle_id):
    return pb_client.getContactPoints(bodyA=robot_id, bodyB=obstacle_id)

## Validity checker for path planning
## Return whether the given joint angles is collision free
class ValidityChecker(ob.StateValidityChecker):
    pb_client = []
    robot_id = -1
    number_of_joints = 7
    obstacle_ids = []
    

    def isValid(self, state):

        ## Check collision with obstacles (by evaulating contact points with each obstacle)
        for obstacle_id in self.obstacle_ids:
            contact_points = get_contact_points(self.pb_client, self.robot_id, obstacle_id)

            if (len(contact_points) > 0):
                return False
        
        return True

## Returns a structure representing the optimization objective to use
#  for optimal motion planning. This method returns an objective
#  which attempts to minimize the length in configuration space of
#  computed paths.
def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

## Returns an optimization objective which attempts to minimize path
#  length that is satisfied when a path of length shorter than 1.51
#  is found.
def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj

## Defines an optimization objective which attempts to steer the
#  robot away from obstacles. To formulate this objective as a
#  minimization of path cost, we can define the cost of a path as a
#  summation of the costs of each of the states along the path, where
#  each state cost is a function of that state's clearance from
#  obstacles.
#
#  The class StateCostIntegralObjective represents objectives as
#  summations of state costs, just like we require. All we need to do
#  then is inherit from that base class and define our specific state
#  cost function by overriding the stateCost() method.
#
class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / (self.si_.getStateValidityChecker().clearance(s) +
                            sys.float_info.min))

## Return an optimization objective which attempts to steer the robot
#  away from obstacles.
def getClearanceObjective(si):
    return ClearanceObjective(si)

## Create an optimization objective which attempts to optimize both
#  path length and clearance. We do this by defining our individual
#  objectives, then adding them to a MultiOptimizationObjective
#  object. This results in an optimization objective where path cost
#  is equivalent to adding up each of the individual objectives' path
#  costs.
#
#  When adding objectives, we can also optionally specify each
#  objective's weighting factor to signify how important it is in
#  optimal planning. If no weight is specified, the weight defaults to
#  1.0.
def getBalancedObjective1(si):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ClearanceObjective(si)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    opt.addObjective(clearObj, 1.0)

    return opt

## Create an optimization objective equivalent to the one returned by
#  getBalancedObjective1(), but use an alternate syntax.
#  THIS DOESN'T WORK YET. THE OPERATORS SOMEHOW AREN'T EXPORTED BY Py++.
# def getBalancedObjective2(si):
#     lengthObj = ob.PathLengthOptimizationObjective(si)
#     clearObj = ClearanceObjective(si)
#
#     return 5.0*lengthObj + clearObj


## Create an optimization objective for minimizing path length, and
#  specify a cost-to-go heuristic suitable for this optimal planning
#  problem.
def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")

def generate_planner(pb_client,
                     robot_id,
                     obstacle_ids,
                     runTime,
                     plannerType="prmstar",
                     objectiveType="pathclearance"):
    
    ## State space for KUKA robot
    space = ob.CompoundStateSpace()
    for _ in range(7):
        space.addSubspace(component=ob.SO2StateSpace(), weight=1)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    ## Set the object used to check which states in the space are valid
    ValidityChecker.pb_client = pb_client
    ValidityChecker.robot_id = robot_id
    ValidityChecker.obstacle_ids = obstacle_ids
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)
    si.setup()

    ## Create a problem instance
    pdef = ob.ProblemDefinition(si)

    ## Create the optimization objective specified by our command-line argument.
    ## This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    ## Construct the optimal planner specified by our command line argument.
    ## This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)

    ## Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    ## Construct Roadmap
    optimizingPlanner.growRoadmap(runTime)
    optimizingPlanner.expandRoadmap(runTime)

    ## Store roadmap as a file
    planner_data = ob.PlannerData(si)
    optimizingPlanner.getPlannerData(planner_data)

    planner_data_storage = ob.PlannerDataStorage()
    # I am using the path clearance as the objective function for the planner
    planner_data_storage.store(planner_data, './planner_data_clearance')

    print('Completed PRM Generation')
    
    return

def load_planner(pb_client, robot_id, obstacle_ids, plannerType="prmstar", objectiveType="planner_data_clearance"):
    
    ## State space for Kuka robot
    space = ob.CompoundStateSpace()
    for _ in range(7):
        space.addSubspace(component=ob.SO2StateSpace(), weight=1)

    ## Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    ## Set the object used to check which states in the space are valid
    ValidityChecker.pb_client = pb_client
    ValidityChecker.robot_id = robot_id
    ValidityChecker.obstacle_ids = obstacle_ids
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)
    si.setup()

    ## Create a problem instance
    pdef = ob.ProblemDefinition(si)

    ## Create the optimization objective specified by our command-line argument.
    ## This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    planner_data = ob.PlannerData(si)
    planner_data_storage = ob.PlannerDataStorage()
    # I am using the path clearance as the objective function for the planner
    planner_data_storage.load('./planner_data_clearance', planner_data)
    optimizingPlanner = og.PRM(planner_data, starStrategy=True)
    optimizingPlanner.setProblemDefinition(pdef)

    return space, si, optimizingPlanner
    
def plan(planner_package, pb_client, robot_id, initial_state, final_state):
    space, si, optimizingPlanner = planner_package
    
    ## Define initial state
    initial_joint_angles = ob.State(space)
    for i in range(7):
        initial_joint_angles[i] = initial_state[i]

    ## Define final state
    final_joint_angles = ob.State(space)
    for i in range(7):
        final_joint_angles[i] = final_state[i]

    ## Create a problem definition instance
    pdef = ob.ProblemDefinition(si)
    ## Set the start and goal states
    pdef.setStartAndGoalStates(initial_joint_angles, final_joint_angles)
    ## Create the optimization objective
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
    ## optimizing planner
    optimizingPlanner.setProblemDefinition(pdef)
    solved = optimizingPlanner.solve(1)

    if solved:
        print("found solution")

        path = pdef.getSolutionPath()
        path_simplifier = og.PathSimplifier(si)
        simplified_solution = path_simplifier.simplifyMax(path)
        pdef.getSolutionPath().interpolate(100)

        trajectory = []
        for state in pdef.getSolutionPath().getStates():
            trajectory.append([state[i].value for i in range(7)])

    else:
        print("No solution found.")

        return []
    
    return trajectory
