import math

from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt
import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


def parse_obstacle(s):
    try:
        x, y, radius = map(float, s.split(','))
        return Obstacle(x, y, radius)
    except Exception:
        raise argparse.ArgumentTypeError("Obstacles must be x,y,radius")


class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, obstacles):
        super(ValidityChecker, self).__init__(si)
        self.obstacles = obstacles

    # Returns whether the given state's position overlaps the
    # circular obstacle
    def isValid(self, state):
        x = state.getX()
        y = state.getY()
        for obstacle in self.obstacles:
            if sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2)) - obstacle.radius <= 0:
                return False

        return True

    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.
    def clearance(self, state):
        if len(self.obstacles) == 0:
            return 1

        # Extract the robot's (x,y) position from its state
        x = state.getX()
        y = state.getY()

        clearance = 0
        # Distance formula between two points, offset by the circle's
        # radius
        for obstacle in self.obstacles:

            clearance += sqrt(pow(x - obstacle.x, 2) + pow(y - obstacle.y, 2)) - obstacle.radius
            if clearance <= 0:
                return 0

        return clearance


def absolute_distance_between_angles(angle1, angle2):
    fabs = math.fabs(math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2)))
    return fabs


class MyDecomposition(oc.GridDecomposition):
    def __init__(self, length, bounds):
        super(MyDecomposition, self).__init__(length, 2, bounds)

    def project(self, s, coord):
        coord[0] = s.getX()
        coord[1] = s.getY()

    def sampleFullState(self, sampler, coord, s):
        sampler.sampleUniform(s)
        s.setXY(coord[0], coord[1])


def get_state_propagator(wind_direction):
    def propagate(start, control, duration, state):
        # Get angle after applying control for duration
        new_angle = start.getYaw() + control[0] * duration * 0.6

        #Angle between the boat and the wind
        boat_wind_angle = absolute_distance_between_angles(reverseAngle(wind_direction), new_angle)

        # Hacky formulas to allow wind direction to influence boat speed. Should be replace with real ones later
        boat_speed = pow((boat_wind_angle * 10) / (math.pi / 2), 1.1) / 3
        rotation_cost = absolute_distance_between_angles(new_angle, state.getYaw()) / math.pi
        boat_speed -= rotation_cost * 0.1

        state.setX(start.getX() + boat_speed * duration * math.cos(start.getYaw()))
        state.setY(start.getY() + boat_speed * duration * math.sin(start.getYaw()))
        state.setYaw(new_angle)
        # print(state.getX(), state.getYaw(), state.getX())

    return propagate


def get_path_length_objective(si):
    return ob.PathLengthOptimizationObjective(si)


def get_threshold_path_length_objective(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj


class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        if (self.si_.getStateValidityChecker().clearance(s) == 0):
            return sys.maxsize
        return ob.Cost(1 / self.si_.getStateValidityChecker().clearance(s))


def get_clearance_objective(si):
    return ClearanceObjective(si)


def get_path_length_obj_with_cost_to_go(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocate_planner(si, planner_type, decomp):
    if planner_type.lower() == "est":
        return oc.EST(si)
    elif planner_type.lower() == "kpiece":
        return oc.KPIECE1(si)
    elif planner_type.lower() == "pdst":
        return og.PDST(si)
    elif planner_type.lower() == "rrt":
        return oc.RRT(si)
    elif planner_type.lower() == "syclopest":
        return oc.SyclopEST(si, decomp)
    elif planner_type.lower() == "sycloprrt":
        return oc.SyclopRRT(si, decomp)

    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocate_objective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return get_clearance_objective(si)
    elif objectiveType.lower() == "pathlength":
        return get_path_length_objective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return get_threshold_path_length_objective(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


def create_numpy_path(states):
    lines = states.splitlines()
    length = len(lines) - 1
    array = np.zeros((length, 2))

    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array


def plan(run_time, planner_type, objective_type, wind_direction, dimensions, obstacles):
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.

    space = ob.SE2StateSpace()

    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions[0])
    bounds.setLow(1, dimensions[1])

    bounds.setHigh(0, dimensions[2])
    bounds.setHigh(1, dimensions[3])

    # Set the bounds of space to be in [0,1].
    space.setBounds(bounds)
    # create a control space
    cspace = oc.RealVectorControlSpace(space, 1)

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(1)
    cbounds.setLow(-math.pi / 3)
    cbounds.setHigh(math.pi / 3)
    cspace.setBounds(cbounds)

    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    propagator = get_state_propagator(wind_direction)
    ss.setStatePropagator(oc.StatePropagatorFn(propagator))

    # Construct a space information instance for this state space
    si = ss.getSpaceInformation()

    si.setPropagationStepSize(1)

    # Set the object used to check which states in the space are valid
    validity_checker = ValidityChecker(si, obstacles)
    ss.setStateValidityChecker(validity_checker)

    # Set our robot's starting state to be the bottom-left corner of
    # the environment, or (0,0).
    start = ob.State(space)
    start[0] = dimensions[0]
    start[1] = dimensions[1]
    start[2] = math.pi / 4

    # Set our robot's goal state to be the top-right corner of the
    # environment, or (1,1).
    goal = ob.State(space)
    goal[0] = dimensions[2]
    goal[1] = dimensions[3]
    goal[2] = math.pi / 4

    # Set the start and goal states
    ss.setStartAndGoalStates(start, goal)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    objective = allocate_objective(si, objective_type)
    ss.setOptimizationObjective(objective)

    # Create a decomposition of the state space
    decomp = MyDecomposition(256, bounds)

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizing_planner = allocate_planner(si, planner_type, decomp)

    ss.setPlanner(optimizing_planner)

    # attempt to solve the planning problem in the given runtime
    solved = ss.solve(run_time)

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization '
              'objective value of {2:.4f}'.format(ss.getPlanner().getName(),
                                                  ss.getSolutionPath().length(),
                                                  0.1))
        print(ss.getSolutionPath().printAsMatrix())
        print(ss.haveExactSolutionPath())
        plot_path(ss.getSolutionPath(), dimensions, obstacles)

    else:
        print("No solution found.")


def reverseAngle(angle):
    if angle <= 0:
        return angle + math.pi
    else:
        return angle - math.pi


def plot_path(solution_path, dimensions, obstacles):
    matrix = solution_path.printAsMatrix()
    path = create_numpy_path(matrix)
    x, y = path.T
    ax = plt.gca()
    ax.plot(x, y, 'r--')
    ax.plot(x, y, 'go') 
    ax.axis(xmin=dimensions[0], xmax=dimensions[2], ymin=dimensions[1], ymax=dimensions[3])
    for obstacle in obstacles:
        ax.add_patch(plt.Circle((obstacle.x, obstacle.y), radius=obstacle.radius))

    plt.show()


if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Sailbot motion planning CLI.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=5.0, help=
    '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    parser.add_argument('-p', '--planner', default='RRT',
                        choices=['EST', 'KPIECE', 'PDST', 'RRT', 'SyclopEST', 'SyclopRRT'],
                        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    parser.add_argument('-o', '--objective', default='PathLength',
                        choices=['PathClearance', 'PathLength', 'ThresholdPathLength',
                                 'WeightedLengthAndClearanceCombo'],
                        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
    parser.add_argument('-i', '--info', type=int, default=2, choices=[0, 1, 2],
                        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
                             ' Defaults to WARN.')
    parser.add_argument('-w', '--windDirection', type=lambda x: math.radians(int(x)), default=math.radians(-135),
                        help='(Optional) Wind direction in degrees')

    parser.add_argument('-d', '--dimensions', nargs=4, type=int, default=[0, 0, 5, 5],
                        help='(Optional) dimensions of the space')
    parser.add_argument('-ob', '--obstacles', nargs='+', type=parse_obstacle, default=[parse_obstacle("2.5,2.5,1")],
                        help='(Optional) dimensions of the space')

    # Parse the arguments
    args = parser.parse_args()

    # Check that time is positive
    if args.runtime <= 0:
        raise argparse.ArgumentTypeError(
            "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
            % (args.runtime,))

    # Set the log level
    if args.info == 0:
        ou.setLogLevel(ou.LOG_WARN)
    elif args.info == 1:
        ou.setLogLevel(ou.LOG_INFO)
    elif args.info == 2:
        ou.setLogLevel(ou.LOG_DEBUG)
    else:
        ou.OMPL_ERROR("Invalid log-level integer.")

    # Solve the planning problem
    plan(args.runtime, args.planner, args.objective, args.windDirection, args.dimensions,
         args.obstacles)
