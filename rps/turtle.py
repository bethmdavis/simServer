import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time
import math
from copy import copy

# Instantiate Robotarium object
N = 1
initial_conditions = np.array(np.mat('0.1; 0.1; 0'))
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=True)

# Define goal points by removing orientation from poses
# goal_points = generate_initial_conditions(N)


# Create unicycle position controller
unicycle_position_controller = create_clf_unicycle_position_controller()

# Create barrier certificates to avoid collision
uni_barrier_cert = create_unicycle_barrier_certificate()

# define x initially
x = r.get_poses()
r.step()

# While the number of robots at the required poses is less
# than N...
def createPoint(x,y,angle):
    pose = np.zeros((3, N))
    pose[0, 0] = x
    pose[1, 0] = y
    pose[2, 0] = angle

    return pose

def moveToPoint(point,x):
    while (np.size(at_pose(x, point, rotation_error=100)) != N):

        # Get poses of agents
        x = r.get_poses()

        # Create single-integrator control inputs
        dxu = unicycle_position_controller(x, point[:2][:])

        # Create safe control inputs (i.e., no collisions)
        dxu = uni_barrier_cert(dxu, x)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu)

        # Iterate the simulation
        r.step()

def actualTurn(point,x, error,rightBool):
    while (point[2,0] > x[2,0] + error or point[2,0] < x[2,0] - error):
        print(point[2,0])
        print(x[2,0])
        print("---------")
        # Get poses of agents
        x = r.get_poses()

        # Create single-integrator control inputs
        dxu = unicycle_position_controller(x, point[:2][:])

        # Create safe control inputs (i.e., no collisions)
        dxu = uni_barrier_cert(dxu, x)

        if(rightBool):
            dxu[1,0] = -0.5
        else:
            dxu[1,0] = 0.5
        
        print(dxu)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu)

        # Iterate the simulation
        r.step()

def moveForward(stepSize):
    x = r.get_poses()
    r.step()
    xMovement = stepSize*math.cos(x[2,0])
    yMovement = stepSize*math.sin(x[2,0])
    point = createPoint(x[0,0]+xMovement,x[1,0]+yMovement,0)
    moveToPoint(point,x)

def turnAngle(angle,rightBool):
    if(rightBool):
        angle*=-1
    x = r.get_poses()
    r.step()
    newX = copy(x)
    newX[2,0] = x[2,0] + angle
    if(newX[2,0] > math.pi):
        newX[2,0] = newX[2,0] - 2*math.pi
    elif(newX[2,0] < -1*math.pi):
        newX[2,0] = newX[2,0] + 2*math.pi
    actualTurn(newX,x,0.01,rightBool)

moveForward(0.5)
turnAngle(1.5708,True)
moveForward(0.5)
turnAngle(1.5708,True)
moveForward(0.5)
turnAngle(1.5708,True)
moveForward(0.5)
turnAngle(1.5708,True)



#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()