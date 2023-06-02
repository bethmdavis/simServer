import robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time

import math
pi = math.pi

# Instantiate Robotarium object
N = 1
initial_conditions = np.array(np.mat('0.4; 0.4; 0'))
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,sim_in_real_time=False)

# Define goal points by removing orientation from poses
goal_points = generate_initial_conditions(N)

# Create unicycle position controller
unicycle_position_controller = create_clf_unicycle_position_controller()

# Create barrier certificates to avoid collision
uni_barrier_cert = create_unicycle_barrier_certificate()


radius=0.4
n=20
#points = [np.array([[math.cos(2*pi/n*x)*radius+0.2,math.sin(2*pi/n*x)*radius+0.2,0]]) for q in range(0,n+1)]
#points = np.asmatrix(np.array([[-0.5,0.5,0.5,-0.5],[0.5,0.5,-0.5,-0.5],[0,0,0,0]]))

poses = []

for x in range(n):
    pose = np.zeros((3, N))
    pose[0, 0] = math.cos(2*pi/n*x)*radius
    pose[1, 0] = math.sin(2*pi/n*x)*radius
    pose[2, 0] = 0
    poses.append(pose)



# define x initially
x = r.get_poses()
r.step()

# While the number of robots at the required poses is less
# than N...

for goal_points in poses:
    line = r.axes.plot(goal_points[0],goal_points[1],color='green', marker='o', linestyle='dashed', linewidth=2, markersize=12)
    while (np.size(at_pose(x, goal_points, rotation_error=100)) != N):

        # Get poses of agents
        x = r.get_poses()

        # Create single-integrator control inputs
        dxu = unicycle_position_controller(x, goal_points[:2][:])

        # Create safe control inputs (i.e., no collisions)
        dxu = uni_barrier_cert(dxu, x)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu*10)

        # Iterate the simulation
        r.step()

#Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()


