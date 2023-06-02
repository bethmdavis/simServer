"""import robotarium as robotarium
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
r.call_at_scripts_end()"""

from glob import glob
from turtle import position
import robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time 

robotList = []
namesList = []

class Robot:

    def __init__(self, name, x, y, t):
        self.target_points = []
        self.currentPoint = 0
        self.totalPoints = 0
        self.finished = False
        self.output_points = np.array([[],[],[]])
        theta = t * 3.14/180
        self.name = name
        self.start_array = np.array([[x],[y],[theta]])
        self.target_points.append((x,y,theta))
        self.output_points = np.array([[x],[y],[theta]])
    
    def add_Target_Point(self,x,y,t):
        self.totalPoints = self.totalPoints + 1
        self.target_points.append((x,y,t*3.14/180))

    def get_Target_Points(self):
        return self.target_points

    def update_Output_Points(self):
        self.output_points[0] = self.target_points[self.currentPoint][0]
        self.output_points[1] = self.target_points[self.currentPoint][1]
        self.output_points[2] = self.target_points[self.currentPoint][2]
    
    def step_Current_Point(self):
        self.currentPoint = self.currentPoint + 1
        if(self.currentPoint > self.totalPoints):
            self.finished = True
        if(not self.finished):
            self.update_Output_Points()
    
    def get_Output_Points(self):
        return self.output_points

    def check_Position(self,robot_num, pos, accuracy = .03):
        if(not self.finished):
            if((self.target_points[self.currentPoint][0]-accuracy< pos[0][robot_num] < self.target_points[self.currentPoint][0]+accuracy) and (self.target_points[self.currentPoint][1]-accuracy< pos[1][robot_num] < self.target_points[self.currentPoint][1]+accuracy)):
                self.step_Current_Point()
                return True
            else:
                return False
    def get_Status(self):
        return self.finished


def Fill_Start_Array():
    start_Array = np.array([[],[],[]])
    for robot in robotList:
        start_Array = np.append(start_Array,robot.get_Output_Points(), axis = 1)
    return start_Array

def Update_Target_Array(pos):
    target_Array = np.array([[],[],[]])
    for i in range(len(robotList)):
        robotList[i].check_Position(i,pos)
        target_Array = np.append(target_Array,robotList[i].get_Output_Points(), axis = 1)
    print(target_Array)
    return target_Array

def Check_if_All_Done():
    for robot in robotList:
        if(robot.get_Status() == False):
            return False
    return True

def New_Robot(name, x,y,t):
    namesList.append(name)
    robotList.append(Robot(name,x,y,t))

def Add_Target_Point(name, x,y,t):
    robotList[namesList.index(name)].add_Target_Point(x,y,t)

New_Robot("Bob",0,0,0)
New_Robot("Kim",.8,.8,90)

Add_Target_Point("Bob",.8,.8,180)
Add_Target_Point("Kim",0,0,90)

r = robotarium.Robotarium(number_of_robots=len(robotList), show_figure=True, initial_conditions=Fill_Start_Array(), sim_in_real_time=False)
unicycle_position_controller = create_hybrid_unicycle_pose_controller()
uni_barrier_cert = create_unicycle_barrier_certificate()
x = r.get_poses()
r.step()

t_end = time.time() + 60 * 5
while time.time() < t_end:
    x = r.get_poses()
    dxu = unicycle_position_controller(x, Update_Target_Array(x))
    dxu = uni_barrier_cert(dxu, x)
    r.set_velocities(np.arange(len(robotList)), dxu)    
    r.step()
    if(Check_if_All_Done()):
        break

r.call_at_scripts_end()

