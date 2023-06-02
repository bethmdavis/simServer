import rps.robotarium as robotarium
ImageRoot= ""

# importing necesary robotarium libraries
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import matplotlib.lines as lines

# importing other necessary libraries
import numpy as np
import time
import math

#Here we define the Robot class in order to condense the code output by the blocks and make that easier to read
class Robot:

    #This sets up theInitial conditions for the  blocks to modify 
    def __init__(self, name, x, y, t, drawing):
        self.target_points = []
        self.currentPoint = 0
        self.totalPoints = 0
        message = 0
        self.finished = False
        self.output_points = np.array([[],[],[]])
        t=t%360
        if(t>180):
            t=t-360
        theta = t * math.pi/180
        self.name = name
        self.start_array = np.array([[x],[y],[theta],[drawing],[message]])
        self.target_points.append((x,y,theta,drawing,message))
        self.output_points = np.array([[x],[y],[theta],[drawing],[message]])
        self.waiting = 0
    
    #this block adds a target point to the target_points array for a specific robot
    def add_Target_Point(self,x,y,t,drawing,message):
        self.totalPoints = self.totalPoints + 1
        t=t%360
        if(t>180):
            t=t-360
        self.target_points.append((x,y,t*math.pi/180,drawing,message))

    #returns the 2D target points aray for a specific robot
    def get_Target_Points(self):
        return self.target_points

    #This turns the tuple array of self.target points into the array of strings that other functions require
    def update_Output_Points(self):
        self.output_points[0] = self.target_points[self.currentPoint][0]
        self.output_points[1] = self.target_points[self.currentPoint][1]
        self.output_points[2] = self.target_points[self.currentPoint][2]
        self.output_points[3] = self.target_points[self.currentPoint][3]
        self.output_points[4] = self.target_points[self.currentPoint][4]
    
    #This function moves the robot to the next step and if it is finished, it sets self.finished to true
    def step_Current_Point(self):
        if self.get_Target_Points()[self.currentPoint][3] >= -1:
          self.currentPoint = self.currentPoint + 1
        if(self.currentPoint > self.totalPoints):
            self.finished = True
        if(not self.finished):
            self.update_Output_Points()
    
    #Displays output points
    def get_Output_Points(self):
        return self.output_points
    
    #checks if a robot is within the allowed degrees of a specified angle
    def angle_In_Range(self, angle, accuracy = .07):
      if((self.target_points[self.currentPoint][2]-accuracy < angle < self.target_points[self.currentPoint][2]+accuracy)):
              return True     
      elif(self.target_points[self.currentPoint][2] == math.pi):
          if(math.pi-accuracy < math.fabs(angle) < math.pi+accuracy):
              return True
      else:
          return False

    #checks if a robot is within the allowed distance of a point
    def check_Position(self,robot_num, pos, accuracy = .03):
        if(not self.finished):
            if((self.target_points[self.currentPoint][0]-accuracy < pos[0][robot_num] < self.target_points[self.currentPoint][0]+accuracy) and (self.target_points[self.currentPoint][1]-accuracy < pos[1][robot_num] < self.target_points[self.currentPoint][1]+accuracy) and (self.angle_In_Range(pos[2][robot_num]))):
                self.step_Current_Point()
                return True
            else:
                return False

    #returns of a robot is finished or not
    def get_Status(self):
        return self.finished

    #Sets a point the specified distance ahead of the robot at the same ange as the starting angle.
    #it then adds that point to the target array
    def move_Forward(self, distance, drawing, message):
        curX = self.target_points[self.totalPoints][0]
        curY = self.target_points[self.totalPoints][1]
        curT = self.target_points[self.totalPoints][2]
        addX = distance * math.cos(curT)
        addY = distance * math.sin(curT)
        self.add_Target_Point((curX+addX),(curY+addY),(curT*180/math.pi),drawing, message)
    
    #adds a target point at the robot's current position, but turned by the specified angle
    def turn(self,by,drawing, message):
          curX = self.target_points[self.totalPoints][0]
          curY = self.target_points[self.totalPoints][1]
          curT = self.target_points[self.totalPoints][2]*180/math.pi
          curT = (curT + by)%360
          self.add_Target_Point((curX),(curY),(curT),drawing, message)

    #turns the robot to a specified angle on the spot
    def turn_To(self,to,drawing, message):
        curX = self.target_points[self.totalPoints][0]
        curY = self.target_points[self.totalPoints][1]
        self.add_Target_Point((curX),(curY),(to),drawing, message)

#necessary lists
#list of robot objects
robotList = []
#list of the robot's names
namesList = []
#list of images
imageList = []
#list of the robot's colors
robotColorsList = []
#list of the label objecs that stay above the robots displaying their names
robot_labels = []
#list of the ideal lines that the robots are currently drawing
robotLines = []
#list of the shapes to be drawn
shapesList = []
#list of lines to be draw
lineList = []
#sets a time limit on the simulation
t_end = time.time() + 60 * 5
#sets initialy not drawing
drawing = -1
#sets default drawing type to ideal
drawingType = "Ideal"
messageList = [["string here boolean there -> ",False]]

#fills the start array with all of the initial robot positions
def Fill_Start_Array():
    start_Array = np.array([[],[],[],[],[]])
    for robot in robotList:
        start_Array = np.append(start_Array,robot.get_Output_Points(), axis = 1)
    return start_Array

#Consolidares all of the robot's output arrays into a single array
def Update_Target_Array(pos):
    target_Array = np.array([[],[],[],[],[]])
    for i in range(len(robotList)):
        robotList[i].check_Position(i,pos)
        target_Array = np.append(target_Array,robotList[i].get_Output_Points(), axis = 1)
    return target_Array

#checks if for every robot, self.finished == true
def Check_if_All_Done():
    for robot in robotList:
        if(robot.get_Status() == False):
            return False
    return True

#initializes a new robot
def New_Robot(name, x,y,t,color):
    namesList.append(name)
    robotList.append(Robot(name,x,y,t,drawing))
    robotColorsList.append(color)
    robotLines.append([])

#adds passed in point to target array
def Add_Target_Point(name, x,y,t,drawing, message):
    robotList[namesList.index(name)].add_Target_Point(x,y,t,drawing, message)


def mazeCheck(mazeNum, pos, robotNum, error = 0.03):
    cur_x = pos[0][robotNum]
    cur_y = pos[1][robotNum]
    mazeArray = mazeArrays.mazeArrays[mazeNum-1]

    y_line = round((cur_y/0.02469)-40.5)*-1
    y_line_minus = y_line-1
    y_line_plus = y_line+1

    for x in mazeArray[y_line]:
        if(x+error >= cur_x > x-error):
            return True
    for x in mazeArray[y_line_minus]:
        if(x+error >= cur_x > x-error):
            return True
    for x in mazeArray[y_line_plus]:
       if(x+error >= cur_x > x-error):
            return True
    return False

def mazeCheckEnd(pos,robotNum, error = 0.05):
    cur_x = pos[0][robotNum]
    cur_y = pos[1][robotNum]
    if((0.28556+error >= cur_x > .11189-error) and (-0.75299+error >= cur_y > -0.92582-error)):
        return True
    return False

IfMaze = False
MazeNum = 0
mazeFailed = False

curName = "Bob"
drawing = -1
message = 0
New_Robot(curName,0,0,90,"black")
Add_Target_Point(curName,0.8,0.8,0,drawing, message)


curName = "Georgie"
drawing = -1
message = 0
New_Robot(curName,-1,-1,90,"black")
robotList[namesList.index(curName)].move_Forward(1,drawing, message)

start_Array = Fill_Start_Array()
#initializes the robotarium object
r = robotarium.Robotarium(number_of_robots=len(robotList), show_figure=True, initial_conditions=start_Array[0:3][:], sim_in_real_time=False)
#starts the controler that drives the robots
unicycle_position_controller = create_hybrid_unicycle_pose_controller()
#creates barrier certificate
uni_barrier_cert = create_unicycle_barrier_certificate()
_,uni_to_si_states = create_si_to_uni_mapping()
#gets robot positions
x = r.get_poses()
r.step()

#generates robot list
robot_labels = [r.axes.text(x[0,namesList.index(kk)],x[1,namesList.index(kk)]+0.15,kk,fontsize=8, color=robotColorsList[namesList.index(kk)],fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
for kk in namesList]

#ii is necessary so that the robot draws every few frames and doesn't overload the computer
ii = 0
#used for actual to track drawing
drawFrame = False

mazeText = r.axes.text(0,.85,"", color="red", ha = "center", size="large", weight="bold")
#for ideal drawing, it sets up a place for the step of the previous line to be stored
if drawingType == "Ideal":
    lineStepCache = []
    for nn in robotList:
        lineStepCache.append(0)

for s in shapesList:
  r.axes.add_patch(s)
for l in lineList:
  r.axes.add_line(l)
for i in imageList:
  r.axes.imshow(i[0], extent=i[1], zorder=-10)
#loop that runs everything
while time.time() < t_end:
  
    #Ideal drawing. It creates a line for each drawing step and then drags one end with the robot leaving the other end at hte start of the step
    if drawingType == "Ideal":
        for i in range(len(robotList)):
          if robotList[i].get_Status() == False:
            if robotList[i].currentPoint != 0:
              if robotList[i].get_Target_Points()[robotList[i].currentPoint - 1][3] >= 0:
                robotLines[i][0].set_data([robotList[i].get_Target_Points()[robotList[i].currentPoint - 2][0],robotList[i].get_Target_Points()[robotList[i].currentPoint - 1][0]],[robotList[i].get_Target_Points()[robotList[i].currentPoint - 2][1],robotList[i].get_Target_Points()[robotList[i].currentPoint - 1][1]])
            if robotList[i].get_Target_Points()[robotList[i].currentPoint][3] >= 0:
              if lineStepCache[i] != robotList[i].currentPoint:
                tempColor = hex(int(robotList[i].get_Target_Points()[robotList[i].currentPoint][3]))
                tempColor = "#" + tempColor[2:]
                while len(tempColor) != 7: 
                  tempColor = tempColor[:2] + "0" + tempColor[2:]
                robotLines[i] = r.axes.plot([0,0],[0,0],color=tempColor, marker='o', linewidth=1, markersize=6,zorder=10)
                lineStepCache[i] = robotList[i].currentPoint
              robotLines[i][0].set_data([x[0][i],robotList[i].get_Target_Points()[robotList[i].currentPoint - 1][0]],[x[1][i],robotList[i].get_Target_Points()[robotList[i].currentPoint - 1][1]])
    
    #get robot positions
    x = r.get_poses()
    #converts robot positions needed for some functions
    xi =  uni_to_si_states(x)

    dxu = unicycle_position_controller(x, Update_Target_Array(x))

    #message goes here
    for i in range(len(robotList)):
      try:
        robotMessageState = robotList[i].get_Target_Points()[robotList[i].currentPoint][4]
      # except:
      #   print(namesList[i])
      #   print(i)
      #   print(robotList[i].currentPoint)
      #   print(robotList[i].get_Target_Points())
      except:
        foo = 0
        #this robot is finished
      if robotMessageState > 0:
        messageList[robotMessageState][1] = True
      elif robotMessageState < 0:
        if messageList[robotMessageState * -1][1] == False:
          dxu[0:2,i] = 0

    dxu = uni_barrier_cert(dxu, x)
    r.set_velocities(np.arange(len(robotList)), dxu)    
    
    #wait happens here
    for i in range(len(robotList)):
      if robotList[i].get_Status() == False:
        if robotList[i].get_Target_Points()[robotList[i].currentPoint][3] <= -3:
          robotList[i].waiting =  (robotList[i].get_Target_Points()[robotList[i].currentPoint][3] + 3) * -1 
          tempList = list(robotList[i].target_points[robotList[i].currentPoint])
          tempList[3] = -2
          robotList[i].target_points[robotList[i].currentPoint] = tuple(tempList)
          #print (robotList[i].target_points[robotList[i].currentPoint])
          
        if robotList[i].get_Target_Points()[robotList[i].currentPoint][3] == -2:
          #print(robotList[i].waiting)
          robotList[i].waiting -= 33
          if robotList[i].waiting <= 0:
            tempList = list(robotList[i].target_points[robotList[i].currentPoint])
            tempList[3] = -1
            robotList[i].target_points[robotList[i].currentPoint] = tuple(tempList)


    #draws points at fixed intervals behind robots with their pen down
    if drawingType == "Actual":
      for i in range(len(robotList)):
        if robotList[i].get_Status() == False:
          if robotList[i].get_Target_Points()[robotList[i].currentPoint][3] >= 0  and ii == 0 :
            tempColor = hex(int(robotList[i].get_Target_Points()[robotList[i].currentPoint][3]))
            tempColor = "#" + tempColor[2:]
            while len(tempColor) != 7: 
                  tempColor = tempColor[:2] + "0" + tempColor[2:]
            r.axes.plot(x[0][i],x[1][i],color=tempColor, marker='o', linestyle='dashed', linewidth=1, markersize=6)
            drawFrame = True
          
      
      if(drawFrame):
        ii = 10
        drawFrame = False
      if ii > 0:
        ii -= 1
    
  


    #draws names
    for robotname in namesList:
      nameindex = namesList.index(robotname)
      robot_labels[nameindex].set_position([xi[0,nameindex],xi[1,nameindex]+0.15])
      
    #ends while loop when the program is finished
    r.step()
    if(Check_if_All_Done()):
        break
    if(IfMaze):
      for i in range(len(robotList)):
        if(mazeCheck(MazeNum, x, i)):
          mazeFailed = True;
          mazeText.set(text = "Maze failed " + str(namesList[i]) + " drove over a line")
        if(mazeCheckEnd(x,i)):
          if(not mazeFailed):
            mazeText.set(color = "green",text = "Maze Succeeded, Congratulations")

    
    
#run at end of the program
r.call_at_scripts_end()
