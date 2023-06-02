import math
import matplotlib.lines as lines
import matplotlib.pyplot as plt
import numpy as np
import time

import mazeArrays
from robot import Robot
import robotarium

from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *


class RobotMovement:
    """
    The class contains robot movement functions and functions to run the robotarium simulation
    to generate a simulation video at end if no errors are encountered.
    
    """

    def __init__(self):

        self.robotList = [] # list of robot objects
        self.namesList = [] # list of robot names
        self.imageList = [] # list of images
        self.robotColorsList = [] # list of robot's colors
        self.robot_labels = [] # list of the labels that stay above the robots
        self.robotLines = [] # list of the lines that the robots are currently drawing
        self.shapesList = [] # list of the shapes to be drawn
        self.lineList = [] # list of lines to be draw
        self.t_end = time.time() + 60 * 10 # sets a time limit on the simulation
        self.drawing = -1 # set drawing to false initially
        self.drawingType = "Ideal" # set default drawing type to ideal
        self.messageList = [["string here boolean there -> ",False]]

        self.frames = 0 # initialize number of frames check to avoid overload computer
        self.drawFrame = False # initialize draw frame to false

        # maze parameters
        self.ifMaze = False
        self.mazeNum = 0
        self.mazeFailed = False

        _, self.uni_to_si_states = create_si_to_uni_mapping()

        # start the controler that drives the robots
        self.unicycle_position_controller = create_hybrid_unicycle_pose_controller()

        # create barrier certificate
        self.uni_barrier_cert = create_unicycle_barrier_certificate()

    
    # initialize a new robot
    def New_Robot(self, name, x_coord, y_coord, angle, color, drawing):
        self.namesList.append(name)
        self.robotList.append(Robot(name, x_coord, y_coord, angle, drawing))
        self.robotColorsList.append(color)
        self.robotLines.append([])


    # add a point to target array
    def Add_Target_Point(self, name, x_coord, y_coord, angle, drawing, message):
        self.robotList[self.namesList.index(name)].add_Target_Point(x_coord, y_coord, angle, drawing, message)

    
    # move the robot forward
    def Move_Forward(self, name, distance, drawing, message):
        self.robotList[self.namesList.index(name)].move_Forward(distance, drawing, message)


    # turn the robot based on its current direction
    def Turn(self, name, turnAngle, drawing, message):
        self.robotList[self.namesList.index(name)].turn(turnAngle, drawing, message)


    # turn the robot to a specific direction regardless of its current direction
    def Turn_To(self, name, angle, drawing, message):
        self.robotList[self.namesList.index(name)].turn_To(angle, drawing, message)

    
    # add polygon shapes
    def Add_Polygon_Shapes(self, x_coord, y_coord, sides, radius, orientation, color, zorder):
        self.shapesList.append(robotarium.patches.RegularPolygon([x_coord, y_coord], sides, 
            radius, orientation, color, zorder))


    # add circles
    def Add_Circle_Shapes(self, x_coord, y_coord, radius, color, zorder):
        self.shapesList.append(robotarium.patches.Circle([x_coord, y_coord], radius, color, zorder))


    # add rectangles
    def Add_Rectangle_Shapes(self, x_coord, y_coord, width, height, angle, color, zorder):
        self.shapesList.append(robotarium.patches.Rectangle([x_coord, y_coord], width, height, angle, color, zorder))

    
    # add lines
    def Add_Lines(self, fx, tx, fy, ty, color, zorder):
        self.lineList.append(lines.Line2D([fx, tx], [fy, ty], linewidth=1, color=color, zorder=zorder))


    # add images
    def Add_Images(self, filename, x_coord, y_coord, size):
        self.imageList.append([plt.imread(filename), (x_coord, x_coord+size, y_coord, y_coord+size)])

    
    # add maze image
    def Add_Maze(self, filename):
        self.imageList.append([plt.imread(filename), (-1.6, 1.6, -1, 1)])


    # send messages
    def Send_Messages(self, name, drawing, message, message_text):

        for messageNum in self.messageList:
            if messageNum[0] == message_text:
                message = self.messageList.index(messageNum)
                break 
            else:
                self.messageList.append([message_text, False])
                message = len(self.messageList) - 1
        
        self.Add_Target_Point(name, self.robotList[self.namesList.index(name)].target_points[-1][0],
            self.robotList[self.namesList.index(name)].target_points[-1][1],
            self.robotList[self.namesList.index(name)].target_points[-1][2] * 180/math.pi, 
            drawing, message)

    
    # wait until message
    def Wait_Until_Message(self, message_text):
        
        for messageNum in self.messageList:
            if messageNum[0] == message_text:
                # wait message by multiplying it by -1
                message = self.messageList.index(messageNum) * -1
                break 
            else:
                # wait message by multiplying it by -1
                message = len(self.messageList) * -1
                self.messageList.append([message_text, False])

        return message


    # fill the start array with all of the initial robot positions
    def Fill_Start_Array(self):

        start_Array = np.array([[],[],[],[],[]])
        for robot in self.robotList:
            start_Array = np.append(start_Array, robot.get_Output_Points(), axis = 1)
        
        return start_Array


    # consolidare all of the robot's output arrays into a single array
    def Update_Target_Array(self, pos):
        target_Array = np.array([[],[],[],[],[]])
        for robotNum in range(len(self.robotList)):
            self.robotList[robotNum].check_Position(robotNum, pos)
            target_Array = np.append(target_Array, self.robotList[robotNum].get_Output_Points(), axis = 1)
        
        return target_Array


    # check if for every robot, self.finished == true
    def Check_if_All_Done(self):
        for robot in self.robotList:
            if(robot.get_Status() == False):
                return False
        return True


    def Maze_Check(self, mazeNum, pos, robotNum, error = 0.03):

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

    def Maze_Check_End(self, pos, robotNum, error = 0.05):

        cur_x = pos[0][robotNum]
        cur_y = pos[1][robotNum]

        if((0.28556+error >= cur_x > .11189-error) and (-0.75299+error >= cur_y > -0.92582-error)):
            return True

        return False

    
    def check_errors(self, errors):

        # check if there are errors in the simulation
        return_message = ""

        if errors:
            print("there be errors yo")
            if "boundary" in errors:

                message1 = "ERROR: Position(s) of robots are found to be outside of boundary"
                message2 = "Here are the erroneous positions:"
                message3 = errors["boundary_positions"]
                message_all = message1 + "\n" + message2 + "\n" + message3
                return_message += message_all # store the error messages

                print(message_all)

            if "collision" in errors:

                message1 = "ERROR: Robots are found to collide with each other with space less than 0.11 meter"
                message2 = "Here are the erroneous positions:"
                message3 = str(errors["collision_positions"])
                message_all = message1 + "\n" + message2 + "\n" + message3
                return_message += message_all # store the error messages

                print(message_all)

            print("Exit out of the simulation now")

        return return_message


    def draw_robot_ideal(self):

        # draw robots ideal
        if self.drawingType == "Ideal":
            for robotNum in range(len(self.robotList)):
                if self.robotList[robotNum].get_Status() == False:
                    if self.robotList[robotNum].currentPoint != 0:
                        if self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint - 1][3] >= 0:
                            self.robotLines[robotNum][0].set_data([self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint - 2][0], 
                                self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint - 1][0]], 
                                [self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint - 2][1], 
                                self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint - 1][1]])
                    
                    if self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][3] >= 0:
                        if self.lineStepCache[robotNum] != self.robotList[robotNum].currentPoint:
                            self.tempColor = hex(int(self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][3]))
                            self.tempColor = "#" + self.tempColor[2:]
                            while len(self.tempColor) != 7: 
                                self.tempColor = self.tempColor[:2] + "0" + self.tempColor[2:]
                            self.robotLines[robotNum] = self.robotariumObj.axes.plot([0,0], [0,0], color=self.tempColor, marker='o', linewidth=1, markersize=6, zorder=10)
                            self.lineStepCache[robotNum] = self.robotList[robotNum].currentPoint
                        self.robotLines[robotNum][0].set_data([self.xpos[0][robotNum], self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint - 1][0]], 
                            [self.xpos[1][robotNum], 
                            self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint - 1][1]])


    def construct_messages(self):

        # message goes here
        for robotNum in range(len(self.robotList)):
            try:
                robotMessageState = self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][4]
      
            except:
                # the robot is finished
                continue

            if robotMessageState > 0:
                self.messageList[robotMessageState][1] = True

            elif robotMessageState < 0:

                if self.messageList[robotMessageState * -1][1] == False:
                    self.dxu[0:2, robotNum] = 0

    
    def construct_waiting(self):

        # wait happens here
        for robotNum in range(len(self.robotList)):

            if self.robotList[robotNum].get_Status() == False:
                if self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][3] <= -3:
                    self.robotList[robotNum].waiting =  (self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][3] + 3) * -1 
                    tempList = list(self.robotList[robotNum].target_points[self.robotList[robotNum].currentPoint])
                    tempList[3] = -2
                    self.robotList[robotNum].target_points[self.robotList[robotNum].currentPoint] = tuple(tempList)
                    
                if self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][3] == -2:
                    self.robotList[robotNum].waiting -= 33
                
                    if self.robotList[robotNum].waiting <= 0:
                        tempList = list(self.robotList[robotNum].target_points[self.robotList[robotNum].currentPoint])
                        tempList[3] = -1
                        self.robotList[robotNum].target_points[self.robotList[robotNum].currentPoint] = tuple(tempList)

    
    def draw_robot_paths(self):

        # draw points at fixed intervals behind robots with their pen down
        if self.drawingType == "Actual":
            for robotNum in range(len(self.robotList)):
                if self.robotList[robotNum].get_Status() == False:
                    if self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][3] >= 0  and self.frames == 0 :
                        tempColor = hex(int(self.robotList[robotNum].get_Target_Points()[self.robotList[robotNum].currentPoint][3]))
                        tempColor = "#" + tempColor[2:]
                        while len(tempColor) != 7: 
                            tempColor = tempColor[:2] + "0" + tempColor[2:]
                        self.robotariumObj.axes.plot(self.xpos[0][robotNum], self.xpos[1][robotNum], color=tempColor, 
                            marker='o', linestyle='dashed', linewidth=1, markersize=6)
                        self.drawFrame = True
          
      
            if(self.drawFrame):
                self.frames = 10
                self.drawFrame = False
            if self.frames > 0:
                self.frames -= 1

    
    def draw_robot_names(self):

        # draw robot names
        for robotname in self.namesList:
            nameindex = self.namesList.index(robotname)
            self.robot_labels[nameindex].set_position([self.xpos_single[0, nameindex], 
                self.xpos_single[1, nameindex]+0.15])

    
    def check_maze(self):

        # check maze
        if(self.ifMaze):
            for robotNum in range(len(self.robotList)):
                if(self.Maze_Check(self.mazeNum, self.xpos, robotNum)):
                    self.mazeFailed = True;
                    self.mazeText.set(text = "Maze failed " + str(self.namesList[robotNum]) + " drove over a line")
                if(self.Maze_Check_End(self.xpos, robotNum)):
                    if(not self.mazeFailed):
                        self.mazeText.set(color = "green", text = "Maze Succeeded, Congratulations")


    def Run_Robotarium(self):

        return_message = "Done"

        start_Array = self.Fill_Start_Array()

        # initializes the robotarium object
        self.robotariumObj = robotarium.Robotarium(number_of_robots=len(self.robotList), show_figure=True, 
            initial_conditions=start_Array[0:3][:], sim_in_real_time=False)

        # get robot positions
        self.xpos = self.robotariumObj.get_poses()

        # run robot steps and check for errors
        errors = self.robotariumObj.step()

        # check if there are errors
        status = self.check_errors(errors)

        if "ERROR" in status:

            return_message = status
            print(return_message)
            return return_message

        # set up robot labels used in the current simulation
        self.robot_labels = [self.robotariumObj.axes.text(self.xpos[0, self.namesList.index(kk)], self.xpos[1, self.namesList.index(kk)]+0.15, 
            kk, fontsize=8, color=self.robotColorsList[self.namesList.index(kk)], fontweight='bold', horizontalalignment='center',
            verticalalignment='center', zorder=0) for kk in self.namesList]

        self.mazeText = self.robotariumObj.axes.text(0, 0.85, "", color="red", ha = "center", size="large", weight="bold")

        # for ideal drawing, set up a place for the step of the previous line to be stored
        if self.drawingType == "Ideal":
            self.lineStepCache = np.zeros(len(self.robotList), dtype=np.int64).tolist()

        for shape in self.shapesList:
            self.robotariumObj.axes.add_patch(shape)

        for line in self.lineList:
            self.robotariumObj.axes.add_line(line)

        for image in self.imageList:
            self.robotariumObj.axes.imshow(image[0], extent=image[1], zorder=-10)

        # loop that runs everything
        while time.time() < self.t_end:
  
            # ideal drawing. It creates a line for each drawing step and then drags one end 
            # with the robot leaving the other end at the start of the step
            self.draw_robot_ideal()

            # get robot positions
            self.xpos = self.robotariumObj.get_poses()

            # take unicycle states and return single-integrator states
            self.xpos_single =  self.uni_to_si_states(self.xpos) # xi

            # start the controller that drives the robots
            self.dxu = self.unicycle_position_controller(self.xpos, self.Update_Target_Array(self.xpos))

            # construct messages
            self.construct_messages()

            # create barrier certificate
            self.dxu = self.uni_barrier_cert(self.dxu, self.xpos)

            # set velocities of the robots
            self.robotariumObj.set_velocities(np.arange(len(self.robotList)), self.dxu) 

            # construct waiting
            self.construct_waiting()

            # draw robot paths
            self.draw_robot_paths()

            # draw robot names
            self.draw_robot_names()

            # run robot steps and check for errors
            errors = self.robotariumObj.step()

            # check if there are errors
            status = self.check_errors(errors)

            if "ERROR" in status:

                return_message = status
                return return_message

            # check if all robots are done processing
            if(self.Check_if_All_Done()):
                break 
            
            # check on maze
            self.check_maze()

        # generate video for the simulation at end of the program
        start_time = time.time()
        return_message = self.robotariumObj.call_at_scripts_end()

        if return_message == "Done":
            print("--- Video generation time spent is: %s seconds ---" % (time.time() - start_time))

        return return_message