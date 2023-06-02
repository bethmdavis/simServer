import math
import numpy as np


class Robot:

    # set up the initial conditions for the  blocks to modify 
    def __init__(self, name, x, y, t, drawing):
        self.target_points = []
        self.currentPoint = 0
        self.totalPoints = 0
        message = 0
        self.finished = False
        self.output_points = np.array([[],[],[]])
        t = t % 360
        if(t > 180):
            t = t - 360
        theta = t * math.pi/180
        self.name = name
        self.start_array = np.array([[x], [y], [theta], [drawing], [message]])
        self.target_points.append((x, y, theta, drawing, message))
        self.output_points = np.array([[x], [y], [theta], [drawing], [message]])
        self.waiting = 0
    
    #this block adds a target point to the target_points array for a specific robot
    def add_Target_Point(self, x, y, t, drawing, message):
        self.totalPoints = self.totalPoints + 1
        t = t % 360
        if(t > 180):
            t = t - 360
        self.target_points.append((x, y, t*math.pi/180, drawing, message))

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
      elif(((self.target_points[self.currentPoint][2]-accuracy+math.pi) < angle + 3* math.pi < (self.target_points[self.currentPoint][2]+accuracy+math.pi))):
              return True
      elif(((self.target_points[self.currentPoint][2]-accuracy+math.pi) < angle - math.pi < (self.target_points[self.currentPoint][2]+accuracy+math.pi))):
              return True        
      else:
          return False

    #checks if a robot is within the allowed distance of a point
    def check_Position(self, robot_num, pos, accuracy = .03):
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
    def turn(self, by, drawing, message):
          curX = self.target_points[self.totalPoints][0]
          curY = self.target_points[self.totalPoints][1]
          curT = self.target_points[self.totalPoints][2]*180/math.pi
          curT = (curT + by)%360
          self.add_Target_Point((curX),(curY),(curT),drawing, message)

    #turns the robot to a specified angle on the spot
    def turn_To(self, to, drawing, message):
        curX = self.target_points[self.totalPoints][0]
        curY = self.target_points[self.totalPoints][1]
        self.add_Target_Point((curX),(curY),(to),drawing, message)