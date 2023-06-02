import time

from robot_movement import RobotMovement


# robots for the current simulation
robotMoveObj = RobotMovement()

curName = "Seth"
drawing = -1
message = 0
robotMoveObj.New_Robot(curName, -1.6, -1.0, 0, "black", drawing)

drawing = 16711680
robotMoveObj.Add_Target_Point(curName, 1.6, -1.0, 0, drawing, message)

#drawing = -1 
#robotMoveObj.Add_Target_Point(curName, 1.3, 0.0, 0, drawing, message)
#robotMoveObj.Move_Forward(curName, 0, -10003, message)

#drawing = 16711680
#robotMoveObj.Add_Target_Point(curName, -1.3, 0.0, 0, drawing, message)
#robotMoveObj.Move_Forward(curName, 0, -10003, message)

#drawing = -1 
#robotMoveObj.Add_Target_Point(curName, -1, 0.0, 0, drawing, message)
#robotMoveObj.Add_Target_Point(curName, -1, 0.8, 0, drawing, message)

#drawing = 16711680
#robotMoveObj.Add_Target_Point(curName, -1, -0.8, 0, drawing, message)

"""
curName = "Saul"
drawing = -1
message = 0
robotMoveObj.New_Robot(curName, -1.3, -0.4, 0, "black", drawing)

drawing = 13395456
robotMoveObj.Add_Target_Point(curName, 1.3, -0.4, 0, drawing, message)

drawing = 13395456 # -1
robotMoveObj.Add_Target_Point(curName, 1.3, 0.4, 0, drawing, message)
#robotList[namesList.index(curName)].move_Forward(0, -5003, message)
robotMoveObj.Move_Forward(curName, 0, -5003, message)

drawing = 13395456
robotMoveObj.Add_Target_Point(curName, -1.3, 0.4, 0, drawing, message)
#robotList[namesList.index(curName)].move_Forward(0, -5003, message)
robotMoveObj.Move_Forward(curName, 0, -5003, message)

drawing = 13395456 # -1
robotMoveObj.Add_Target_Point(curName, 0, 0.4, 0, drawing, message)
robotMoveObj.Add_Target_Point(curName, 0, 0.8, 0, drawing, message)

drawing = 13395456
robotMoveObj.Add_Target_Point(curName, 0, -0.8, 0, drawing, message)

"""

# run the simulation
start_time = time.time()
return_message = robotMoveObj.Run_Robotarium()

if return_message == "Done":
    print("--- Whole simulation time spent is: %s seconds ---" % (time.time() - start_time))