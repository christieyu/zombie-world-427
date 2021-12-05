"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
import math
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs
import numpy as np
import cv2 as cv

# def camera_detection(camera):
#     print("camera detection")
    
#     # get image from camera sensor
#     pic = camera.getImageArray()
#     result = {}
#     if pic:
#         # convert rgb bytes to hsv array
#         pic = np.uint8(pic)
#         hsv = cv.cvtColor(pic, cv.COLOR_BGR2HSV)
#         # define color thresholds
#         ranges =    {
#                         "black" : [np.array([0, 0, 0]), np.array([180, 255, 46])],\
#                         "grey" : [np.array([0, 0, 46]), np.array([180, 43, 220])],\
#                         "white" : [np.array([0, 0, 221]), np.array([180, 30, 225])],\
#                         "red" : [np.array([0, 43, 46]), np.array([10, 255, 225])],\
#                         "red2" : [np.array([156, 43, 46]), np.array([180, 255, 225])],\
#                         "yellow" : [np.array([26, 43, 46]), np.array([34, 255, 225])],\
#                         "green" : [np.array([35, 43, 46]), np.array([77, 255, 225])],\
#                         "aqua" : [np.array([78, 43, 46]), np.array([99, 255, 225])],\
#                         "blue" : [np.array([100, 43, 46]), np.array([124, 255, 255])], \
#                         "orange" : [np.array([11, 43, 46]), np.array([25, 255, 255])],\
#                         "purple" : [np.array([11, 43, 46]), np.array([25, 255, 255])]
#                     }
#         #add the corresponding color point array into the result dict
#         for col_key in ranges:
#             pic_copy = hsv
#             curr = cv.inRange(pic_copy, ranges[col_key][0], ranges[col_key][1])
#             # print(curr)
#             if col_key not in result:
#                 result[col_key] = []
            
#             #TODO: check output format of cv.inRange
#                 #print("size is ", curr.size)
                
#                 print(curr.size, curr.size)
#                 for r in range (curr[0].size):
#                     for c in range (curr[1].size):
#                         if curr[r][c] == 255:
#                             result[col_key].append((r, c))
#             print(result)
#             # #result[col_key].append(curr)
       
#     return result


#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    # accelerometer = robot.getDevice("accelerometer")
    # accelerometer.enable(timestep)
    
    # gps = robot.getDevice("gps")
    # gps.enable(timestep)
    
    # compass = robot.getDevice("compass")
    # compass.enable(timestep)
    
    # camera1 = robot.getDevice("ForwardLowResBigFov")
    # camera1.enable(timestep)
    
    # camera2 = robot.getDevice("ForwardHighResSmallFov")
    # camera2.enable(timestep)
    
    # camera3 = robot.getDevice("ForwardHighRes")
    # camera3.enable(timestep)
    
    # camera4 = robot.getDevice("ForwardHighResSmall")
    # camera4.enable(timestep)
    
    # camera5 = robot.getDevice("BackLowRes")
    # camera5.enable(timestep)
    
    # camera6 = robot.getDevice("RightLowRes")
    # camera6.enable(timestep)
    
    # camera7 = robot.getDevice("LeftLowRes")
    # camera7.enable(timestep)
    
    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)
    
    # gyro = robot.getDevice("gyro")
    # gyro.enable(timestep)
    
    # lightSensor = robot.getDevice("light sensor")
    # lightSensor.enable(timestep)
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    
    # rangeFinder = robot.getDevice("range-finder")
    # rangeFinder.enable(timestep)
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))
    
    #test camera
    i=0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 8):
        
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            
            
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
        
        # get image from camera sensor
        pic = camera8.getImageArray()
        result = {}
        if pic:
            # convert rgb bytes to hsv array
            pic = np.uint8(pic)
            hsv = cv.cvtColor(pic, cv.COLOR_BGR2HSV)
            # define color thresholds
            ranges =    {
                            "black" : [np.array([0, 0, 0]), np.array([180, 255, 46])],\
                            "grey" : [np.array([0, 0, 46]), np.array([180, 43, 220])],\
                            "white" : [np.array([0, 0, 221]), np.array([180, 30, 225])],\
                            "red" : [np.array([0, 43, 46]), np.array([10, 255, 225])],\
                            "red2" : [np.array([156, 43, 46]), np.array([180, 255, 225])],\
                            "yellow" : [np.array([26, 43, 46]), np.array([34, 255, 225])],\
                            "green" : [np.array([35, 43, 46]), np.array([77, 255, 225])],\
                            "aqua" : [np.array([78, 43, 46]), np.array([99, 255, 225])],\
                            "blue" : [np.array([100, 43, 46]), np.array([124, 255, 255])], \
                            "orange" : [np.array([11, 43, 46]), np.array([25, 255, 255])],\
                            "purple" : [np.array([11, 43, 46]), np.array([25, 255, 255])]
                        }
            #add the corresponding color point array into the result dict
            for col_key in ranges:
                pic_copy = hsv
                curr = cv.inRange(pic_copy, ranges[col_key][0], ranges[col_key][1])
                # print(curr)
                if col_key not in result:
                    result[col_key] = []
                
                #TODO: check output format of cv.inRange
                    #print("size is ", curr.size)
                    
                    print(curr.size, curr.size)
                    for r in range (curr[0].size):
                        for c in range (curr[1].size):
                            if curr[r][c] == 255:
                                result[col_key].append((r, c))
                print(result)
                # #result[col_key].append(curr)
        # range_image = lidar.getRangeImage()
        # range_image = lidar.getRangeImage()
        # print(range_image)
        
         #called every timestep
        
        
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        #if i <100
            #base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        #if == 100 
            # base_reset() 
            # base_turn_left()  
            #it takes about 150 timesteps for the robot to complete the turn
                 
        #if i==300
            # i = 0
        
        #i+=1
        
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
