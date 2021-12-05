"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

import numpy as np
import cv2 as cv

class robotPathFinder:
    "pathfinding manager for robot. avoids zombies and seeks berries depending on current robot state. \
    default state is to seek berries"
    def __init__(self, robot, state = "sentry"):
        self.robot = robot
        self.st = state
        self.edible = {} #set of all edible berries
        self.pZ = 0 # boolean representing if we have seen a purple zombie
        self.states = ["sentry", "turn&ID", "berryAction", "survive"]
        self.berries =  {
            # Set "points" for berries:
                # negative pts = bad result — do not try again
                # 0 pt = untried
                # 1 pt = add health
                # 2 pt = add energy
                # 3 pt = armor
                            "red": {},
                            "yellow": {},
                            "orange": {},
                            "pink": {}
                        }


    def range4_10(self, lst):
        for val in lst:
            if 4 <= val <= 10:
                return True
        return False

    def range0_4(self, lst):
        for val in lst:
            if 0 <= val <= 4:
                return True
        return False

    def move(self, direction):
        self.robot.turn(direction) # turn to correct direction
        self.robot.forward(.5) # move forward .5 meters
        return

    def takeAction(self, lidarOut, recOut, camOut):
        if self.st == "sentry":
            self.sentry(lidarOut, camOut, recOut)
        elif self.st == "turn&ID":
            self.turnID(lidarOut, camOut, recOut)
        elif self.st == "berryAction":
            self.berryAction(lidarOut, camOut)
        elif self.st == "survive":
            self.survive(lidarOut, camOut, recOut)
        return

    def sentry(self, lidarOut, camOut, recOut):
        "do nothing. If low health or low energy, turn&ID"
        if self.robot.health <= 20 or self.robot.energy <= 20:
            self.turnID(lidarOut, camOut)
        return

    def turnID(self, lidarOut, camOut, recOut):
        pass

    def survive(self, lidarOut, camOut, recOut):
        "survival mode. determine free space and run away. if abg zombie, go back to start and potentially sentry. \
        if purple, stay in survival"
        while true:
            # best direction is that of most avail space, naive implementation, may need to change later
            direction = lidarOut.index(max(lidarOut))
            if 'p' not in recOut:
                self.move(direction)
                break
            else:
                self.move(direction)
        return

    def identify(self, direction, camOut):
        "determines if item is zombie, tree, or berry. If zombie, or berry, also returns what color. \
        second term if empty string if is tree"
        self.robot.turn((180+direction)%360) # turn so back of robot faces the direction
        color = camOut[len(camOut)//2] # find the color of what is at the middle of the array returned by camera
        if color == 'b':
            return 'tree', ""
        elif color in list("abgp"):
            return 'zomb', color
        else:
            return 'berry', color

    def berryAction(self, lidarOut, camOut, direction = None):
        "makes choices about whether to eat or not eats berries"
        # If direction is None, then that means this was directly called by pathfinder
        if direction is None:
            direction = lidarOut.index(min(lidarOut))
            self.move(direction)
            return

        # if purple zombie has been seen, then don't eat berries that aren't already within 4m safe range and don't approach trees
        if self.pZ:
            return

        item, color = self.identify(direction, lidarOut)
        if item == 'tree': #if item is tree, then do tree action
            self.treeAction(direction, lidarOut)
            return

        # if berry is edible, eat it by moving towards it
        if color in self.edible:
            self.move(direction)
        return

    def tryBerry(self, berrycolor):
        "tries berry and returns appropriate # of pts to berries dict"
        if len(self.berries[berrycolor]) < 2:
            old_stats = [robot_info[0], robot_info[1], robot_info[2]]
            buffer = 5
            # try berry
            if robot_info[1] + buffer < old_stats[1]:
                self.berries[berrycolor].add(-1)
            if robot_info[0] > old_stats[0] + buffer:
                self.berries[berrycolor].add(1)
            if robot_info[1] > old_stats[1] + buffer:
                self.berries[berrycolor].add(2)
            if robot_info[2] > old_stats[2] + buffer:
                self.berries[berrycolor].add(3)
        return

    def treeAction(self, direction, lidarOut):
        "make choice about whether to approach or avoid tree"
        armReach = 0.1
        if lidarOut[direction] > armReach:
            self.move(direction)
        else:
            self.robot.arm.move()
        return


    def pathfinder(self, lidar_output, reciver_output, camera_output):
        "top level controller for pathfinding. functions for output from sensors are passed as paramaters\
        each cycle of loop determines what state to be in. Actions are taken by takeAction function"
        while True:
            # go through each point in lidar output, determine appropriate action if item is within action threshold
            lidarOut = lidar_output()
            recOut = reciver_output()
            camOut = camera_output()

            if not recOut and not lidarOut:
                self.st = "sentry"
            elif not recOut and self.range4_10(lidarOut):
               self.st = "turn&ID"
            elif not recOut and self.range0_4(lidarOut):
                self.st = "berryAction"
            elif rec:
                self.st = "survive"
            else:
                self.st = "sentry"
            self.takeAction(lidarOut, recOut, camOut)



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
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))
    
    
    i=0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
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
            # get the predominant color of the central 3x3 matrix
            # # print(len(pic), len(pic[0]))
            # avg = np.array([0, 0, 0])
            # # color_tally = 0
            # for row in range((len(pic) // 2 - 1), (len(pic) // 2) + 1):
            #     for col in range((len(pic[0])//2) - 1, (len(pic[0])//2) + 1):
            #         avg[0] += pic[row][col][0]
            #         avg[1] += pic[row][col][1]
            #         avg[2] += pic[row][col][2]
            #         # color_tally += 1
            # # print(color_tally)

            # avg = np.divide(avg, 4)
            
            # for color in ranges:
            #     if all(np.less(ranges[color][0], avg)) and all(np.less(avg, ranges[color][1])):
            #         print(color)

            for row in range(0, 64):
                for col in range(63, 65):
                    pixel = pic[row][col]
                    for color in ranges:
                        if all(np.less(ranges[color][0], pixel)) and all(np.less(pixel, ranges[color][1])):
                            print(color)


            # for col_key in ranges:
            #     pic_copy = hsv
            #     curr = cv.inRange(pic_copy, ranges[col_key][0], ranges[col_key][1])
            #     # print(curr)
            #     if col_key not in result:
            #         result[col_key] = []
                
            #     #TODO: check output format of cv.inRange
            #         #print("size is ", curr.size)
                    
            #         print(curr.size, curr.size)
            #         for r in range (curr[0].size):
            #             for c in range (curr[1].size):
            #                 if curr[r][c] == 255:
            #                     result[col_key].append((r, c))
            #     print(result)
        
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
