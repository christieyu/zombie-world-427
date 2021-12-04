"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
import math
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

class robotPathfinder:
    "pathfinding manager for robot. avoids zombies and seeks berries depending on current robot state. \
    default state is to seek berries"
    def __init__(self, state = "seek", threshold = 8):
        self.st = state
        self.states = ["avoid","seek","survive"]
        self.threshold = threshold
        self.berrydict = {}
        self.dangerZ = set("p","o")

    def distance(self,point):
        "finds distance from a lidar point to (0,0)"
        x, y, z = point
        return math.sqrt((x**2)+(y**2)+(z**2))

    def identify(self, item):
        "determines if item is zombie, tree, or berry. If zombie, or berry, also returns what color. \
        second term if empty string if is tree"
        return ("z","g")

    def escape(self):
        "Makes robot drive away from threat at normal speed if not dangerous zombie, at max speed \
        if super dangerous zombie"
        # implement code to drive robot in the most open direction for 5 seconds
        if self.st == "avoid":
            #run away at normal speed
        if self.st == "survive":
            #run away really really really fast
        pass

    def approach(self, item):
        "robot drives in direction of item"
        pass

    def eatBerries(self, berrycolor):
        "makes choices about whether to eat or not eats berries"
        # implement code to eat berries
        pass

    def treeAction(self):
        "make choice about whether to approach or avoid tree"
        pass

    def pathfind(self, lidar_output):
        "top level controller for pathfinding. function for getting lidar output is passed as param"

        while true:
            # go through each point in lidar output, determine appropriate action if item is within action threshold
            output = lidar_output()
            if not output:
                self.st = "seek" # randomly wander until we find an energy berry, and then enter eat berry state

            for point in :
                if self.distance(point) <= self.threshold:
                    objectID, color = self.identify(point)

                    # if zombie, set state to avoid or survive based on danger level, call escape() function
                    if objectID == "z":
                        if color in self.dangerZ:
                            self.st = "avoid"
                        else:
                            self.st = "survive"
                        self.escape()

                    # if berry and robot is not in avoid or survive states, call eatBerry function
                    elif objectID == "b":
                        if self.state == "seek":
                            self.eatBerries(color)
                        pass

                    # if tree and robot is not in avoid or survive states, call treAction function
                    elif objectID == "t":
                        if self.state == "seek"
                            self.treeAction()
                        pass

            #may need to add a time delay here before entering next iteration of loop




    


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
    
    # camera8 = robot.getDevice("BackHighRes")
    # camera8.enable(timestep)
    
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
