"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
import math


# ------------------CHANGE CODE BELOW HERE ONLY--------------------------
# define functions here for making decisions and using sensor inputs

class robotWeiFinder:
    "pathfinding manager for robot. avoids zombies and seeks berries depending on current robot state. \
    default state is to seek berries"

    def __init__(self, robot, state="sentry"):
        self.robot = robot
        self.health = 100
        self.energy = 100
        self.st = state
        self.punch = 0  # only punch one tree: prevent us from getting stuck trying to punch trees forever.
        self.edible = {}  # set of all edible berries
        self.pZ = 0  # boolean representing if we have seen a purple zombie
        self.states = ["sentry", "turn&ID", "berryAction", "treeAction", "survive"]
        self.berries = {
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
        "find things within lidar range of 4 to 10 meters"
        for val in lst:
            if 4 <= val <= 10:
                return True
        return False

    def range0_4(self, lst):
        "find things within lidar range or 0 to 4 meters"
        for val in lst:
            if 0 <= val <= 4:
                return True
        return False

    def move(self, direction):
        "move robot in direction for .5 meters"
        self.robot.turn(direction)  # turn to correct direction
        self.robot.forward(.5)  # move forward .5 meters
        return

    def takeAction(self, lidarOut, camOut, recOut, direction=None):
        "take action based on robot state"
        if self.st == "sentry":
            self.sentry(lidarOut, camOut, recOut)
        elif self.st == "turn&ID":
            self.turnID(lidarOut, camOut, recOut)
        elif self.st == "berryAction":
            self.berryAction(lidarOut, camOut, recOut, direction)
        elif self.st == "survive":
            self.survive(lidarOut, camOut, recOut)
        elif self.st == "treeAction":
            self.treeAction(lidarOut, camOut, recOut, direction)
        return

    def sentry(self, lidarOut, camOut, recOut):
        "do nothing. If low health or low energy, turn&ID"
        if self.health <= 35 or self.energy <= 35:
            self.st = "turnID"
            self.takeAction(lidarOut, camOut, recOut)
        return

    def turnID(self, lidarOut, camOut, recOut):
        "turn and identify peaks (technically troughs) in lidar data with camera"
        pass

    def survive(self, lidarOut, camOut, recOut):
        "survival mode. determine free space and run away. if abg zombie, go back to start and potentially sentry. \
        if purple, stay in survival"
        direction = lidarOut.index(max(lidarOut))
        self.move(direction)


    def identify(self, direction, camOut):
        "determines if item is zombie, tree, or berry. If zombie, or berry, also returns what color. \
        second term if empty string if is tree"
        self.robot.turn((180 + direction) % 360)  # turn so back of robot faces the direction
        color = camOut[len(camOut) // 2]  # find the color of what is at the middle of the array returned by camera
        if color == 'b':
            return 'tree', ""
        elif color in list("abgp"):
            return 'zomb', color
        else:
            return 'berry', color

    def berryAction(self, lidarOut, camOut, recOut, direction=None):
        "makes choices about whether to eat or not eats berries"
        # If direction is None, then that means this was directly called by weifinder
        if direction is None:
            direction = lidarOut.index(min(lidarOut))
            self.move(direction)
            return

        # if purple zombie has been seen, then don't eat berries that aren't already within 4m safe range and don't approach trees
        if self.pZ:
            return

        item, color = self.identify(direction, lidarOut)
        # if item is tree and have not punched tree before, then do tree action.
        # if already punched a tre before, don't punch
        if item == 'tree' and self.punch == 0:
            self.st = "treeAction"
            self.takeAction(lidarOut, camOut, recOut, direction)
            return
        elif item == 'tree':
            return

        # if berry is edible, eat it by moving towards it
        if item == berry and color in self.edible:
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

    def treeAction(self, lidarOut, camOut, recOut, direction = None):
        "make choice about whether to approach or punch tree. If punch, log punch"
        if direction is None:
            direction = lidarOut.index(min(lidarOut))
            self.move(direction)
            return

        armReach = 0.1
        if lidarOut[direction] > armReach:
            self.move(direction)
        else:
            self.punch = 1
            self.robot.arm.move()
        return

    def averageWindow(self, array, window=5):
        "returns list of average values of lst for windows of size"

        def averageCenterd(array, i, window):
            half = window // 2
            val = sum(array[i - half:i + half + 1])
            return val / float(window)

        array = array[-10:] + array[:] + array[0:10]
        ret = []
        for ind in range(10, len(array) - 9):
            ret.append(averageCenterd(array, ind, window))
        return ret

    def peaks(self, lst):
        def ispeak(i, lst):
            if i == 0:
                return lst[i] > lst[1] and lst[i] > lst[-1]
            elif i == len(lst) - 1:
                return lst[i] > lst[-1] and lst[i] > lst[0]
            else:
                return lst[i - 1] < lst[i] and lst[i + 1] < lst[i]

        ret = []
        for indx in range(len(lst)):
            ret.append(ispeak(indx, lst))
        return ret

    def compressLidar(self, lst):
        ret = [0] * 360
        for i in range(len(lst) - 1):
            ni = int(i / (512 / 360.0))
            if ret[ni] == 0:
                ret[ni] = lst[i]
            else:
                ret[ni] = (ret[ni] + lst[i]) / 2.0
        ret[-1] = (ret[0] + ret[-2]) / 0.2
        return ret


    def weifinder(self, robot_info, lidar_output, receiver_output, camera_output= lambda : []):
        "top level controller for pathfinding. functions for output from sensors are passed as paramaters\
        each cycle of loop determines what state to be in. Actions are taken by takeAction function"
        #update robot information
        self.health = robot_info[0]
        self.energy = robot_info[1]

        # go through each point in lidar output, determine appropriate action if item is within action threshold
        lidarOut = lidar_output()
        lidarOut = self.compressLidar(lidarOut)  # compress lidarOut into len 360 array
        recOut = receiver_output()
        camOut = camera_output()

        lidarOut = self.averageWindow(lidarOut) # convert lidar out to windowed lidar
        procLidar = list(map(lambda x: x * -1, lidarOut))  # turn peaks into troughs for detect transitions between vals in peak

        if not recOut and not any(self.peaks(procLidar)):
            self.st = "sentry"
        elif not recOut and self.range4_10(lidarOut):
            self.st = "turn&ID"
        elif not recOut and self.range0_4(lidarOut):
            self.st = "berryAction"
        elif recOut:
            self.st = "survive"
        else:
            self.st = "sentry"
        self.takeAction(lidarOut, camOut, recOut)


# ------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # health, energy, armour in that order
    robot_info = [100, 21, 0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0

    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")

    get_all_berry_pos(robot)

    robot_not_dead = 1

    # ------------------CHANGE CODE BELOW HERE ONLY--------------------------

    # COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
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

    i = 0

    # create a robotWeiFinder object to manage the robot and name it andrew due to him being a dank boi
    Andrew = robotWeiFinder(robot)

    # ------------------CHANGE CODE ABOVE HERE ONLY--------------------------

    while (robot_not_dead == 1):

        if (robot_info[0] < 0):
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            # if(zombieTest):
            #    print("TEST PASSED")
            # else:
            #    print("TEST FAILED")
            # robot.simulationQuit(20)
            # exit()

        if (timer % 2 == 0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)

        if (timer % 16 == 0):
            robot_info = update_robot(robot_info)
            timer = 0

        if (robot.step(timestep) == -1):
            exit()

        timer += 1

        # ------------------CHANGE CODE BELOW HERE ONLY--------------------------
        # the following is called every timestep

        def recFxn():
            "function to get reciever output"
            # receiver output ---> ['a', 'p', 'g', 'b']
            import struct
            rec_out = []
            while (receiver.getQueueLength() > 0):
                message = receiver.getData()
                dataList = struct.unpack("chd", message)
                # dir = receiver.getEmitterDirection()
                # signal = receiver.getSignalStrength()
                # print(f"message is {message} and {dataList}, dir is {dir}, signal is {signal}")
                receiver.nextPacket()
                rec_out.append(dataList[0].decode('utf-8'))
            return rec_out

        # lidar output ---> array of distances of length 512 (our horizontal resolution)
        Andrew.weifinder(robot_info, lidar.getRangeImage, recFxn)


        # possible pseudocode for moving forward, then doing a 90 degree left turn
        # if i <100
        # base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own

        # if == 100
        # base_reset()
        # base_turn_left()
        # it takes about 150 timesteps for the robot to complete the turn

        # if i==300
        # i = 0

        # i+=1

        # make decisions using inputs if you choose to do so

        # ------------------CHANGE CODE ABOVE HERE ONLY--------------------------

    return 0


main()
