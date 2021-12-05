"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
import math


# ------------------CHANGE CODE BELOW HERE ONLY--------------------------
# define functions here for making decisions and using sensor inputs

class robotPathFinder:
    "pathfinding manager for robot. avoids zombies and seeks berries depending on current robot state. \
    default state is to seek berries"

    def __init__(self, robot, state="sentry"):
        self.robot = robot
        self.st = state
        self.edible = {}  # set of all edible berries
        self.pZ = 0  # boolean representing if we have seen a purple zombie
        self.states = ["sentry", "turn&ID", "berryAction", "survive"]
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
        self.robot.turn(direction)  # turn to correct direction
        self.robot.forward(.5)  # move forward .5 meters
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
            procLidar = self.averageWindow(lidarOut, 5)
            direction = lidarOut.index(max(procLidar))
            if 'p' not in recOut:
                self.move(direction)
                break
            else:
                self.move(direction)
        return

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

    def berryAction(self, lidarOut, camOut, direction=None):
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
        if item == 'tree':  # if item is tree, then do tree action
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

    def pathfinder(self, lidar_output, reciver_output, camera_output):
        "top level controller for pathfinding. functions for output from sensors are passed as paramaters\
        each cycle of loop determines what state to be in. Actions are taken by takeAction function"
        while True:
            # go through each point in lidar output, determine appropriate action if item is within action threshold
            lidarOut = lidar_output()
            recOut = reciver_output()
            camOut = camera_output()

            procLidar = self.averageWindow(lidarOut)
            procLidar = map(lambda x: x * -1, procLidar)  # turn peaks into troughs
            procLidar = self.peaks(procLidar)

            if not recOut and not any(procLidar):
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


# ------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # health, energy, armour in that order
    robot_info = [100, 100, 0]
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

        # lidar output ---> array of distances of length 512 (our horizontal resolution)
        lid_out = lidar.getRangeImage()
        print(lid_out)

        # receiver output ---> ['a', 'p', 'g', 'b']
        import struct
        rec_out = []
        while (receiver.getQueueLength() > 0):
            message = receiver.getData()
            dataList = struct.unpack("chd", message)
            dir = receiver.getEmitterDirection()
            signal = receiver.getSignalStrength()
            # print(f"message is {message} and {dataList}, dir is {dir}, signal is {signal}")
            receiver.nextPacket()
            rec_out.append(dataList[0].decode('utf-8'))
        print(rec_out)

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
