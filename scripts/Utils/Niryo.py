from pyniryo import *
from Utils.Pose import *

# This is a module that cotains the functions that are in commom between 
# the three robots of the production line.

# Function that allow to retrive the joint pose value from the pose name.
# Each robot has a specific pose enumeration that define all the poses that
# it can reach. This enumeration must be passed to the function.
def selectPose(poseName, enumPoses: Enum):
    for pose in (enumPoses):
        if pose.name == poseName:
            return pose.value

# Function that check if the specified robot is near with a specified delta error
# to a certain pose.        
def checkMove(robot : NiryoRobot, jointPose, delta):
    arrived = False
    jointsOK = [False, False, False, False, False, False]
    
    # Block the execution until the robot is close to the specified pose.
    while not arrived:
        joints = robot.get_joints()
        for i in range(len(joints)):
            if joints[i] < jointPose[i]+delta and joints[i] > jointPose[i]-delta:
                jointsOK[i] = True
        
        if jointsOK.count(True) == 6:
            arrived = True
            print("Arrived")
    
    return True