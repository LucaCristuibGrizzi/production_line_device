#!/usr/bin/env python
import sys, threading
import rospy, rospkg
from std_msgs.msg import String, Bool, Int16
from pyniryo import *

# Append Custom Path to Execution
package_path = rospkg.RosPack().get_path('production_line_device')
sys.path.append(f'{package_path}/scripts/')

# Custom Imports
from Utils.Pose import Ned2PosesR2
import Utils.Niryo as Niryo
from production_line_device.msg import PickAndPlace, WrongAssemblySolution

#--------- GLOBAL VARIABLE ---------#
# Publishers needed for communicating with the production line node.
pub = rospy.Publisher('r2_operation_status', String, queue_size=10)

# Publisher needed for communicating with the FlexBE behavior "Production Line".
pubWA = rospy.Publisher('rs_wrong_assembly', Bool, queue_size=5)

# Global variables
nDevice = 3

# Variables used to generate errors at the specified time
# during the node launch.
slowdownError = False
nSlowdownError = 0
counterSlowdownError = 0
componentError = False
nComponentError = 0
counterComponentError = 0

# Lock variable for all the global variables that are shared across multiple threads
# and might be accessed at the same time.
slowdownErrorLock = threading.Lock()
componentErrorLock = threading.Lock()

# Lock variable to avoid possible errors arising from the request for the execution
# of multiple commands simultaneously (The pyNiryo library doesn't allow this, so it must be avoided).
r2OperationLock = threading.Lock()

#--------- FUNCTION ---------#
# Functions for getting and setting variables shared across multiple threads.
def getCounterSlowdownError():
    with slowdownErrorLock:
        return counterSlowdownError
    
def addCounterSlowdownError():
    with slowdownErrorLock:
        global counterSlowdownError
        counterSlowdownError += 1

def getCounterComponentError():
    with componentErrorLock:
        return counterComponentError
    
def addCounterComponentError():
    with componentErrorLock:
        global counterComponentError
        counterComponentError += 1

# Function for controlling the pick-and-place operations of the robot.
def pickAndPlace(data):

    # Generate specified errors during node launch for the robot.
    if (slowdownError and nSlowdownError == getCounterSlowdownError()):
        addCounterSlowdownError()
        speed = 50
        pub.publish(str(speed))
        return
    elif (componentError and nComponentError == getCounterComponentError()):
        addCounterComponentError()
        pub.publish("OK")
        return

    addCounterSlowdownError()
    addCounterComponentError()

    # Ensure exclusive access when executing this operation; no other operations
    # related to the robot should be executed
    with r2OperationLock:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pickPoses)

        # Extract the number of pick-and-place operations and their
        # respective poses from the received message.
        n_operations = data.nOperations
        pickPoses = data.pickPoses
        placePoses = data.placePoses

        # Get the joint pose for the Home Position.
        jointHome = Niryo.selectPose("Home_Pose", Ned2PosesR2)

        # Execute the pick-and-place operations.
        for i in range(n_operations):
            pickPoseName = pickPoses[i]
            placePoseName = placePoses[i]

            pickPose = Niryo.selectPose(pickPoseName, Ned2PosesR2)
            placePose = Niryo.selectPose(placePoseName, Ned2PosesR2)

            # Forward kinematics is necessary because the pick_and_place function uses
            # end-effector positions, while selectPose returns joint poses.
            pickPosition = robot.forward_kinematics(pickPose)
            placePosition = robot.forward_kinematics(placePose)

            robot.pick_and_place(pickPosition, placePosition)
            robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)

        # Return the robot to the home pose and publish a message when the is near that pose.
        robot.move_joints(jointHome)
        if Niryo.checkMove(robot, jointHome, 0.05):
            pub.publish("OK")

# Function to set the new speed for the robot controlled by this node.
def setSpeed(data):
    # Ensure exclusive access when setting the speed.
    with r2OperationLock:
        robot.set_arm_max_velocity(data.data)

# Function that controls the operation of managing the error of a component not arriving.
# Note: The operation can be personalized for each robot based on the characteristics
# of the production line in that area.
def wrongAssembly(data):
    # Check if the message received is for this device.
    if data.deviceNumber == nDevice:
        # The robot can be an "assistant" (or "helper") or an "operative" robot in the operation.
        # This determines the first difference in the operation to execute.

        # Assistant
        if data.assistantRobot:
            # The assistant robot must hold the product during the entire operation of the
            # operative robot.
            # So, the operation is divided into two parts: before the holding task and after it (return Home).
            if data.goHome:
                # Ensure exclusive access when executing this operation.
                with r2OperationLock:
                    # After correcting the element present inside the case to end the handling of
                    # the wrong assembly error, the assistant robot must pick the product from the center
                    # of the rotating table and place it in its standard position to resume the production
                    # line work.

                    # Get all the position.
                    helpPoseName = data.helpPose
                    casePoseName = helpPoseName.replace("Help_", "")
                    helpPoseName = helpPoseName[:-4]
                    jointHome = Niryo.selectPose("Home_Pose", Ned2PosesR2)                
                    helpPose= Niryo.selectPose(helpPoseName, Ned2PosesR2)                  
                    casePose = Niryo.selectPose(casePoseName, Ned2PosesR2)
                    centerRotatingTablePose = Niryo.selectPose("Pose_Center_RT", Ned2PosesR2)
                    centerRotatingTablePosition = robot.forward_kinematics(centerRotatingTablePose)
                    casePosition = robot.forward_kinematics(casePose)
                    helpPosition = robot.forward_kinematics(helpPose)
                    casePosition = casePosition.copy_with_offsets(yaw_offset=-3.14)
                    afterHelpPosition = helpPosition.copy_with_offsets(z_offset=0.12)

                    # Execute the operations.
                    robot.open_gripper(max_torque_percentage=50, hold_torque_percentage=50)
                    robot.move_pose(afterHelpPosition)
                    robot.pick_and_place(centerRotatingTablePosition, casePosition)
                    robot.move_joints(jointHome)
                    robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)

                    # Return the robot to the home pose and publish a message when the is near that pose.
                    if Niryo.checkMove(robot, jointHome, 0.05):
                        pubWA.publish(True)
            else:
                # Ensure exclusive access when executing this operation
                with r2OperationLock:
                    # The robot picks the product from its position and places it in the middle of the rotating table.
                    # This position is more comfortable for Robot3 to remove the elements from the case.
                    # After this task, the robot holds the product with the end-effector oriented horizontally.

                    # Get all the position.
                    helpPoseName = data.helpPose
                    casePoseName = helpPoseName.replace("Help_", "")
                    helpPoseName = helpPoseName[:-4]
                    helpPose= Niryo.selectPose(helpPoseName, Ned2PosesR2)
                    casePose = Niryo.selectPose(casePoseName, Ned2PosesR2)
                    centerRotatingTablePose = Niryo.selectPose("Pose_Center_RT", Ned2PosesR2)
                    casePosition = robot.forward_kinematics(casePose)
                    helpPosition = robot.forward_kinematics(helpPose)
                    centerRotatingTablePosition = robot.forward_kinematics(centerRotatingTablePose)
                    casePosition = casePosition.copy_with_offsets(yaw_offset=-3.14)
                    beforeHelpPosition = helpPosition.copy_with_offsets(z_offset=0.12)

                    # Execute the operations.
                    robot.pick_and_place(casePosition, centerRotatingTablePosition)
                    robot.move_pose(beforeHelpPosition)
                    robot.open_gripper(max_torque_percentage=50, hold_torque_percentage=50)
                    robot.move_pose(helpPosition)
                    robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=10)

                    # Publish a message when the robot is near the helpPose.
                    if Niryo.checkMove(robot, helpPose, 0.05):
                        pubWA.publish(True)
        # Operative
        else:
            # Ensure exclusive access when executing this operation
            with r2OperationLock:
                rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pickPoses)

                # Note: It is implemented the same code as the pickAndPlace function, but
                # in this production line, Robot2 never gets the role of "operative".
                # So this code needs to be changed in future production lines.
                n_operations = data.nOperations
                pickPoses = data.pickPoses
                placePoses = data.placePoses

                jointHome = Niryo.selectPose("Home_Pose", Ned2PosesR2)

                for i in range(n_operations):
                    pickPoseName = pickPoses[i]
                    placePoseName = placePoses[i]

                    pickPose = Niryo.selectPose(pickPoseName, Ned2PosesR2)
                    placePose = Niryo.selectPose(placePoseName, Ned2PosesR2)

                    pickPosition = robot.forward_kinematics(pickPose)
                    placePosition = robot.forward_kinematics(placePose)
                    robot.pick_and_place(pickPosition, placePosition)
                    robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)

                # Return the robot to the home pose and publish a message when the is near that pose.
                robot.move_joints(jointHome)
                if Niryo.checkMove(robot, jointHome, 0.05):
                    pubWA.publish(True)

# Function to close correctly this node.
def shutdownOperations():
    robot.close_connection()

# Function that defines the ROS node and sets up subscribers for communication with
# the production line node and the FlexBE behavior "Production Line".    
def listener():
    rospy.init_node('niryo_R2', anonymous=True)
    rospy.on_shutdown(shutdownOperations)
    rospy.Subscriber('cmd_r2', PickAndPlace, pickAndPlace)
    rospy.Subscriber('slowdown_error', Int16, setSpeed)
    rospy.Subscriber('wrong_assembly', WrongAssemblySolution, wrongAssembly)
    rospy.spin()

if __name__ == "__main__":

    # When launching the node, arguments can be used to define the type
    # of error to generate and when to generate it.
    # s -> slowdown error
    # c -> component not arrived error
    for av in sys.argv[1:]:
        errorType = av[:2]
        if errorType == "-s":
            try:
                nSlowdownError = int(av[2]) - 1
                slowdownError = True
            except:
                print("Error: no number in the argument")
        elif errorType == "-c":
            try:
                nComponentError = int(av[2]) - 1
                componentError = True
            except:
                print("Error: no number in the argument")

    # Connect to the robot and preliminary operations.
    robot = NiryoRobot("169.254.200.201")
    robot.calibrate_auto()
    robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)
    robot.set_arm_max_velocity(100)
    robot.move_to_home_pose()

    # Initialize the ROS node and start the subscribers.
    listener()