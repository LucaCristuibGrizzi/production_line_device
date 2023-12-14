#!/usr/bin/env python
import sys, threading
import rospy, rospkg
from std_msgs.msg import String, Bool, Int16
from pyniryo import *

# Append Custom Path to Execution
package_path = rospkg.RosPack().get_path('production_line_device')
sys.path.append(f'{package_path}/scripts/')

# Custom Imports
from Utils.Pose import Ned2PosesR3
import Utils.Niryo as Niryo
from production_line_device.msg import PickAndPlace, WrongAssemblySolution

#--------- GLOBAL VARIABLE ---------#
# Publishers needed for communicating with the production line node.
pub = rospy.Publisher('r3_operation_status', String, queue_size=10)

# Publisher needed for communicating with the FlexBE behavior "Production Line".
pubCNA = rospy.Publisher('rs_component_not_arrived', Bool, queue_size=10)
pubWA = rospy.Publisher('rs_wrong_assembly', Bool, queue_size=5)

# Global variables
nDevice = 5

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
r3OperationLock = threading.Lock()

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
        speed = 100
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
    with r3OperationLock:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pickPoses)

        # Extract the number of pick-and-place operations and their
        # respective poses from the received message.
        n_operations = data.nOperations
        pickPoses = data.pickPoses
        placePoses = data.placePoses

        # Get the joint pose for the Home Position.
        jointHome = Niryo.selectPose("Home_Pose", Ned2PosesR3)

        # Execute the pick-and-place operations.
        for i in range(n_operations):
            pickPoseName = pickPoses[i]
            placePoseName = placePoses[i]

            pickPose = Niryo.selectPose(pickPoseName, Ned2PosesR3)
            placePose = Niryo.selectPose(placePoseName, Ned2PosesR3)

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

# Function that controls the operation of managing the error of component not arrived.
# Note: The operation can be personalized for each robot based on the characteristics
# of the production line in that area.
def moveComponent(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    list = data.data.split(";")

    # Ensure exclusive access when executing this operation.
    with r3OperationLock:
        # Check if the message received is for this device.
        if int(list[0]) == nDevice:

            # It is implemented the same code as the pickAndPlace function, but the place position is
            # rotated by 360° around the vertical axis.  
            pickPoseName = list[1]
            placePoseName = list[2]

            jointHome = Niryo.selectPose("Home_Pose", Ned2PosesR3)
            pickPose = Niryo.selectPose(pickPoseName, Ned2PosesR3)
            placePose = Niryo.selectPose(placePoseName, Ned2PosesR3)

            pickPosition = robot.forward_kinematics(pickPose)
            placePosition = robot.forward_kinematics(placePose)
            placePosition = placePosition.copy_with_offsets(yaw_offset=3.14)

            robot.pick_and_place(pickPosition, placePosition)
            robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)

        robot.move_joints(jointHome)
        if Niryo.checkMove(robot, jointHome, 0.05):
            pubCNA.publish(True)

# Function that controls the operation of managing the error of a component not arriving.
# Note: The operation can be personalized for each robot based on the characteristics
# of the production line in that area.
def wrongAssembly(data):
     # Check if the message received is for this device.
    if data.deviceNumber == nDevice:
        # The robot can be an "assistant" (or "helper") or an "operative" robot in the operation.
        # This determines the first difference in the operation to execute.

        # Assistant
        # Note: In the tested error handling operation, Robot3 never assumes the role of "assistant".
        # Therefore, the subsequent code is similar to what is implemented for Robot2, but it is not
        # designed for a specific recovery operation. If needed, implement an appropriate operation. 
        if data.assistantRobot:
            if data.goHome:
                with r3OperationLock:
                    jointHome = Niryo.selectPose("Home_Pose", Ned2PosesR3)
                    helpPoseName = data.helpPose
                    helpPose= Niryo.selectPose(helpPoseName, Ned2PosesR3)
                    casePoseName = helpPoseName.replace("Help_", "")
                    casePose = Niryo.selectPose(casePoseName, Ned2PosesR3)
                    casePosition = robot.forward_kinematics(casePose)
                    helpPosition = robot.forward_kinematics(helpPose)
                    rotateCasePosition = casePosition.copy_with_offsets(yaw_offset=1.507)
                    afterHelpPosition = helpPosition.copy_with_offsets(z_offset=0.05)
                    robot.open_gripper(max_torque_percentage=50, hold_torque_percentage=50)
                    robot.move_pose(afterHelpPosition)
                    robot.pick_and_place(rotateCasePosition, casePosition)
                    robot.move_joints(jointHome)
                    robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)

                    if Niryo.checkMove(robot, jointHome, 0.05):
                        pubWA.publish(True)
            else:
                helpPoseName = data.helpPose
                helpPose= Niryo.selectPose(helpPoseName, Ned2PosesR3)
                casePoseName = helpPoseName.replace("Help_", "")
                casePose = Niryo.selectPose(casePoseName, Ned2PosesR3)
                casePosition = robot.forward_kinematics(casePose)
                helpPosition = robot.forward_kinematics(helpPose)
                rotateCasePosition = casePosition.copy_with_offsets(yaw_offset=1.507)
                beforeHelpPosition = helpPosition.copy_with_offsets(z_offset=0.05)#x_offset=-0.05)
                robot.pick_and_place(casePosition, rotateCasePosition)
                robot.move_pose(beforeHelpPosition)
                robot.open_gripper(max_torque_percentage=50, hold_torque_percentage=50)
                robot.move_joints(helpPose)
                robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=1)

                if Niryo.checkMove(robot, helpPose, 0.05):
                    pubWA.publish(True)
        # Operative
        else:
            # Ensure exclusive access when executing this operation.
            with r3OperationLock:
                rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pickPoses)

                # Extract the number of pick-and-place operations and their
                # respective poses from the received message.
                n_operations = data.nOperations
                pickPoses = data.pickPoses
                placePoses = data.placePoses

                # Get the joint pose for the Home Position.
                jointHome = Niryo.selectPose("Home_Pose", Ned2PosesR3)

                # Execute the pick-and-place operations.
                for i in range(n_operations):
                    pickPoseName = pickPoses[i]
                    placePoseName = placePoses[i]

                    # Note: The name of the pose for the element to remove differs between the one received
                    # and the one that the selectPose function can find. This is because the algorithm to
                    # manage the wrong assembly error is more generic than the code controlling the robot node.
                    # The received names have the suffix "_3_4" that needs to be removed.
                    if ("Pose_E1" in pickPoseName or
                        "Pose_E2" in pickPoseName or
                        "Pose_E3" in pickPoseName):
                        pickPoseName = pickPoseName[:-4]
                        pickPose = Niryo.selectPose(pickPoseName, Ned2PosesR3)
                    else:
                        pickPose = Niryo.selectPose(pickPoseName, Ned2PosesR3)

                    # Similar reasoning also applies to the name of placePose.
                    if "Pose_E_Top" in placePoseName:
                        placePose = Niryo.selectPose("Pose_E_Top_Center_RT", Ned2PosesR3)
                    else:
                        placePose = Niryo.selectPose(placePoseName, Ned2PosesR3)

                    # Forward kinematics is necessary because the pick_and_place function uses
                    # end-effector positions, while selectPose returns joint poses.
                    pickPosition = robot.forward_kinematics(pickPose)
                    placePosition = robot.forward_kinematics(placePose)

                    # For placing in warehouse_2 the removed element, Robot3 must change the way it
                    # holds the element. An exchange position is used for this change, and other than
                    # that, the operations in both cases (removing or inserting an element) are simple pick-and-place.
                    if "W2" in placePoseName:
                        exchangePose = Niryo.selectPose("Pose_Exchange", Ned2PosesR3)
                        verticalExchangePose = Niryo.selectPose("Pose_V_Exchange", Ned2PosesR3)
                        overPickPose = Niryo.selectPose("Pose_Over_Center_RT", Ned2PosesR3)
                        overPickPosition = robot.forward_kinematics(overPickPose)
                        exchangePosition = robot.forward_kinematics(exchangePose)
                        verticalExchangePosition = robot.forward_kinematics(verticalExchangePose)
                        robot.pick_from_pose(pickPosition)
                        robot.move_pose(overPickPosition)
                        robot.place_from_pose(exchangePosition)
                        robot.pick_and_place(verticalExchangePosition, placePosition)
                    else:
                        pickPosition = robot.forward_kinematics(pickPose)
                        placePosition = robot.forward_kinematics(placePose)
                        robot.pick_and_place(pickPosition, placePosition)
                        robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)

                # Return the robot to the home pose and publish a message when the is near that pose.
                robot.move_joints(jointHome)
                if Niryo.checkMove(robot, jointHome, 0.05):
                    pubWA.publish(True)

# Function to set the new speed for the robot controlled by this node.
def setSpeed(data):
    # Ensure exclusive access when setting the speed.
    with r3OperationLock:
        robot.set_arm_max_velocity(data.data)
        print("set speed")

# Function to close correctly this node.
def shutdownOperations():
    robot.close_connection()

# Function that defines the ROS node and sets up subscribers for communication with
# the production line node and the FlexBE behavior "Production Line".   
def listener():
    rospy.init_node('niryo_R3', anonymous=True)
    rospy.on_shutdown(shutdownOperations)
    rospy.Subscriber('cmd_r3', PickAndPlace, pickAndPlace)
    rospy.Subscriber('slowdown_error', Int16, setSpeed)
    rospy.Subscriber('component_not_arrived', String, moveComponent)
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
    robot = NiryoRobot("169.254.200.202")
    robot.calibrate_auto()
    robot.close_gripper(max_torque_percentage=50, hold_torque_percentage=50)
    robot.set_arm_max_velocity(100)
    robot.move_to_home_pose()

    # Initialize the ROS node and start the subscribers.
    listener()