#!/usr/bin/env python
import sys
import rospy, rospkg
from std_msgs.msg import String, Bool, Int16
from pyniryo import *
import threading

# Append Custom Path to Execution
package_path = rospkg.RosPack().get_path('production_line_device')
sys.path.append(f'{package_path}/scripts/')

# Custom Imports
from Utils.Pose import NedPosesR1
import Utils.Niryo as Niryo
from production_line_device.msg import PickAndPlace

# Note: Robot1 controls both the operation of the conveyor belt and
# the Infrared Sensor.

#--------- GLOBAL VARIABLE ---------#
# Publishers needed for communicating with the production line node.
pub = rospy.Publisher('r1_operation_status', String, queue_size=10)
pubIR = rospy.Publisher('check_IR', Bool, queue_size=1)
pubConveyor = rospy.Publisher('conveyor_operation_status', String, queue_size=10)

# Publisher needed for communicating with the FlexBE behavior "Production Line".
pubCNA = rospy.Publisher('rs_component_not_arrived', Bool, queue_size=10)

# Global variables
sensor_pin_id = PinID.GPIO_1A
deviceN = 1
conveyorSpeed = 100

# Variables used to generate errors at the specified time
# during the node launch.
deviceType = None
slowdownError = False
nSlowdownError = 0
counterSlowdownError = 0
componentError = False
nComponentError = 0
counterComponentError = 0

# Threading event to synchronize and control the conveyor belt operations.
conveyorEvent = threading.Event()

# Lock variable for all the global variables that are shared across multiple threads
# and might be accessed at the same time.
conveyorLock = threading.Lock()
slowdownErrorLock = threading.Lock()
componentErrorLock = threading.Lock()

# Lock variable to avoid possible errors arising from the request for the execution
# of multiple commands simultaneously (The pyNiryo library doesn't allow this, so it must be avoided).
r1OperationLock = threading.Lock()

#--------- FUNCTION ---------#
# Functions for getting and setting variables shared across multiple threads.
def getSpeedConveyor():
    with conveyorLock:
        return conveyorSpeed
    
def setSpeedConveyor(speed):
    with conveyorLock:
        global conveyorSpeed
        conveyorSpeed = speed

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
    if (deviceType == "R" and slowdownError and
        nSlowdownError == getCounterSlowdownError()):
        addCounterSlowdownError()
        speed = 30
        pub.publish(str(speed))
        return
    elif (deviceType == "R" and componentError 
          and nComponentError == getCounterComponentError()):
        addCounterComponentError()
        pub.publish("OK")
        return

    if deviceType == "R":
        addCounterSlowdownError()
        addCounterComponentError()

    # Ensure exclusive access when executing this operation; no other operations
    # related to the robot, conveyor belt, or IR sensor should be executed 
    with r1OperationLock:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pickPoses)

        # Extract the number of pick-and-place operations and their
        # respective poses from the received message.
        n_operations = data.nOperations
        pickPoses = data.pickPoses
        placePoses = data.placePoses

        # Get the joint pose for the Home Position.
        jointHome = Niryo.selectPose("Home_Pose", NedPosesR1)

        # Execute the pick-and-place operations.
        for i in range(n_operations):
            pickPoseName = pickPoses[i]
            placePoseName = placePoses[i]

            pickPose = Niryo.selectPose(pickPoseName, NedPosesR1)
            placePose = Niryo.selectPose(placePoseName, NedPosesR1)

            # Forward kinematics is necessary because the pick_and_place function uses
            # end-effector positions, while selectPose returns joint poses.
            pickPosition = robot.forward_kinematics(pickPose)
            placePosition = robot.forward_kinematics(placePose)

            robot.pick_and_place(pickPosition, placePosition, 0.001)
            robot.close_gripper(speed=300)
            
        # Return the robot to the home pose and publish a message when the is near that pose.
        robot.move_joints(jointHome)
        if Niryo.checkMove(robot, jointHome, 0.05):
            pub.publish("OK")

# Function to stop the conveyor belt.
# Note: The procedure to stop the conveyor belt differs when an
# error occurs and when the stop is part of the standard procedure.
def conveyorStop(data):
    if data.data == "ERROR" or data.data == "FAILED":
        conveyorEvent.set()
    elif data.data == "STOP":
        # Ensure exclusive access when executing this operation; no other operations
        # related to the robot, conveyor belt, or IR sensor should be executed 
        with r1OperationLock:
            robot.stop_conveyor(conveyor_id)
            pubConveyor.publish("OK")

# Function to run the conveyor belt
def conveyorRun(data):

    # Generate specified errors during node launch for the robot.
    if (deviceType == "C" and slowdownError and
        nSlowdownError == getCounterSlowdownError()):
        addCounterSlowdownError()
        speed = 30
        setSpeedConveyor(speed)
        pubConveyor.publish(str(speed))
        return
    elif (deviceType == "C" and componentError 
          and nComponentError == getCounterComponentError()):
        addCounterComponentError()
        pubConveyor.publish("OK")
        return
    
    if deviceType == "C":
        addCounterSlowdownError()
        addCounterComponentError()

    # Ensure exclusive access when executing this operation; no other operations
    # related to the robot, conveyor belt, or IR sensor should be executed 
    with r1OperationLock:
        # Reset the value of the variable indicating conveyor state.
        conveyorStopped = False
        # Get the speed for the conveyor belt
        speed = getSpeedConveyor()

        global conveyor_id

        # Attempt to start moving the conveyor belt. Retry if connection issues occur.
        # Note: Sometimes the robot has a problem detecting that the conveyor belt is connected.
        # This is why multiple attempts are needed before launching an error.
        maxAttempt = 5
        for attempt in range(maxAttempt):
            try:
                # Start moving the conveyor belt.
                robot.run_conveyor(conveyor_id, speed = speed, direction = ConveyorDirection.FORWARD)
                break
            except Exception as e:
                print(f"Attempt {attempt}: An error occurred - {e}")
                conveyor_id = robot.set_conveyor()
        else:
            raise RuntimeError("Failed after 5 attempts!")

        # After the conveyor belt starts moving, publish the message.
        pubConveyor.publish("OK")

        # Block the code until the IR Sensor doesn't detect the product.
        while robot.digital_read(PinID.GPIO_1A) == PinState.HIGH:
            # Procedure needed to stop the conveyor belt when an error is detected.
            if conveyorEvent.is_set():
                robot.stop_conveyor(conveyor_id)
                conveyorEvent.clear()
                conveyorStopped = True
                break
        
        # Publish that the IR sensor has detected the product.
        if not conveyorStopped:
            pubIR.publish(True)

# Function to set the new speed for the devices controlled by this node.
def setSpeed(data):
    # Ensure exclusive access when setting the speed.
    with r1OperationLock:
        robot.set_arm_max_velocity(data.data)
        setSpeedConveyor(data.data)

# Function that controls the operation of managing the error of component not arrived.
def moveComponent(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    list = data.data.split(";")

    if int(list[0]) == deviceN:
      # TODO: implement the necessary operation for
      #       the component not arrived recovery.
      pass

# Function to close correctly this node.
def shutdownOperations():
    robot.unset_conveyor(conveyor_id)
    robot.close_connection()

# Function that defines the ROS node and sets up subscribers for communication with
# the production line node and the FlexBE behavior "Production Line".
def listener():
    rospy.init_node('niryo_R1', anonymous=True)
    rospy.on_shutdown(shutdownOperations)
    rospy.Subscriber("cmd_r1", PickAndPlace, pickAndPlace)
    rospy.Subscriber("cmd_run_conveyor", String, conveyorRun)
    rospy.Subscriber("cmd_stop_conveyor", String, conveyorStop)
    rospy.Subscriber("slowdown_error", Int16, setSpeed)
    rospy.Subscriber("component_not_arrived", String, moveComponent)
    rospy.spin()    

if __name__ == "__main__":

    # When launching the node, arguments can be used to define the type
    # of error to generate and when to generate it.
    # r -> robot
    # c -> conveyor belt
    # s -> slowdown error
    # c -> component not arrived error
    for av in sys.argv[1:]:
        errorType = av[:3]
        if errorType == "-rs":
            try:
                nSlowdownError = int(av[3]) - 1
                slowdownError = True
                deviceType = "R"
            except:
                print("Error: no number in the argument")
        elif errorType == "-cs":
            try:
                nSlowdownError = int(av[3]) - 1
                slowdownError = True
                deviceType = "C"
            except:
                print("Error: no number in the argument")
        elif errorType == "-rc":
            try:
                nComponentError = int(av[3]) - 1
                componentError = True
                deviceType = "R"
            except:
                print("Error: no number in the argument")
        elif errorType == "-cc":
            try:
                nComponentError = int(av[3]) - 1
                componentError = True
                deviceType = "C"
            except:
                print("Error: no number in the argument")

    # Connect to the robot and preliminary operations.
    robot = NiryoRobot("169.254.200.200")
    robot.calibrate_auto()
    robot.close_gripper(speed=500)
    robot.set_arm_max_velocity(100)
    setSpeedConveyor(100)
    robot.move_to_home_pose()
    global conveyor_id
    conveyor_id = robot.set_conveyor()

    # Initialize the ROS node and start the subscribers.
    listener()