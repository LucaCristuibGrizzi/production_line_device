#!/usr/bin/env python
import threading
import rospy
from std_msgs.msg import String, Bool

# Custom Imports
from production_line_device.msg import InitialCondition, ProductionLineStatus, PickAndPlace

#--------- GLOBAL VARIABLE ---------#
# Publishers needed for communicating with all devices of the production line.
pubR1 = rospy.Publisher("cmd_r1", PickAndPlace, queue_size=10)
pubR2 = rospy.Publisher("cmd_r2", PickAndPlace, queue_size=10)
pubR3 = rospy.Publisher("cmd_r3", PickAndPlace, queue_size=10)
pubConveyorRun = rospy.Publisher("cmd_run_conveyor", String, queue_size=10)
pubConveyorStop = rospy.Publisher("cmd_stop_conveyor", String, queue_size=10)
pubObjectDetection = rospy.Publisher("object_detection", String, queue_size=10)
pubCheckAssembly = rospy.Publisher("check_assembly", String, queue_size=10)
pubRotatingTable = rospy.Publisher("cmd_rotating_table", String, queue_size=10)

# Publisher needed for communicating with the FlexBE behavior "Production Line".
pubStatus = rospy.Publisher("production_line_status", ProductionLineStatus, queue_size=10)

# Variable needed to start the operation of the production line 
# and manage errors correctly.
runProductionLine = True
maxTime = 40
setInitialCondition = True
counterColorCode = 0
colorCodes = None
r1PosesNames = None
r2PosesNames = None
r3PosesNames = None

# Variable used for synchronizing all operations of the production line.
# Note that these global variables are shared across multiple threads.
# Refer to the state of the operations of the robots.
isR1Executing = False
isR2Executing = False
isR3Executing = False
isR1Executed = False
isR2Executed = False
isR3Executed = False
nOperationR3 = 0
isW2Busy = False

# Refer to the state of the operations of the conveyor belt and the IR sensor.
startTime = None
isConveyorRunning = False
isConveyorBeltStopped = True
isConveyorBeltFreeAtStart = True
isConveyorBeltFreeAtEnd = True
afterIRDetect = False
IRDetect = False

# Refer to the state of the operations of the rotating table.
isRotatingTableBusy = False
isRotatingTableMoving = False
hasRotatingTableMoved = False

# Refer to the state of the camera vision operations.
isObjectDetectedLeft = False
isObjectDetectedRight = False
isAssemblyOK = False

# Allow avoiding publishing multiple messages for a single operation.
isRunConveyorBeltCommandPublished = False
isStopConveyorBeltCommandPublished = False
isLeftObjectDetectionCommandPublished = False
isRightObjectDetectionCommandPublished = False
isCheckAssemblyCommandPublished = False

# Lock variable for all the global variables that are shared across multiple threads
# and might be accessed at the same time.
lock_runProductionLine = threading.Lock()
lock_counterColorCode = threading.Lock()
lock_afterIRDetect = threading.Lock()
lock_isConveyorBeltStopped = threading.Lock()
lock_isConveyorBeltFreeAtStart = threading.Lock()
lock_isConveyorBeltFreeAtEnd = threading.Lock()
lock_isR1Executing = threading.Lock()
lock_isR2Executing = threading.Lock()
lock_isR3Executing = threading.Lock()
lock_isR1Executed = threading.Lock()
lock_isR2Executed = threading.Lock()
lock_isR3Executed = threading.Lock()
lock_nOperationR3 = threading.Lock()
lock_IRDetect = threading.Lock()
lock_isW2Busy = threading.Lock()
lock_isRotatingTableBusy = threading.Lock()
lock_isRotatingTableMoving = threading.Lock()
lock_hasRotatingTableMoved = threading.Lock()
lock_isObjectDetectedLeft = threading.Lock()
lock_isObjectDetectedRight = threading.Lock()
lock_isAssemblyOK = threading.Lock()
lock_isRunConveyorBeltCommandPublished  = threading.Lock()
lock_isStopConveyorBeltCommandPublished  = threading.Lock()
lock_isLeftObjectDetectionCommandPublished = threading.Lock()
lock_isRightObjectDetectionCommandPublished = threading.Lock()
lock_isCheckAssemblyCommandPublished = threading.Lock()

#--------- FUNCTION ---------#
# Functions for getting and setting variables shared across multiple threads.
def get_runProductionLine():
    with lock_runProductionLine:
        return runProductionLine
    
def set_runProductionLine(value):
    with lock_runProductionLine:
        global runProductionLine
        runProductionLine = value

def get_afterIRDetect():
    with lock_afterIRDetect:
        return afterIRDetect
    
def set_afterIRDetect(value):
    with lock_afterIRDetect:
        global afterIRDetect
        afterIRDetect = value

def get_isRunConveyorBeltCommandPublished():
    with lock_isRunConveyorBeltCommandPublished:
        return isRunConveyorBeltCommandPublished
    
def set_isRunConveyorBeltCommandPublished(value):
    with lock_isRunConveyorBeltCommandPublished:
        global isRunConveyorBeltCommandPublished
        isRunConveyorBeltCommandPublished = value

def get_isStopConveyorBeltCommandPublished():
    with lock_isStopConveyorBeltCommandPublished:
        return isStopConveyorBeltCommandPublished
    
def set_isStopConveyorBeltCommandPublished(value):
    with lock_isStopConveyorBeltCommandPublished:
        global isStopConveyorBeltCommandPublished
        isStopConveyorBeltCommandPublished = value

def get_isLeftObjectDetectionCommandPublished ():
    with lock_isLeftObjectDetectionCommandPublished:
        return isLeftObjectDetectionCommandPublished
    
def set_isLeftObjectDetectionCommandPublished(value):
    with lock_isLeftObjectDetectionCommandPublished:
        global isLeftObjectDetectionCommandPublished
        isLeftObjectDetectionCommandPublished = value

def get_isRightObjectDetectionCommandPublished():
    with lock_isRightObjectDetectionCommandPublished:
        return isRightObjectDetectionCommandPublished
    
def set_isRightObjectDetectionCommandPublished(value):
    with lock_isRightObjectDetectionCommandPublished:
        global isRightObjectDetectionCommandPublished
        isRightObjectDetectionCommandPublished = value

def get_isCheckAssemblyCommandPublished():
    with lock_isCheckAssemblyCommandPublished:
        return isCheckAssemblyCommandPublished
    
def set_isCheckAssemblyCommandPublished(value):
    with lock_isCheckAssemblyCommandPublished:
        global isCheckAssemblyCommandPublished
        isCheckAssemblyCommandPublished = value

def get_counterColorCode():
    with lock_counterColorCode:
        return counterColorCode

def add_counterColorCode():
    with lock_counterColorCode:
        global counterColorCode
        counterColorCode += 1

def get_isConveyorBeltStopped():
    with lock_isConveyorBeltStopped:
        return isConveyorBeltStopped
    
def set_isConveyorBeltStopped(value):
    with lock_isConveyorBeltStopped:
        global isConveyorBeltStopped
        isConveyorBeltStopped = value

def get_isConveyorBeltFreeAtStart():
    with lock_isConveyorBeltFreeAtStart:
        return isConveyorBeltFreeAtStart
    
def set_isConveyorBeltFreeAtStart(value):
    with lock_isConveyorBeltFreeAtStart:
        global isConveyorBeltFreeAtStart
        isConveyorBeltFreeAtStart = value

def get_isConveyorBeltFreeAtEnd():
    with lock_isConveyorBeltFreeAtEnd:
        return isConveyorBeltFreeAtEnd
    
def set_isConveyorBeltFreeAtEnd(value):
    with lock_isConveyorBeltFreeAtEnd:
        global isConveyorBeltFreeAtEnd
        isConveyorBeltFreeAtEnd = value

def get_isR1Executing():
    with lock_isR1Executing:
        return isR1Executing
    
def set_isR1Executing(value):
    with lock_isR1Executing:
        global isR1Executing
        isR1Executing = value

def get_isR2Executing():
    with lock_isR2Executing:
        return isR2Executing
    
def set_isR2Executing(value):
    with lock_isR2Executing:
        global isR2Executing
        isR2Executing = value

def get_isR3Executing():
    with lock_isR3Executing:
        return isR3Executing
    
def set_isR3Executing(value):
    with lock_isR3Executing:
        global isR3Executing
        isR3Executing = value

def get_isR1Executed():
    with lock_isR1Executed:
        return isR1Executed
    
def set_isR1Executed(value):
    with lock_isR1Executed:
        global isR1Executed
        isR1Executed = value

def get_isR2Executed():
    with lock_isR2Executed:
        return isR2Executed
    
def set_isR2Executed(value):
    with lock_isR2Executed:
        global isR2Executed
        isR2Executed = value

def get_isR3Executed():
    with lock_isR3Executed:
        return isR3Executed
    
def set_isR3Executed(value):
    with lock_isR3Executed:
        global isR3Executed
        isR3Executed = value
        
def get_nOperationR3():
    with lock_nOperationR3:
        return nOperationR3
    
def set_nOperationR3(value):
    with lock_nOperationR3:
        global nOperationR3
        nOperationR3 = value

def get_IRDetect():
    with lock_IRDetect:
        return IRDetect
    
def set_IRDetect(value):
    with lock_IRDetect:
        global IRDetect
        IRDetect = value

def get_isW2Busy():
    with lock_isW2Busy:
        return isW2Busy
    
def set_isW2Busy(value):
    with lock_isW2Busy:
        global isW2Busy
        isW2Busy = value

def get_isRotatingTableBusy():
    with lock_isRotatingTableBusy:
        return isRotatingTableBusy
    
def set_isRotatingTableBusy(value):
    with lock_isRotatingTableBusy:
        global isRotatingTableBusy
        isRotatingTableBusy = value

def get_isRotatingTableMoving():
    with lock_isRotatingTableMoving:
        return isRotatingTableMoving
    
def set_isRotatingTableMoving(value):
    with lock_isRotatingTableMoving:
        global isRotatingTableMoving
        isRotatingTableMoving = value

def get_hasRotatingTableMoved():
    with lock_hasRotatingTableMoved:
        return hasRotatingTableMoved
    
def set_hasRotatingTableMoved(value):
    with lock_hasRotatingTableMoved:
        global hasRotatingTableMoved
        hasRotatingTableMoved = value

def get_isObjectDetectedLeft():
    with lock_isObjectDetectedLeft:
        return isObjectDetectedLeft
    
def set_isObjectDetectedLeft(value):
    with lock_isObjectDetectedLeft:
        global isObjectDetectedLeft
        isObjectDetectedLeft = value

def get_isObjectDetectedRight():
    with lock_isObjectDetectedRight:
        return isObjectDetectedRight
    
def set_isObjectDetectedRight(value):
    with lock_isObjectDetectedRight:
        global isObjectDetectedRight
        isObjectDetectedRight = value

def get_isAssemblyOK():
    with lock_isAssemblyOK:
        return isAssemblyOK
    
def set_isAssemblyOK(value):
    with lock_isAssemblyOK:
        global isAssemblyOK
        isAssemblyOK = value


# CORE FUNCTION -> Manages the synchronization and execution of operations 
#                  for all devices in the production line.
def productionLine(data):
    # Initial conditions are set only at the start of the operation.
    # When this function resumes after error management, the initial conditions
    # do not need to be reset.
    global setInitialCondition, startTime, isConveyorRunning

    if setInitialCondition:
        defineInitialCondition(data)
        setInitialCondition = False

        startTime = None
        isConveyorRunning = False

    # Set that the production line can start its operations.
    set_runProductionLine(True)

    # Loop that manages the work of the production line.
    # It is stopped only if an error is detected, allowing for error management.
    # Its work finishes when all three products are assembled.
    while get_runProductionLine():

        # Robot1 can execute its operations only if the conveyor belt is stationary,
        # the first workstation is available, and the robot hasn't completed all of its operations.
        if (len(r1PosesNames) != 0 and get_isConveyorBeltStopped() and
            get_isConveyorBeltFreeAtStart() and not get_isR1Executing()):
            
            # A delay is added to avoid communication issues between Robot1, the conveyor belt, 
            # and the IR sensor, which may occur with simultaneous commands.
            rospy.sleep(1)

            # Define the message for PickAndPlace operations and publish it.
            operations = PickAndPlace()
            operations.nOperations = 2
            operations.pickPoses = [r1PosesNames[0], r1PosesNames[1]]
            operations.placePoses = ["Pose_C_1_2", "Pose_E_Top_1_2"]
            pubR1.publish(operations)
            print("R1 published")

            # Update the variables for correct synchronization.
            set_isR1Executing(True)
            set_isConveyorBeltFreeAtStart(False)
  
        # The conveyor belt can execute its operation only if the second workstation (at the end of this device) is available,
        # Robot1 has finished its operation, and Robot2 isn't executing.
        # Furthermore, this check ensures not to publish multiple messages.   
        if (get_isConveyorBeltFreeAtEnd() and get_isR1Executed() and not get_isR2Executing() and
             not get_isRunConveyorBeltCommandPublished()):
            
            # Define the message to start the conveyor belt and publish it.
            pubConveyorRun.publish("RUN")
            print("Run conveyor published")

            # Update the variables for correct synchronization.
            set_isRunConveyorBeltCommandPublished(True)
            isConveyorRunning = True

            # Set the start time to check if the product takes longer than the maximum
            # time to reach the second workstation.
            startTime = rospy.get_rostime()
            
        # When the Infrared Sensor detects the presence of the product in the second workstation, 
        # the conveyor belt should be stopped. Furthermore, this check ensures not to publish multiple messages. 
        if get_IRDetect() and not get_isStopConveyorBeltCommandPublished():
            
            # Define the message to stop the conveyor belt and publish it.
            pubConveyorStop.publish("STOP")
            print("Stop conveyor published")
            
            # Update the variables for correct synchronization.
            set_afterIRDetect(True)
            set_isStopConveyorBeltCommandPublished(True)
            set_IRDetect(False)
            isConveyorRunning = False  # Note that this variable is used only within this thread.

        # While the conveyor is in the running state, check if the time elapsed to carry the 
        # product between the first workstation and the second is greater than the maximum time.
        if isConveyorRunning and (rospy.get_rostime() - startTime).to_sec() > float(maxTime):
            print("Max time elapsed")
            
            # Update the variables for correct synchronization.
            set_runProductionLine(False)
            set_isRunConveyorBeltCommandPublished(False)
            set_isR1Executed(True)
            isConveyorRunning = False

            # Define the message to indicate the error of component not arrived at the second workstation
            # and publish it.
            status = setStatusError("component_not_arrived", faulty_device = 2)
            pubStatus.publish(status)
            
        # Robot2 can execute its operations only if the conveyor belt is stationary, the product is in the second workstation,
        # warehouse2 is not busy with Robot3, no products are on the rotating table,
        # and the robot hasn't completed all of its operations.  
        if (get_afterIRDetect() and len(r2PosesNames) != 0 and get_isConveyorBeltStopped() and
            not get_isR2Executing() and not get_isW2Busy() and not get_isRotatingTableBusy()):
            
            # Define the message for PickAndPlace operations and publish it.
            operations = PickAndPlace()
            operations.nOperations = 2
            operations.pickPoses = [r2PosesNames[0], "Pose_C_2_3"]
            operations.placePoses = ["Pose_E_Top_2_3", "Pose_C_3_4"]
            pubR2.publish(operations)
            print("R2 published")

            # Update the variables for correct synchronization.
            set_isW2Busy(True)
            set_isR2Executing(True)
            set_isRotatingTableBusy(True)

        # The object detection operation on the left side of the rotating table checks if the product was really
        # carried to this workstation by Robot2. So, to execute it, Robot2 must have completed its operation,
        # and Robot3 must not be executing.
        # Furthermore, this check ensures not to publish multiple messages. 
        if get_isR2Executed() and not get_isR3Executing() and not get_isLeftObjectDetectionCommandPublished():

            # Define the message to execute the object detection on the left side of the rotating table
            # and publish it.
            pubObjectDetection.publish("L")
            print("Object Detection Left Published")

            # Update the variables for correct synchronization.
            set_isLeftObjectDetectionCommandPublished(True)
            set_isR2Executed(False)

        # If vision operation on the left side is successful, the check assembly operation can be executed
        # to obtain the color code of the actual product. Also Robot3 must not be executing.
        # Additionally, this check ensures not to publish multiple messages. 
        if (get_isObjectDetectedLeft() and not get_isR3Executing() and not get_isR2Executing() and
             not get_isCheckAssemblyCommandPublished()):
            
            # Define the message to execute the check assembly operation and publish it.
            # Note that "CHECKinLeft" is used for better understanding, but the name doesn't affect the operation.
            pubCheckAssembly.publish("CHECKinLeft") 
            print("Check Assembly Left Published")

            # Update the variable for correct synchronization.
            set_isCheckAssemblyCommandPublished(True)
        
        # If both vision operations on the left side of the rotating table are successful,
        # the rotating table can execute a 180° rotation. Additionally, this check ensures
        # not to publish multiple messages when the rotating table is moving 
        if get_isObjectDetectedLeft() and get_isAssemblyOK() and not get_isRotatingTableMoving():

            # Define the message to start the rotating table and publish it.
            pubRotatingTable.publish("RUN")
            print("Run Rotating Table Published")

            # Update the variables for correct synchronization.
            set_isRotatingTableMoving(True)
            set_hasRotatingTableMoved(False)
            set_isAssemblyOK(False)
            set_isObjectDetectedLeft(False)
            set_isCheckAssemblyCommandPublished(False)

        # To execute object detection on the right side of the rotating table, the rotating table
        # must have completed its operation, and neither Robot3 nor Robot2 should be executing.
        # Furthermore, this check ensures not to publish multiple messages.   
        if (get_hasRotatingTableMoved() and not get_isR3Executing()
            and not get_isR2Executing() and not get_isRightObjectDetectionCommandPublished()):

            # Define the message to execute the object detection on the right side of the rotating table
            # and publish it.
            pubObjectDetection.publish("R")
            print("Object Detection Right Published")

            # Update the variable for correct synchronization.
            set_isRightObjectDetectionCommandPublished(True)

        # If the object detection on the right side is successful, Robot3 can execute its first assembly operation.
        # Conditions include warehouse2 not being busy with Robot2, Robot3 not having completed all operations,
        # and the last operation executed by Robot3 being its second operation in the assembly process. 
        if (len(r3PosesNames["first_operation"]) != 0 and get_isObjectDetectedRight() and
            get_nOperationR3() != 1 and not get_isW2Busy() and not get_isR3Executing()):

            # Define the message for PickAndPlace operations and publish it.
            operations = PickAndPlace()
            operations.nOperations = 2
            operations.pickPoses = [r3PosesNames["first_operation"][0], "Pose_C_4_5"]
            operations.placePoses = ["Pose_E_Top_4_5", "Pose_C_4_5_View"]
            pubR3.publish(operations)
            print("R3 first operation published")

            # Update the variables for correct synchronization
            set_isW2Busy(True)
            set_isR3Executing(True)
            set_isR3Executed(False)
            set_nOperationR3(1)

        # After the first task of Robot3, now the product shows its colored side to the camera; hence,
        # another check assembly can be executed. Also, Robot2 must not be executing.
        # Additionally, this check ensures not to publish multiple messages. 
        if (get_isR3Executed() and not get_isR2Executing() and get_isObjectDetectedRight() and not get_isCheckAssemblyCommandPublished()):
            
            # Define the message to execute the check assembly operation and publish it.
            # Note that "CHECKinRight" is used for better understanding, but the name doesn't affect the operation.
            pubCheckAssembly.publish("CHECKinRight")
            print("Check Assembly Right Published")

            # Update the variable for correct synchronization.
            set_isCheckAssemblyCommandPublished(True)

        # If the check assembly on the right side is successful, Robot3 can execute its second assembly operation.
        # Conditions include Robot3 not having completed all operations
        # and the last operation executed by Robot3 being its first operation in the assembly process.  
        if (len(r3PosesNames["second_operation"]) != 0 and get_nOperationR3() != 2 and
            get_isObjectDetectedRight() and get_isAssemblyOK() and not get_isR3Executing()):

            # Define the message for PickAndPlace operations and publish it.
            operations = PickAndPlace()
            operations.nOperations = 1
            operations.pickPoses = ["Pose_C_4_5_View"]
            operations.placePoses = [r3PosesNames["second_operation"][0]]
            pubR3.publish(operations)
            print("R3 second operation published")

            # Update the variables for correct synchronization.
            set_isR3Executing(True)
            set_isR3Executed(False)
            set_isAssemblyOK(False)
            set_isObjectDetectedRight(False)
            set_nOperationR3(2)
            set_isCheckAssemblyCommandPublished(False)
            set_isRotatingTableBusy(False)

        # If Robot3 has completed all its second operations, signal the end of the production line.
        if len(r3PosesNames["second_operation"]) == 0:
            
            # Define and publish a message indicating the completion of all operations.
            status = ProductionLineStatus()
            status.finished = True
            status.error = False
            pubStatus.publish(status)
            print("Finished")

            # Update the variable to stop the production line.
            set_runProductionLine(False)      

# Function to handle the response of Robot1 at the end of its operation.
def r1OperationStatus(data):
    currentTime = rospy.Time.now().to_sec()
    rospy.loginfo("Time: %f; \nData R1 operation: %s" % (currentTime, data.data))

    # If the operation completes correctly, publish to the "Production Line" behavior
    # to update the status of warehouse_1.
    if data.data == "OK":
        status = setStatusRobotOperations("robot1", "warehouse_1", r1PosesNames[1])
        pubStatus.publish(status)

        # Delete the first two poses from the list, so the next operation
        # can execute the correct pick-and-place.
        del r1PosesNames[:2]

        # Update the variables for correct synchronization.
        set_isR1Executing(False)
        set_isR1Executed(True)

    # If a different message is received, this means that a slowdown of the robot was detected.  
    else:
        # Update the variables for correct synchronization.
        set_runProductionLine(False)
        set_isR1Executing(False)
        set_isConveyorBeltFreeAtStart(False)

        # Define the message to indicate the error of slowdown and publish it.
        status = setStatusError("slowdown", speed= int(data.data))
        pubStatus.publish(status)

# Function to handle the response of Robot2 at the end of its operation.
def r2OperationStatus(data):
    currentTime = rospy.Time.now().to_sec()
    rospy.loginfo("Time: %f; \nData R2 operation: %s" % (currentTime, data.data))

    # If the operation completes correctly, publish to the "Production Line" behavior
    # to update the status of warehouse_2.
    if data.data == "OK":
        status = setStatusRobotOperations("robot2", "warehouse_2", r2PosesNames[0])
        pubStatus.publish(status)

        # Delete the first pose from the list, so the next operation
        # can execute the correct pick-and-place.
        del r2PosesNames[0]

        # Update the variables for correct synchronization.
        set_isR2Executing(False)
        set_isR2Executed(True)
        set_isW2Busy(False)
        set_isAssemblyOK(False)
        set_afterIRDetect(False)

    # If a different message is received, this means that a slowdown of the robot was detected.  
    else:
        # Update the variables for correct synchronization.
        set_runProductionLine(False)
        set_isR2Executing(False)
        set_isW2Busy(False)
        set_isRotatingTableBusy(False)

        # Define the message to indicate the error of slowdown and publish it.
        status = setStatusError("slowdown", speed= int(data.data))
        pubStatus.publish(status)

# Function to handle the response of Robot3 at the end of its operation.
def r3OperationStatus(data):
    currentTime = rospy.Time.now().to_sec()

    # If the operation completes correctly, manage the different procedures between the first
    # and second tasks of Robot3.
    if data.data == "OK":

        # Update the variables that are common between the first and
        # second tasks of Robot3 for correct synchronization.
        set_isR3Executing(False)
        set_isR3Executed(True)
        
        # If the operation completes correctly is the first one,
        # publish to the "Production Line" behavior to update the status of warehouse_2.
        if get_nOperationR3() == 1:
            rospy.loginfo("Time: %f; \nData R3 Operation First: %s" % (currentTime, data.data))

            status = setStatusRobotOperations("robot3", "warehouse_2", r3PosesNames["first_operation"][0])
            pubStatus.publish(status)

            # Delete the first pose from the list, so the next operation
            # can execute the correct pick-and-place.
            del r3PosesNames["first_operation"][0]

            # Update the variable for correct synchronization.
            set_isW2Busy(False)

        # If the operation completes correctly is the second one, it isn't necessary communicate
        # with the "Production Line" behavior because the warehouse_3 doesn't need to update its status.     
        elif get_nOperationR3() == 2:
            rospy.loginfo("Time: %f; \nData R3 Operation Second: %s" % (currentTime, data.data))

            # Delete the first pose from the list, so the next operation
            # can execute the correct pick-and-place.
            del r3PosesNames["second_operation"][0]

    # If a different message is received, this means that a slowdown of the robot was detected.          
    else:
        # Update the variables that are common between the first and
        # second tasks of Robot3 for correct synchronization.
        set_runProductionLine(False)
        set_isR3Executing(False)
        set_isW2Busy(False)

        # The operation in which the slowdown is detected needs to be re-executed, so
        # the value of nOperationR3 needs to be set to a correct value that allows this.
        if get_nOperationR3() == 1:
            set_nOperationR3(0)
        elif get_nOperationR3() == 2:
            set_nOperationR3(1)

        # Define the message to indicate the error of slowdown and publish it.
        status = setStatusError("slowdown", speed= int(data.data))
        pubStatus.publish(status)

# Function to handle the response of the conveyor belt at the end of its operation.
def conveyorOperationStatus(data):
    currentTime = rospy.Time.now().to_sec()

    # If the operation completes correctly, manage the different procedures 
    # between the run and stop commands.
    if data.data == "OK":
        if get_isRunConveyorBeltCommandPublished():
            rospy.loginfo("Time: %f; \nData Run Conveyor: %s" % (currentTime, data.data))

            # Update the variables for correct synchronization.
            set_isR1Executed(False)
            set_isRunConveyorBeltCommandPublished(False)
        elif get_isStopConveyorBeltCommandPublished():
            rospy.loginfo("Time: %f; \nData Stop Conveyor: %s" % (currentTime, data.data))

            # Update the variables for correct synchronization.
            set_isConveyorBeltFreeAtStart(True)
            set_isConveyorBeltStopped(True)
            set_isStopConveyorBeltCommandPublished(False)
    
    # If a different message is received, this means that a slowdown of the conveyor belt was detected.
    else:
        # Update the variables for correct synchronization.
        set_runProductionLine(False)
        set_isRunConveyorBeltCommandPublished(False)

        # Define the message to indicate the error of slowdown and publish it.
        status = setStatusError("slowdown", speed= int(data.data))
        pubStatus.publish(status)

# Function to handle the response of the Infrared Sensor when it detects the arrival
# of the product at the workstation at the end of the conveyor belt.
def checkIR(data):
    currentTime = rospy.Time.now().to_sec()
    rospy.loginfo("Time: %f; \nData CheckIR: %s" % (currentTime, data.data))

    # If the operation completes correctly, the data value is True.
    if data.data:
        # Update the variables for correct synchronization.
        set_IRDetect(True)
        set_isConveyorBeltFreeAtEnd(False)

# Function to handle the response of the rotating table at the end of its operation.
def rsRotatingTable(data):
    currentTime = rospy.Time.now().to_sec()

    # If the operation completes correctly, manage the procedure to update
    # the state of the production line.
    if data.data == "OK":
        rospy.loginfo("Time: %f; \nData Rotating Table: %s" % (currentTime, data.data))

        # Update the variables for correct synchronization.
        set_isRotatingTableMoving(False)
        set_hasRotatingTableMoved(True)

    # If a different message is received, this means that a slowdown of the rotating table was detected.
    else:
        # Update the variables for correct synchronization.
        set_runProductionLine(False)
        set_isRotatingTableMoving(False)

        # Define the message to indicate the error of slowdown and publish it.
        status = setStatusError("slowdown", speed= int(data.data))
        pubStatus.publish(status)

# Function to handle the response of the check assembly operation.
def rsCheckAssembly(data):
    currentTime = rospy.Time.now().to_sec()
    rospy.loginfo("Time: %f; \nData Check Assembly: %s" % (currentTime, data.data))

    # The procedure has a few differences if the product is on the left or the right side of the rotating table.
    # Left
    if get_isObjectDetectedLeft():
        # Extract the color code detected from the data string received.
        colorCodeDetected = data.data.split(";")

        # Check if the color code detected is correct; otherwise, launch the error of wrong assembly.
        # Note: For this workstation, the expected number of elements inside the case is 2.
        # The counter of the color code is used to select the correct desired color code for the 
        # product being processed.
        if checkColorCode(colorCodes[get_counterColorCode()], colorCodeDetected, 2):
            # Update the variable for correct synchronization.
            set_isAssemblyOK(True)
        else:
            # Update the variable to pause the production line.
            set_runProductionLine(False)

            # Define the message with all data needed to manage the error of wrong assembly and publish it.
            status = setStatusError("assembly", warehouseName="warehouse_2", faulty_device = 3, colorCodeDetected = colorCodeDetected,
                                    nProduct = get_counterColorCode()+1, nElements = 2)
            pubStatus.publish(status)

            # Update the variable for correct synchronization.
            set_isCheckAssemblyCommandPublished(False)

    # Right
    elif get_isObjectDetectedRight():
        # Extract the color code detected from the data string received.
        colorCodeDetected = data.data.split(";")

        # Check if the color code detected is correct; otherwise, launch the error of wrong assembly.
        # Note: For this workstation, the expected number of elements inside the case is 3.
        if checkColorCode(colorCodes[get_counterColorCode()], colorCodeDetected, 3):
            # Update the variables for correct synchronization.
            set_isAssemblyOK(True)
            add_counterColorCode()
        else:
            # Update the variable to pause the production line.
            set_runProductionLine(False)

            # Define the message with all data needed to manage the error of wrong assembly and publish it.
            status = setStatusError("assembly", warehouseName="warehouse_2", faulty_device = 5, colorCodeDetected = colorCodeDetected,
                                    nProduct = get_counterColorCode()+1, nElements = 3)
            pubStatus.publish(status)

            # Update the variable for correct synchronization.
            set_isCheckAssemblyCommandPublished(False)

# Function to handle the response of the object detection operation.    
def rsObjectDetection(data):
    currentTime = rospy.Time.now().to_sec()

    # The procedure has a few differences if the product is on the left or the right side of the rotating table.
    # Left
    if get_isLeftObjectDetectionCommandPublished():
        
        # Check if the object detection is successful; otherwise, launch the error of component not arrived.
        if data.data:
            rospy.loginfo("Time: %f; \nData OD Left: %s" % (currentTime, data.data))

            # Update the variables to pause the production line.
            set_isObjectDetectedLeft(True)
            set_isConveyorBeltFreeAtEnd(True)
            set_isLeftObjectDetectionCommandPublished(False)
        else:
            # Update the variables to pause the production line.
            set_runProductionLine(False)
            set_isLeftObjectDetectionCommandPublished(False)
            set_isR2Executed(True)

            # Define the message to indicate the error of component not arrived at the third workstation
            # and publish it.
            status = setStatusError("component_not_arrived", faulty_device = 3)
            pubStatus.publish(status)
    # Right
    elif get_isRightObjectDetectionCommandPublished():

        # Check if the object detection is successful; otherwise, launch the error of component not arrived.
        if data.data:
            rospy.loginfo("Time: %f; \nData OD Right: %s" % (currentTime, data.data))

            # Update the variables to pause the production line.
            set_isObjectDetectedRight(True)
            set_hasRotatingTableMoved(False)
            set_isRightObjectDetectionCommandPublished(False)
        else:
            # Update the variables to pause the production line.
            set_runProductionLine(False)
            set_isRightObjectDetectionCommandPublished(False)

            # Define the message to indicate the error of component not arrived at the last workstation
            # and publish it.
            status = setStatusError("component_not_arrived", faulty_device = 4)
            pubStatus.publish(status)

# Function to check if the components are assembled correctly.
# If the color code detected corresponds with the expected color code for the specified number of components,
# the product is assembled correctly (returns True); otherwise, the check is unsuccessful (returns False).
def checkColorCode(colorCode, colorCodeDetected, nComponents):
    if colorCodeDetected[0] == "No color found":
        return False
    elif colorCodeDetected == colorCode[: nComponents]:
        return True
    else:
        return False

# Function that sets up all the variables needed for the production line operations
# based on the Initial Condition message.    
def defineInitialCondition(data):
    global colorCodes, r1PosesNames, r2PosesNames, r3PosesNames

    colorCodes = [data.firstColorCode, data.secondColorCode, data.thirdColorCode]
    r1PosesNames = data.robot1Poses
    r2PosesNames = data.robot2Poses
    r3PosesNames = {
        "first_operation": data.robot3PosesFirstOperation,
        "second_operation": data.robot3PosesSecondOperation
    }

# Function that sets up the status messagge (based on the info passed) to comunicate 
# to the "Production Line" behavior that the status of the warehouses need to be updated.
def setStatusRobotOperations(robotName, warehouseName, poseWarehouse):
    status = ProductionLineStatus()
    status.finished = False
    status.error = False
    status.robotName = robotName
    status.warehouseName = warehouseName
    status.poseWarehouse = poseWarehouse
    return status

# Function that sets up the status messagge (based on the info passed) to comunicate 
# to the "Production Line" behavior that error detected in the production line.
def setStatusError(error_type, warehouseName = "", speed = 0, faulty_device = 0, colorCodeDetected = None, nProduct = 0, nElements = 0):
    status = ProductionLineStatus()
    status.finished = False
    status.error = True

    if error_type == "slowdown":
        status.error_type = error_type
        status.speed = speed
    elif error_type == "component_not_arrived":
        status.error_type = error_type
        status.faulty_device = faulty_device
    elif error_type == "assembly":
        status.error_type = error_type
        status.colorCodeDetected = colorCodeDetected
        status.faulty_device = faulty_device
        status.nProduct = nProduct
        status.nElements = nElements
        status.warehouseName = warehouseName

    return status

# Function that defines the ROS node and sets up subscribers for communication with
# devices and the FlexBE behavior "Production Line".
def listener():
    rospy.init_node("production_line", anonymous=True)
    rospy.Subscriber("start_production_line", InitialCondition, productionLine)
    rospy.Subscriber("r1_operation_status", String, r1OperationStatus)
    rospy.Subscriber("r2_operation_status", String, r2OperationStatus)
    rospy.Subscriber("r3_operation_status", String, r3OperationStatus)
    rospy.Subscriber("conveyor_operation_status", String, conveyorOperationStatus)
    rospy.Subscriber("check_IR", Bool, checkIR)
    rospy.Subscriber("rs_rotating_table", String, rsRotatingTable)
    rospy.Subscriber("rs_check_assembly", String, rsCheckAssembly)
    rospy.Subscriber("rs_object_detection", Bool, rsObjectDetection)
    rospy.spin()

if __name__ == "__main__":
    listener()