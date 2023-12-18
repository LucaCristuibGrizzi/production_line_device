from pyniryo import *

# IP address of the robot.
# Change the value based on the robot you are controlling.
IP_address = "169.254.200.202"

# Poses to define
Pose_E_Top_4_5 = (0.233, -0.811, 0.352, -0.009, -1.074, -1.288)
Pose_E_W2_R_2 = (0.484, -1.207, 0.661, 0.026, -0.960, -1.236)
Pose_E_W2_R_3 = (0.621, -1.488, 1.211, 0.014, -1.223, -1.096)
Pose_E_W2_G_1 = (0.371, -0.841, -0.066, 0.058, -0.595, -1.360)
Pose_E_W2_G_2 = (0.551, -0.928, 0.106, 0.031, -0.675, -1.158)
Pose_E_W2_G_3 = (0.700, -1.084, 0.417, 0.011, -0.828, -0.995)

if __name__ == "__main__":
    
    # Connection to the robot.
    robot = NiryoRobot(IP_address)

    # Automatic Calibration of the robot. If needed, it will be executed.
    robot.calibrate_auto()

    # Print the joints pose of the robot.
    # After you find the position desired, save this joints pose in the Pose.py file.
    joints = robot.get_joints()
    print(joints)

    # Get the end-effector position from the joint pose.
    Position = robot.forward_kinematics(Pose_E_Top_4_5)

    # Define a new position with an offset.
    # The offset represents the delta between the real position and the desired position.
    # At the first run with a new position, it is advisable to only have a positive offset on the z-axis.
    # Remember to always position the robot in a safe state before executing any movement.
    # Ensure there are no objects present that could cause a collision between the current
    # pose of the robot and the target pose.
    PositionOffset = Position.copy_with_offsets(z_offset=0.05, x_offset=0.00, y_offset=0.0,
                                                yaw_offset=0.0, roll_offset=0.0, pitch_offset=0.0)
    

    # The following part of the code can be modified based on the necessity of the case in question.

    # Open the gripper
    # This line can be commented if not needed.
    robot.open_gripper(speed=500)

    # Move the robot to the position specified.
    robot.move_pose(PositionOffset)

    # Close gripper.
    # This line can be commented if not needed.
    robot.close_gripper(speed=500)

    # Return the robot to home pose.
    # This line can be commented if not needed.
    # robot.move_to_home_pose()
    
    # Close connection
    robot.close_connection()
    