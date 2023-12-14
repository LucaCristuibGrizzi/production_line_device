from enum import Enum

# In this module, three enumerations are defined, one for each robot in the production line.
# These enumerations define all the joint poses that the robots can reach.
# Poses are represented as tuples with joint values [rad].
# The nomenclature for the production line poses is as follows: Pose_Type_Info_Where, where:
# - Pose: is the prefix for all names.
# - Type: is used to indicate the type of the element that the robot will grasp. It can be "C" for
#   the case, "E" for the internal element, or "E1," "E2," and "E3" for the element during the product disassembly phases.
# - Info: is used to add additional information. For example, "Top" to indicate the release position
#   for inserting internal elements into the case, or "Help" to indicate that it is an assistance position for disassembly.
# - Where: provides information about the layout zone of the production line. For items in the warehouses,
#   this format is followed: "Wi_Color_Number," where i is the warehouse number, Color can be "R", "G", and "B",
#   and is only necessary for colored elements, Number is used to indicate the position in the warehouse, especially
#   in the case of repeating items. For workstation positions, the numbering of the line devices is used, ranging
#   from 1 for robot 1 to 5 for robot 3 (sensors are not associated with a number). Therefore, it follows the format "i_j,"
#   where i is the number of the previous device, and j is the number of the next one. For example, for the station between
#   robot 2 and the rotating table, it will be "3_4".
# Note that some poses may not adhere to this nomenclature. In such cases, the poses are not searched with an algorithm but
# are manually forced in the code, so the nomenclature loses its importance for those cases.
# In conclusion, the poses in this module are only those needed for testing the production line in this work.
# More poses can be added to achieve a complete working production line. 

# Robot1
class NedPosesR1(Enum):
    Home_Pose = (0, 0.30, -1.30, 0, 0, -0.007)
    
    # Pose for warehouses.
    Pose_C_W1_1 = (-0.244, -0.507, -0.674, -0.144, -0.456, -0.170)
    Pose_C_W1_2 = (-0.218, -0.649, -0.391, -0.109, -0.595, -0.170)
    Pose_C_W1_3 = (-0.191, -0.806, -0.083, -0.087, -0.747, -0.170)
    Pose_E_W1_R_1 = (0.140, -0.643, -0.692, -0.075, -0.255, -1.357)
    Pose_E_W1_G_1 = (0.084, -0.759, -0.406, -0.047, -0.420, -1.459)
    Pose_E_W1_B_1 = (0.047, -0.917, -0.057, -0.035, -0.612, -1.510)

    # Pose for inserting elements into the case.
    Pose_E_Top_1_2 = (1.569, -0.365, -0.307, -0.018, -0.959, 0.039)

    # Pose for the workstation of the production line.
    Pose_C_1_2 = (1.579, -0.472, -0.306, -0.064, -0.876, 0.0)

# Robot2
class Ned2PosesR2(Enum):
    Home_Pose = (0, 0.299, -1.300, 0, 0, -0.004)

    # Pose for warehouses.
    Pose_E_W2_R_1 = (-0.400, -0.723, -0.398, -0.003, -0.457, 1.118)
    Pose_E_W2_R_2 = (-0.586, -0.815, -0.201, -0.110, -0.547, 1.009)
    Pose_E_W2_R_3 = (-0.736, -0.952, 0.069, -0.086, -0.673, 0.833)
    Pose_E_W2_G_1 = (-0.312, -0.912, -0.009, -0.003, -0.668, 1.195)
    Pose_E_W2_G_2 = (-0.479, -1.006, 0.184, -0.006, -0.760, 1.040)
    Pose_E_W2_G_3 = (-0.628, -1.153, 0.479, -0.011, -0.908, 0.896)
    Pose_E_W2_B_1 = (-0.246, -1.155, 0.476, 0.006, -0.915, 1.235)
    Pose_E_W2_B_2 = (-0.407, -1.256, 0.679, 0.000, -1.014, 1.103)
    Pose_E_W2_B_3 = (-0.570, -1.546, 1.250, -0.006, -1.291, 0.931)

    # Pose for inserting elements in the case.
    Pose_E_Top_2_3 = (-1.573, -0.797, 0.408, -0.007, -1.224, 0.115)
    Pose_E_Top_4_5 = (0.233, -0.811, 0.352, -0.009, -1.074, -1.288)

    # Pose for the workstation of the production line.
    Pose_C_2_3 = (-1.570, -0.812, 0.304, -0.007, -1.104, 0.118)
    Pose_C_3_4 = (0.327, -0.393, -0.448, -0.068, -0.745, -1.164)
    Pose_Center_RT = (0.277, -0.544, -0.212, -0.009, -0.850, 0.214)
    Pose_C_Help = (0.412, -0.888, -0.828, 0.288, 1.627, 0.058)

# Robot3
class Ned2PosesR3(Enum):
    Home_Pose = (0, 0.299, -1.300, 0, 0, -0.004)

    # Pose for warehouses.
    Pose_E_W2_R_1 = (0.327, -1.086, 0.423, 0.044, -0.845, -1.401)
    Pose_E_W2_R_2 = (0.484, -1.207, 0.661, 0.026, -0.960, -1.236)
    Pose_E_W2_R_3 = (0.621, -1.488, 1.211, 0.014, -1.223, -1.096)
    Pose_E_W2_G_1 = (0.371, -0.841, -0.066, 0.058, -0.595, -1.360)
    Pose_E_W2_G_2 = (0.551, -0.928, 0.106, 0.031, -0.675, -1.158)
    Pose_E_W2_G_3 = (0.700, -1.084, 0.417, 0.011, -0.828, -0.995)
    Pose_E_W2_B_1 = (0.429, -0.653, -0.447, 0.073, -0.398, -1.322)
    Pose_E_W2_B_2 = (0.639, -0.755, -0.242, 0.024, -0.497, -1.069)
    Pose_E_W2_B_3 = (0.794, -0.884, 0.017, 0.000, -0.627, -0.885)
    Pose_C_W3_1 = (-1.983, -0.685, -0.146, -0.059, -0.685, -1.929)
    Pose_C_W3_2 = (-1.719, -0.598, -0.299, -0.086, -0.630, -1.641)
    Pose_C_W3_3 = (-1.433, -0.597, -0.301, -0.102, -0.645, -1.348)

    # Poses for removing elements from the case.
    Pose_E1 = (-0.342, -0.717, -0.767, -0.374, 1.437, 0.028)
    Pose_E2 = (-0.342, -0.814, -0.730, -0.371, 1.489, 0.020)
    Pose_E3 = (-0.341, -0.905, -0.693, -0.369, 1.538, 0.013)

    # Additional poses for management operations of wrong assembly error.
    Pose_Over_Center_RT = (-0.346, -0.458, -0.823, -0.397, 1.247, 0.098)
    Pose_Exchange = (1.003, -1.000, -0.670, -0.603, 1.400, -0.088)
    Pose_V_Exchange = (1.274, -0.844, -0.058, -0.075, -0.646, -0.240)

    # Pose for inserting elements into the case.
    Pose_E_Top_Center_RT = (-0.164, -0.623, 0.111, -0.032, -1.038, -0.274)
    Pose_E_Top_4_5 = (-0.214, -0.318, -0.344, 0.0215, -0.897, -1.771)

    # Pose for the workstation of the production line.
    Pose_C_3_4 = (-0.130, -0.930, 0.497, -0.016, -1.181, -1.725)
    Pose_C_4_5 = (-0.214, -0.388, -0.391, 0.026, -0.780, -1.767)
    Pose_C_4_5_View = (-0.239, -0.388, -0.392, 0.057, -0.788, 1.288) 