import numpy as np

"MAIN GLOBAL VARIABLES ________________________________________________________________"
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

# Gripper Offset parameter - To change if we have new gripper
grip_dz = 168.4405 / 1000  # meters
grip_dy = 0.12  # meters

pouring_dx = 0.08

grab_height_offset = 0.075  # grab height offset to cup from origin_L

origin_L = [0.305, 0.506, 0.31504431455331383 + 2 * grab_height_offset, -3.13713023885791, 0.08284771453405795,
            -0.009878696005977336]

pouring_1_L = [0.1329460444039362, 0.001992027271668584, 0.028232181258689593, 0.4172834491949249, 0.5468231534718957,
               -0.7511273037641358]
pouring_2_L = [-0.1646497520298684, 0.017626478234580634, -0.056484304018509124, -0.7349117459413628,
               -0.3580076965193224, 0.4108333718214877]
pouring_3_L = [-0.07186949898256362, -0.023014226767514745, -0.08788628407032015, -0.352989202528292,
               -0.4261227544800278, 0.33786829085844206]
pouring_4_L = [-0.014978186806280838, -0.010535790024139513, -0.02171575440950893, -0.17231163199225163,
               -0.20942507249648468, 0.07274833908588763]


"simple Pouring global variables ________________________________________________________________"
rMax = 0.156 + 0.05  # maximum radius to check if cups obstruct the gripper (usually go for 1.3 times the gripper dy
dCups = 0.19 # distance between each cup in the end position
xLoc = -0.11
yLoc = 0.235
rCups = 0.055 * 1.5

"Marker Cup detection VARIABLES ________________________________________________________________"

video_resolution = (640, 480)  # resolution the video capture will be resized to, smaller sizes can speed up detection

# Environment width and height
r_width = 290
r_height = 172

# Calibration Parameters
calibrationMatrix = np.load('Cam_Calibration/calibration_matrix.npy')
distortionCoefficient = np.load('Cam_Calibration/distortion_coefficients.npy')

# ArUco marker ref origin in the coordinates of the UR5 base ref frame
origin_L2 = [0.305, 0.506, 0.31504431455331383, -3.13713023885791, 0.08284771453405795,
            -0.009878696005977336]

# misc. hyperparameters
rCups2 = 0.055  # Cup radius in cm
dMtCe = 0.025  # distance from marker center to cup edge in cm

# misc. Terrain Field to be defined at the beginning
xMax = 0.5
yMax = 1
xMin = -0.2

"Move robot global variables ________________________________________________________________"
home_J = [-298.22, -66.90, -113.00, -89.74, 90.11, -24.63]


"Sim global variables ________________________________________________________________"
sim_cup_height = 0.765