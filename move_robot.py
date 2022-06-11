import copy

import numpy as np
import math
import rtde_control
import rtde_receive
from comms_wrapper import *
import sim

import globalvariables as g
import PathPlanning.geometry as geometry
from Simulation import Simulation_globalvariables as sg
from Simulation import Simulation_moveL as mL
"SETTINGS AND VARIABLES ________________________________________________________________"

# Setup robot with robot IP address
rtde_c = rtde_control.RTDEControlInterface("192.168.1.20")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.20")

# Setup Gripper
arduino = Arduino(descriptiveDeviceName="ARD", portName="COM5", baudrate=115200)
# Connects with the Arduino Nano and does the handshake to start void loop()
arduino.connect_and_handshake()




def initialize_robot():

    rtde_c.moveJ(np.deg2rad(g.home_J), g.VELOCITY, g.ACCELERATION)

    time.sleep(1)  # just a short wait to make sure everything is initialised

    # test gripper
    print("Testing Gripper")
    arduino.send_message("grab")
    time.sleep(3)
    arduino.send_message("release")
    time.sleep(3)


def grab_cup(cX, cY, angle, simulation):

    temp_L = copy.copy(g.origin_L)

    pX = copy.copy(cX)
    pY = copy.copy(cY) - g.grip_dy
    pX, pY = geometry.rotate(cX, cY, pX, pY, angle)

    temp_L[0] = pX
    temp_L[1] = pY

    #Get desired angle
    rot = geometry.get_angle((pX, pY), (cX , cY))

    if simulation:
        returnCode, orient = sim.simxGetObjectOrientation(sg.clientID, sg.target, -1, sim.simx_opmode_buffer)
        for i in range(3):
            orient[i] = float(orient[i])
        sim_pose = [pX, pY, temp_L[2], 0, 0, orient[2]-rot]
        mL.move_L(sg.clientID, sg.target, sim_pose, sg.kFinal)
        time.sleep(0.5)
        # go down and grab and go back up
        sim_pose = [pX, pY, temp_L[2] - 3 * g.grab_height_offset, 0, 0, orient[2]-rot]
        mL.move_L(sg.clientID, sg.target, sim_pose, sg.kFinal)

    else:
        rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)
        # Get current Joint position in Joint space
        center_J = rtde_r.getActualQ()
        center_J[5] -= rot
        rtde_c.moveJ(center_J)

        Cartesian_positions = rtde_r.getActualTCPPose()
        temp_L = copy.copy(Cartesian_positions)
        # go down and grab and go back up
        temp_L[2] = g.origin_L[2] - 3 * g.grab_height_offset #was calculated by hand
        rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)
        arduino.send_message("grab")
        time.sleep(7)

        # go down and grab and go back up
        temp_L[2] = g.origin_L[2]
        rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    return

def place_cup(cX, cY, simulation):
    temp_L = copy.copy(g.origin_L)

    #print("place Cup")
    temp_L[0] = cX + g.grip_dy
    temp_L[1] = cY

    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    
    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    #have a 90 degree rotation
    if center_J[5] > 0:
        center_J[5] -= math.pi/2
    else:
        center_J[5] += 3* math.pi/2

    rtde_c.moveJ(center_J)

    # go down and grab and go back up
    temp_L = rtde_r.getActualTCPPose()
    temp_L[2] = g.origin_L[2] - 3 * g.grab_height_offset
    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)
    arduino.send_message("release")
    time.sleep(7)

    # go down and grab and go back up
    temp_L[2] = g.origin_L[2]
    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    return


def place_cupP(cX, cY, simulation):
    temp_L = copy.copy(g.origin_L)

    #print("place Cup")
    temp_L[0] = cX
    temp_L[1] = cY - g.grip_dy

    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    # go down and grab and go back up
    temp_L = rtde_r.getActualTCPPose()
    temp_L[2] = g.origin_L[2] - 3 * g.grab_height_offset
    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)
    arduino.send_message("release")
    time.sleep(7)

    # go down and grab and go back up
    temp_L[2] = g.origin_L[2]
    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    return


def grab_cupP(cX, cY, angle, simulation):

    temp_L = copy.copy(g.origin_L)

    #####################################################################
    # Go to Desired Position
    #print("Go to desired position")

    pX = copy.copy(cX)
    pY = copy.copy(cY) - g.grip_dy

    pX, pY = geometry.rotate(cX, cY, pX, pY, angle)


    temp_L[0] = pX
    temp_L[1] = pY

    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    #####################################################################
    # Rotate Wrist Correspondingly
    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    #Get desired angle
    rot = angle
    print(rot)

    print(center_J[5])

    center_J[5] -= rot
    rtde_c.moveJ(center_J)

    Cartesian_positions = rtde_r.getActualTCPPose()
    temp_L = copy.copy(Cartesian_positions)
    # go down and grab and go back up
    temp_L[2] = g.origin_L[2] - 3 * g.grab_height_offset #was calculated by hand
    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)
    arduino.send_message("grab")
    time.sleep(7)

    # go down and grab and go back up
    temp_L[2] = g.origin_L[2]
    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    return

def pouring(cX, cY, simulation):
    temp_L = copy.copy(g.origin_L)

    # print("place Cup")
    temp_L[0] = cX + g.grip_dy + g.pouring_dx
    temp_L[1] = cY

    rtde_c.moveL(temp_L, g.VELOCITY, g.ACCELERATION)

    # Get current Joint position in Joint space
    center_J = rtde_r.getActualQ()
    # have a 90 degree rotation
    if center_J[5] > 0:
        center_J[5] -= math.pi / 2
    else:
        center_J[5] += 3 * math.pi / 2

    rtde_c.moveJ(center_J)

    Cartesian_positions = rtde_r.getActualTCPPose()
    temp_L = copy.copy(Cartesian_positions)

    #Move to Pouring Position 1
    for i in range(len(temp_L)):
        temp_L[i] -= g.pouring_1_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)

    #Move to Pouring Position 2
    for i in range(len(temp_L)):
        temp_L[i] += g.pouring_2_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)

    #Move to Pouring Position 3
    for i in range(len(temp_L)):
        temp_L[i] += g.pouring_3_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)

    #Move to Pouring Position 4
    for i in range(len(temp_L)):
        temp_L[i] += g.pouring_4_L[i]
    print(temp_L)
    rtde_c.moveL(temp_L)
    return

def stop_script():
    # Stop the RTDE control script
    rtde_c.stopScript()
    return