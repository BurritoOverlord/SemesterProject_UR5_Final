"""
Launch this script to test simulation - Hardcoded robot displacements
"""


import time

import sim
from Simulation import Simulation_globalvariables as g
from Simulation import Simulation_gripper as grip
from Simulation import Simulation_moveL as mL

# Initialization

g.kFinal = 700/1000  # speed
g.connectionMessage(g.clientID)  # Printing out a successful/unsuccessful remote API connection message



# Functions


#Performs the operation of moving the bone-chicken basket from the conveyor to the deep-fryer
def function():
    initCups() #Spawning a non-dynamic basket to the 'batter and breading machine' platform

    grabCupFunc(g.clientID, 1, "")  # Grab Cup "" from its position
    placeCupFunc(g.clientID, 1, "") # Place Cup "" from its position

def initCups():
    errorCode, cupH = sim.simxGetObjectHandle(g.clientID, 'Cup', sim.simx_opmode_blocking)
    sim.simxSetObjectPosition(g.clientID, cupH, -1, g.basket_spawn, sim.simx_opmode_oneshot)

    grip.openGripperAtStart(g.clientID, g.j1, g.j2, g.p1, g.p2) #Opens gripper at the very beginning of moving the basket
    time.sleep(2)


def grabCupFunc(clientid, targetPosition, arrIndex):
    errorCode, cupH = sim.simxGetObjectHandle(g.clientID, 'Cup' + arrIndex, sim.simx_opmode_blocking)

    # Moving to position (on top of cup then down)
    mL.move_L(clientid, g.target, g.b1_return_pos, g.kFinal)
    time.sleep(0.5)
    mL.move_L(clientid, g.target, g.b2_return_pos, g.kFinal)
    time.sleep(2)
    sim.simxSetObjectParent(clientid, cupH, g.connector, True, sim.simx_opmode_blocking)

    # Closing gripper
    grip.closeGripper(clientid)
    time.sleep(1)

    # Moving object Up
    mL.move_L(clientid, g.target, g.b1_return_pos, g.kFinal)
    time.sleep(2)

def placeCupFunc(clientid, targetPosition, arrIndex):
    errorCode, cupH = sim.simxGetObjectHandle(g.clientID, 'Cup' + arrIndex, sim.simx_opmode_blocking)

    # Moving to position (on top of cup then down)
    mL.move_L(clientid, g.target, g.b3_return_pos, g.kFinal)
    time.sleep(0.5)
    mL.move_L(clientid, g.target, g.b4_return_pos, g.kFinal)
    time.sleep(2)
    sim.simxSetObjectParent(clientid, cupH, -1, True, sim.simx_opmode_blocking) #Resets Cup Parent to the Scene-"No longer grasping"


    # Open gripper
    grip.openGripper(clientid)
    time.sleep(1)

    # Moving object Up
    mL.move_L(clientid, g.target, g.b3_return_pos, g.kFinal)
    time.sleep(2)

    #Rotating Gripper
    # NOTE: CAN ONLY ROTATE GRIPPER IF THERE IS ALSO A TRANSLATION (X,Y,Z) - Need To Fix moveL function
    mL.move_L(clientid, g.target, g.b6_return_pos, g.kFinal)
    time.sleep(2)
    mL.move_L(clientid, g.target, g.b5_return_pos, g.kFinal)
    time.sleep(2)
    mL.move_L(clientid, g.target, g.b3_return_pos, g.kFinal)
    time.sleep(2)


############################# Python Script ###############################

def main():
    try:
        sim.simxStartSimulation(g.clientID, sim.simx_opmode_oneshot_wait)
        time.sleep(1)  # necessary for simulation to start properly

        ######### Main Functions ##########################################
        function()


    except Exception as e:
        print(e)
        sim.simxFinish(g.clientID)
    finally:
        sim.simxPauseSimulation(g.clientID, sim.simx_opmode_oneshot_wait)


if __name__ == "__main__":
    main()
