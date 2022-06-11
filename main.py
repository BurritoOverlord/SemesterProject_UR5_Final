import cv2
import imutils
from imutils.video import VideoStream

# My Modules
from comms_wrapper import *
import PathPlanning.simplePouring as sp
import move_robot
import marker_cup_detection
import sim
from Simulation import Simulation_globalvariables as sg

"SETTINGS AND VARIABLES ________________________________________________________________"
#Change this boolean to activate sim
simulation = False


# resolution the video capture will be resized to, smaller sizes can speed up detection
video_resolution = (640, 480)

vs = VideoStream(src=1,
                 resolution=video_resolution,
                 framerate=13,
                 meter_mode="backlit",
                 exposure_mode="auto",
                 shutter_speed=8900,
                 exposure_compensation=2,
                 rotation=0).start()
time.sleep(0.2)

"""FUNCTIONS _____________________________________________________________________________"""
# Change sp.simple_pouring for different control scheme
def pouring_algorithm(listCups_data):

    for i in listCups_data:
        if not i:
            print("no cups detected")
            return

    #Plot the Initial Cup Layout and initialize sim cup locations
    marker_cup_detection.cup_layout(listCups_data)
    print("Establishing initial cup layout")

    # Initialize cup layout in sim
    if simulation:
        marker_cup_detection.sim_init(listCups_data)
        print("Initializing Cup Sim layout")

    # TO-DO Make simple algorithm more robust, but I need more time, and it is not the focus of the project
    #Plan the simple algorithm
    sp.simple_pouring(listCups_data, simulation)


"""Main Script _____________________________________________________________________________"""

try:
    print("starting loop")

    #Initializing real robot
    if not simulation:
        move_robot.initialize_robot()

    while True:

        frame = vs.read()
        frame = imutils.resize(frame, width=video_resolution[0], height=video_resolution[1])

        # detected markers are in pixels
        detected_markers, field_corners = marker_cup_detection.findArUcoMarkers(frame)
        # convert to real values
        detected_markers = marker_cup_detection.getRealCoordinates(frame, field_corners, detected_markers)

        cv2.imshow('RobotCamera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vs.stream.release()
    cv2.destroyAllWindows()

    # Get Cup Coordinates in robot base reference frame
    listCups_data = marker_cup_detection.getCupCoordinates(detected_markers)

    #Apply pouring algorithm
    pouring_algorithm(listCups_data)
    print("exiting loop")

    if simulation:
        sim.simxPauseSimulation(sg.clientID, sim.simx_opmode_oneshot_wait)
    time.sleep(10)
    move_robot.stop_script()

except KeyboardInterrupt:
    print("closing robot connection")
    vs.stream.release()
    cv2.destroyAllWindows()
    move_robot.stop_script()
    sim.simxFinish(sg.clientID)
except:
    print("closing robot connection")
    print("Check Camera Connection")
    vs.stream.release()
    cv2.destroyAllWindows()
    move_robot.stop_script()
    sim.simxFinish(sg.clientID)
