import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import time

# my modules
import PathPlanning.geometry as geometry
import globalvariables as g
from Simulation import Simulation_gripper as grip
from Simulation import Simulation_globalvariables as sg
import sim


def findArUcoMarkers(frame, markerSize=4, totalMarkers=50, draw=True):
    # start by converting to gray scale
    img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(img_gray, arucoDict, parameters=arucoParam,
                                              cameraMatrix=g.calibrationMatrix, distCoeff=g.distortionCoefficient)

    # Store the "center coordinates" of all marker in detected objects
    # in order from the upper left in the clockwise direction.
    detected_markers = list()  # Storage destination of center coordinates for all detected markers with their ID
    field_corners = np.empty((4, 2))  # detect the corner values of the "play field" ->marker ID: 0-3
    # verify *at least* one ArUco marker was detected
    if len(bbox) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(bbox, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            eX = int((topLeft[0] + topRight[0]) / 2.0)
            eY = int((topLeft[1] + topRight[1]) / 2.0)

            if markerID == 0:
                field_corners[markerID] = bottomRight
            if markerID == 1:
                field_corners[markerID] = bottomLeft
            if markerID == 2:
                field_corners[markerID] = topLeft
            if markerID == 3:
                field_corners[markerID] = topRight

            detected_markers.append(dict(id=markerID, cx=cX, cy=cY, ex=eX, ey=eY))

            """
            if draw:
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            """

    if draw:
        # Draw bbox
        aruco.drawDetectedMarkers(frame, bbox, ids)

    return [detected_markers, field_corners]


def getRealCoordinates(frame, field_corners, detected_markers, p_width=g.video_resolution[0],
                       p_height=g.video_resolution[1]):
    # resize frame into the "playing field" coordinate space
    # width, height = (500, 500)  # Image size after transformation in pixels
    field_corners_vect = np.float32([field_corners[0], field_corners[1], field_corners[2], field_corners[3]])
    true_coordinates = np.float32([[0, 0], [p_width, 0], [p_width, p_height], [0, p_height]])
    trans_mat = cv2.getPerspectiveTransform(field_corners_vect, true_coordinates)

    # show playing field
    img_trans = cv2.warpPerspective(frame, trans_mat, (p_width, p_height))
    cv2.imshow("new frame", img_trans)

    # print(detected_markers)
    ##########################################################################q##################
    ############################################################################################
    detected_true_coordinates = np.empty((len(detected_markers), 3))  # create matrix of coordinates
    for i, obj in enumerate(detected_markers, 0):
        # apply the transform matrix to each vector of center coordinates
        detected_true_coordinates[i] = np.dot(trans_mat, (obj["cx"], obj["cy"], 1))
        # change back the values in the detected markers list to have the real coordinates
        obj["cx"] = - detected_true_coordinates[i][0] * g.r_width / p_width  # x axis is upside down
        obj["cy"] = detected_true_coordinates[i][1] * g.r_height / p_height

        # apply the transform matrix to each vector of top left coordinates
        detected_true_coordinates[i] = np.dot(trans_mat, (obj["ex"], obj["ey"], 1))
        # change back the values in the detected markers list to have the real coordinates
        obj["ex"] = - detected_true_coordinates[i][0] * g.r_width / p_width  # x axis is upside down
        obj["ey"] = detected_true_coordinates[i][1] * g.r_height / p_height

        # Print real coordinates to calibrate the offset manually
        """
        if obj["id"] > 3:
            print(f'■ REAL COORDINATES: Detected markerID{obj["id"]:>3.0f} Center position X={obj["cx"]:>3.0f}mm'
                  f' Y={obj["cy"]:>3.0f}mm ')
        """
    return detected_markers


def getCupCoordinates(detected_markers):
    """
    :param detected_markers:
    :return:
    """

    # Create list for Cups
    listCups_data = []

    # loop over the detected markers to calculate the cup center points and angle
    # For each specific marker ID do move the robot
    for i, obj in enumerate(detected_markers, 0):

        # ignore the reference ArUco markers and store all important values in cupList
        # convert all values to meters
        if obj["id"] > 3:
            # Get cup edge en center from ArUco markers - in Meters!
            mX = g.origin_L2[0] + obj["cx"] / 1000
            mY = g.origin_L2[1] + obj["cy"] / 1000

            tX = g.origin_L2[0] + obj["ex"] / 1000
            tY = g.origin_L2[1] + obj["ey"] / 1000

            dist1 = g.rCups2 + g.dMtCe
            dist2 = 2 * g.rCups2 + g.dMtCe

            [cX, cY] = geometry.segmentExtension((mX, mY), (tX, tY), dist1)
            [eX, eY] = geometry.segmentExtension((mX, mY), (tX, tY), dist2)

            listCups_data.append(dict(id=obj["id"], center=[cX, cY], edge=[eX, eY]))

    return listCups_data


def cup_layout(listCups):

    figure, ax = plt.subplots()
    # change default range so that new circles will work
    ax.set_xlim((g.xMin, g.xMax))
    ax.set_ylim((0, g.yMax))

    for i, obj in enumerate(listCups, 0):
        # plot circles
        circle = plt.Circle(obj["center"], g.rCups2, color='red', ec='black')
        plt.gcf().gca().add_patch(circle)

        # plot cup edge dots
        circle = plt.Circle(obj["edge"], g.rCups2 / 5, color='blue', ec='black')
        plt.gcf().gca().add_patch(circle)

    plt.title('Cup Layout 2')
    ax.set_aspect(1)
    plt.show()

    return

def sim_init(listCups):

    sim.simxStartSimulation(sg.clientID, sim.simx_opmode_oneshot_wait)
    time.sleep(1)  # necessary for simulation to start properly

    for i, obj in enumerate(listCups, 0):
        arrIndex = str(obj["id"])
        cup_spawn = [obj["center"][0], obj["center"][1], g.sim_cup_height]
        errorCode, cupH = sim.simxGetObjectHandle(sg.clientID, 'Cup' + arrIndex, sim.simx_opmode_blocking)
        sim.simxSetObjectPosition(sg.clientID, cupH, -1, cup_spawn, sim.simx_opmode_oneshot)

    grip.openGripperAtStart(sg.clientID, sg.j1, sg.j2, sg.p1, sg.p2)  # Opens gripper at the very beginning of moving the basket
    time.sleep(2)
