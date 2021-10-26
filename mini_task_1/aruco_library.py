#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
    # function to detect ArUco markers in the image using ArUco library
    # argument: img is the test image
    # return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    # 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    # 				{0: array([[315, 163],
    #							[319, 263],
    #							[219, 267],
    #							[215,167]], dtype=float32)}
    Detected_ArUco_markers = {}
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if len(corners):
    	for i in range(len(corners)):
            temp_1 = corners[i][0]
            temp_2 = ids[i][0]
            Detected_ArUco_markers[temp_2] = temp_1
    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
    ## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    ## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
    ##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the
    ##			function should return: {1: 120 , 2: 164}

    orient_centre_list = []
    for i in Detected_ArUco_markers:
        x = int((Detected_ArUco_markers[i][0][0] + Detected_ArUco_markers[i][1][0])/2)
        y = int((Detected_ArUco_markers[i][0][1] + Detected_ArUco_markers[i][1][1])/2)
        orient_centre_list.append([x, y])

    key_list = Detected_ArUco_markers.keys()
    centre_list = []
    for key in key_list:
        dict_entry = Detected_ArUco_markers[key]
        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
        centre_list.append([int(x / 4) for x in centre])

    counter = 0
    ArUco_marker_angles = {}
    for i in Detected_ArUco_markers:
        deltaX = Detected_ArUco_markers[i][1][0] - Detected_ArUco_markers[i][0][0]
        deltaY = Detected_ArUco_markers[i][0][1] - Detected_ArUco_markers[i][1][1]

        angle = math.degrees(math.atan2(deltaY, deltaX)) 
        if(angle < 0 and abs(angle) > 90): angle += 180
        elif(angle > 0 and angle > 90): angle -= 90
        elif(angle < 0 and abs(angle) < 90): angle += 90

        ArUco_marker_angles[i] = int(angle)
        counter += 1

    return ArUco_marker_angles  ## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles):
    # function to mark ArUco in the test image as per the instructions given in problem statement
    # arguments: img is the test image
    #			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    #			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    # return: image namely img after marking the aruco as per the instruction given in problem statement

    # enter your code here
    orient_centre_list = []
    for i in Detected_ArUco_markers:
        x = int((Detected_ArUco_markers[i][0][0] + Detected_ArUco_markers[i][1][0])/2)
        y = int((Detected_ArUco_markers[i][0][1] + Detected_ArUco_markers[i][1][1])/2)
        orient_centre_list.append(tuple([x, y]))

    key_list = Detected_ArUco_markers.keys()
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    counter = 0    
    for key in key_list:
        dict_entry = Detected_ArUco_markers[key]
        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
        centre[:] = [int(x / 4) for x in centre]
        centre = tuple(centre)
        orient_centre = orient_centre_list[counter]

        cv2.circle(img, centre, 1, (0,0,255), 8)
        # cv2.circle(img, orient_centre, 1, (0,0,255), 8)

        cv2.circle(img, tuple(dict_entry[0]), 1, (125,125,125), 8) # top-left
        cv2.circle(img, tuple(dict_entry[1]), 1, (0,255,0), 8) # top-right
        cv2.circle(img, tuple(dict_entry[2]), 1, (180,105,255), 8) # bottom-right
        cv2.circle(img, tuple(dict_entry[3]), 1, (255,255,255), 8) # bottom-left

        
        cv2.line(img, orient_centre, centre, (255,0,0), 4)
        cv2.putText(img, str(key), (int(centre[0] + 20), int(centre[1])), font, 1, (0,0,255), 2, cv2.LINE_AA)  

        cv2.putText(img, str(ArUco_marker_angles[key]), (int(centre[0] - 60), int(centre[1] + 20)), font, 1, (0,255,0), 2, cv2.LINE_AA)  
        counter += 1  
    return img
