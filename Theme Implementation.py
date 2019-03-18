'''
* Team Id : 1079
* Author List : Sourabh Khemka
* Filename: path_traversal.py
* Theme: Thirsty Crow (TC)
* Functions: detect_markers(img), getCameraMatrix(), detect_positions(frame, flag, flag_2), dist(a, b)
*			 calculate_euclidean_distances(), calculate_slope(a, b), axis_change(direction), orientation_finder(),
*			 traversal_function(target_location, target_axis), init_gl(), resize(w,h), drawGLScene(), draw_background(img),
*			 init_object_texture(image_filepath), overlay(img, ar_list, ar_id), check(), rendering_main()
* Global Variables: pickup_pos_list, crow_robot, deposition_zone, pickup_array, frame, imgpts, camera_matrix, dist_coeff, _, 
*					ser, slope_axis_along_arm, cap, axis, texture_object, texture_background, INVERSE_MATRIX, crow, 
*					pitcher_high, pitcher_low, pitcher_medium, pebble_diminished, pebble_large, pickup_data
'''

import numpy as np
import cv2
import cv2.aruco as aruco
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import pygame
from objloader import *

# serial library is imported to enable serial communication between the two X-Bee modules
import serial

# time library is imported to help add sleep statements in the code
import time


# VARIABLES DEFINED SEPARATELY (GLOBALLY) AS THEIR VALUES HAVE TO BE MODIFIED INSIDE OTHER FUNCTIONS

# Data corresponding to aruco ids ranging from 1-9 in the variable aruco_list are stored in pickup_pos_list
# It will also contain the Euclidean distance from the robot to help identify nearest pick-up zone from the bot
pickup_pos_list = []

# Contains the data in aruco_list corresponding to aruco id 10
crow_robot = 0;

# Contains the data in aruco_list corresponding to aruco id 0
deposition_zone = 0;

# Stores the aruco_list returned by the function detect_markers
pickup_array = 0;

# variable to store the image read using the camera
# Defined separately as it has to be modified by some functions
frame = 0;

# Contains the information of Serial communication
ser = 0;

# This variable has the slope of line along axis of pickup arm
slope_axis_along_arm = 0;

# Video Capturing object
cap = cv2.VideoCapture(1)

# Axis from which robot starts (1-1 in case of both start-1 and start-2)
axis = 1;

flag_for_360 = True

texture_object = None
texture_background = None

camera_matrix = None
dist_coeff = None
type = "LARGE"

INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
							[-1.0,-1.0,-1.0,-1.0],
							[-1.0,-1.0,-1.0,-1.0],
							[ 1.0, 1.0, 1.0, 1.0]])

crow = None
pitcher_high = None
pitcher_medium = None
pitcher_low = None
pitcher_full = None
pebble_large = None
pebble_diminished = None

temp_wait = 0

# This dictionary stores the aruco id of pickup/deposition zone as Key and wether operation done or not (False/1/2/True) as value 
pickup_data = {}



'''
* Function Name: detect_markers(img)
* Input: img
* Output: aruco_list
* Logic: detect the aruco markers and return the aruco list which contains all ARUCO IDs, coo-rdinate of their center
*        rotation vector and translation vector corresponding to all IDs
* Example Call: detect_markers(img)
'''

def detect_markers(img):                                             #FUNCTION detect_markers() TO DETECT THE ARUCO-MARKERS AND RETURN THE
    markerLength = 100                                                                                  #FUNCTION REQUIRED BY drawing FUNCTIONS
    aruco_list = []

    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)                                             #CONVERTING THE IMAGE TO GRAYSCALE
    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)                                       #DEFINING ARUCO DICTIONARY OF 250 5X5 MARKERS

    parameters = aruco.DetectorParameters_create()                                              #PRE-SET PARAMETERS FOR MARKER DETECTION 
    
    corners,ids,_ = aruco.detectMarkers(img_gray,aruco_dict,parameters=parameters)              #FUNCTION TO DETECT MARKERS. RETURNS ID OF EACH MARKER AND THEIR CORNERS

    rvec,tvec,_ = aruco.estimatePoseSingleMarkers(corners,markerLength,camera_matrix,dist_coeff)#OPENCV FUNCTION TO RETURN ROTATIONAL AND TRANS. VECTORS FOR THE MARKERS

    count = 0                                                                                   #COUNTER VARIABLE TO ITERATE OVER THE LIST OF ids AND corners RETURNED 
                                                                                                        #BY aruco.detectMarkers()
    
    

    for corner in corners:                                                                      #CORNER VARIABLE ITERATES OVER VALUES FOR DIFFERENT MARKERS
        
        for i in corner:                                                                        #for LOOP TO ENTER THE list-of-Lists [[],[],[]]
            
            mid_x = (i[0][0]+i[1][0]+i[2][0]+i[3][0])/4.0
            mid_y = (i[0][1]+i[1][1]+i[2][1]+i[3][1])/4.0
            mid_x = int(mid_x); mid_y = int(mid_y)
            aruco_center = (mid_x,mid_y)
            
            #APPEND THE LIST aruco_list = [] , ALSO ARRANGING THE rvec and tvec IN THE PROPER FORMAT AS REQUIRED BY THE FILE TestSuite.py
            aruco_list.append((ids[count][0],aruco_center,np.array([[rvec[count][0]]]),np.array([[tvec[count][0]]])))            
            
            count += 1



    #CODE TO LABLE ARUCO_ID OVER THE MARKER AND DRAW A SQUARE BOUNDARY AROUND IT
    for x in aruco_list:

            cv2.putText(img, str('id=')+str(x[0]), x[1], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 4)   #PUTS A TEXT(ARUCO ID) OVER THE MARKER

            #COMMENTS FOR THE FOLLOWING LINE CAN BE REFERENCED FROM THE FILE detect.py COMPLETED IN TASK 1.1 OF THE COMPETETION
            rvec, tvec = x[2][0][0],x[3][0][0]

            m = 50
            pts = np.float32([[m,m,0],[-m,m,0],[-m,-m,0],[m,-m,0]])
            imgpts,_ = cv2.projectPoints(pts,rvec,tvec,camera_matrix,dist_coeff)

            for i in imgpts:
                      for j in i:
                              j[0] = int(j[0]); j[1] = int(j[1]);

            contours = np.array([ tuple(imgpts[0][0]) ,
                                 tuple(imgpts[1][0]),
                                    tuple(imgpts[2][0]),
                                 tuple(imgpts[3][0]) ], np.int32)
            contours = contours.reshape((-1,1,2))

            img = cv2.polylines(img,[contours],True,(0,255,0),2,cv2.LINE_AA)

    return aruco_list                                                                           #RETURNS THE REQUIRED LIST aruco_list



'''
* Function Name: getCameraMatrix()
* Input: NONE
* Output: NONE
* Logic: imports the file generated after camera calliberation and stores the camera matrix and distortion coeffecient in global variables
* Example Call: getCameraMatrix()
'''

def getCameraMatrix():

	# VALUES OF THESE GLOBAL VARIABLES ARE MEANT TO BE MODIIED IN THIS FUNCTION HENCE USING global
    global camera_matrix, dist_coeff
    with np.load('System.npz') as X:
            camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]






'''
* Function Name: detect_positions(frame, flag, flag_2)
* Input: frame, flag, flag_2
* Output: NONE
* Logic: Calls the detect_markers() function and stores the values in pickup_array for pick-up locations, 
*        crow robot and deposition zone in their corresponding variables
* Example Call: detect_positions(frame, flag, flag_2)
'''

def detect_positions(frame, flag, flag_2):
	
	# VALUES OF THESE GLOBAL VARIABLES ARE MEANT TO BE MODIIED IN THIS FUNCTION HENCE USING global
	global pickup_pos_list, crow_robot, deposition_zone, pickup_array;
	
	#  STORING THE ARUCO LIST WHICH CONTAINS THE IDs AND DATA OF ALL MARKERS IN pickup_array
	pickup_array = detect_markers(frame)
	len_array = len(pickup_array)
		
	for i in range(len_array):

		# IF THE MARKER ID BELONGS TO [1,9] , IT IS PICK-UP LOCATION HENCE DATA ALONG WITH ID'S STORED IN pickup_pos_list
		if(flag == True):
			if pickup_array[i][0] in range(1,10):	
				pickup_pos_list.append([pickup_array[i][0], pickup_array[i][1], pickup_array[i][2], pickup_array[i][3]])
			
		# IF MARKER ID IS 10 IT IS THE CROW-ROBOT HENCE STORED IN THE VARIABLE crow_robot
		if pickup_array[i][0] == 10:
			crow_robot = ([pickup_array[i][0], pickup_array[i][1], pickup_array[i][2], pickup_array[i][3]])
	
		# IF ID IS 0, IT IS THE DEPOSITION ZONE, HENCE DATA IS STORED IN deposition_zone
		if flag_2 == True:
			if pickup_array[i][0] == 0:
				deposition_zone = ([pickup_array[i][0], pickup_array[i][1], pickup_array[i][2], pickup_array[i][3]])


		
'''
* Function Name: dist(a, b)
* Input: a, b
* Output: euclidean_distance
* Logic: Calculates the euclidean distance between the two points provided as input a and b
* Example Call: dist(a, b)
'''
def dist(a, b):
	# DISTANCE CALCULATED USING THE DISTANCE FORMULA
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


	
'''
* Function Name: calculate_euclidean_distances()
* Input: NONE
* Output: NONE
* Logic: Calculates the Euclidean distance of pick up locations from the crow robot and appends it in the pickup_pos_list
		 The function then sorts the list pickup_pos_list with the euclidean distance as KEY
* Example Call: calculate_euclidean_distances()
'''

def calculate_euclidean_distances():

	global pickup_pos_list;
	
	for i in range(len(pickup_pos_list)):

		# if THE FUNCTION calculate_euclidean_distances() IS BEING RECALLED THEN THE VALUES WILL BE REFRESHED else THEY WILL BE APPENDED
		if(len(pickup_pos_list[i]) == 5):
			pickup_pos_list[i][4] = (dist(pickup_pos_list[i][1], crow_robot[1]))

		else:
			pickup_pos_list[i].append(dist(pickup_pos_list[i][1], crow_robot[1]))
	
	# USING lambda FUNCTION TO GET THE KEY FOR SORTING (WHICH IS EUCLIDEAN DISTANCE)
	sorting_key = lambda pickup_pos_list : pickup_pos_list[4]
	pickup_pos_list.sort(key = sorting_key)
	
	
	
'''
* Function Name: calculate_slope(a, b)
* Input: a, b
* Output: slope
* Logic: calulates the slope of line connecting the two points provided as input a and b
* Example Call: calculate_slope(a, b)
'''

def calculate_slope(a, b):
	# MULTIPLY ORDINATES WITH -1 TO CONVERT THE CO-ORDINATE SYSTEM USED BY OPENCV TO CARTESIAN CO-ORDINATE SYSTEM
	a = (a[0], a[1]*-1); b = (b[0], b[1]*-1);
	
	# CALCULATION OF SLOPE 
	slope = ((b[1] - a[1])/(b[0] - a[0]))
	return slope



'''
* Function Name: axis_change(direction)
* Input: direction
* Output: NONE 
* Logic: Applies the logic to store the axis with which the robot is currently aligned to
* Example Call: axis_change(direction)
'''
def axis_change(direction):
	global axis;
	
	# IT IS OBSERVABLE THAT THERE IS A DEFINITE PATTER IN WHICH THE AXIS OF ROBOT IS CHANGING
	
	if(direction == "LEFT"):
		if(axis == 1):
			axis = 3
		elif(axis == 2):
			axis = 1
		elif(axis == 3):
			axis = 2
	elif(direction == "RIGHT"):
		if(axis == 1):
			axis = 2
		elif(axis == 2):
			axis = 3
		elif(axis == 3):
			axis = 1
	print(str("AXIS :: ") + str(axis)); print("	");
		

'''
* Function Name: orientation_finder()
* Input: NONE
* Output: NONE 
* Logic: Function calculates slope of axis of aruco markers at pickup/deposition zone and decode their orientation 
*		 (1-1, 2-2 or 3-3)
* Example Call: orientation_finder()
'''
def orientation_finder():
	global pickup_pos_list,deposition_zone;
	
	for i in range(len(pickup_pos_list)):
		
		rvec = pickup_pos_list[i][2]; tvec = pickup_pos_list[i][3]
		markerLength = 100; m = markerLength/2
		pts = np.float32([[0, m, 0]])
		point, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
		
		slope = ((-1*point[0][0][1]) - (-1*pickup_pos_list[i][1][1])) / (point[0][0][0] - pickup_pos_list[i][1][0])
		angle = math.atan(slope) * 57.2958
		
		if(angle > 30):
			pickup_pos_list[i].append(3)
		elif(angle < -30):
			pickup_pos_list[i].append(2)
		else:
			pickup_pos_list[i].append(1)
		
	rvec = deposition_zone[2]; tvec = deposition_zone[3]
	markerLength = 100; m = markerLength/2
	pts = np.float32([[0, m, 0]])
	point, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
	
	slope = ((-1*point[0][0][1]) - (-1*deposition_zone[1][1])) / (point[0][0][0] - deposition_zone[1][0])
	angle = math.atan(slope) * 57.2958
	
	if(angle > 30):
		deposition_zone.append(3)
	elif(angle < -30):
		deposition_zone.append(2)
	else:
		deposition_zone.append(1)
		
		
		
'''
* Function Name: traversal_function(target_location, target_axis)
* Input: target_location, target_axis
* Output: NONE 
* Logic: Function takes the target location i-e marker's location where bot has to reach and the axis of marker over there as input
*			Function then implements the logic for arena traversal (as described in task 3) to send the robot the required instruction
*			to implement arena traversal
* Example Call: traversal_function(target_location, target_axis)
'''

def traversal_function(target_location, target_axis):
	
	global frame, slope_axis_along_arm, axis,flag_for_360, temp_wait;
	loop_counter = 0
	
	rvec = crow_robot[2]; tvec = crow_robot[3]
	markerLength = 100; m = markerLength/2
	pts = np.float32([[0, 5.5*m, 0]])
	point, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
	point[0][0][1] += 0 #30
	
	# while loop terminates when robot nears the target location. The point at distance of 5.5 times the marker length 
	# is taken as reference to check if bot has reached. When the distance between the reference and center of target reduces to
	# less than 35 the condition .... > 35 becomes False and the loop terminates 
	while( dist(point[0][0], target_location)>35): 
		
		_, frame = cap.read()
		detect_positions(frame, False, False)

		# DEFINING THE VARIABLES REQUIRED TO CONVERT THE POINTS IN MARKER'S FRAME TO WORLD FRAME
		rvec_robot = crow_robot[2]; tvec_robot = crow_robot[3]
		markerLength = 100; m = markerLength/2
		pts = np.float32([[0, m, 0]])
		imgpts, _ = cv2.projectPoints(pts, rvec_robot, tvec_robot, camera_matrix, dist_coeff)
		
		# CALCULATING THE SLOPE OF THE PERPENDICAULAR AXIS OF ROBOT (WHICH PASSES THROUGH THE FRONT FACE OF BOT) AND THE 
		# LINE JOINING CENTER OF CROW-ROBOT TO CENTER OF NEAREST PICK-UP ZONE
		
		# m1 is the slope of line along the pickup arm's axis # m2 is slope of line joining robots center with center of target location
		# m3 is slope of line perpendicular to m1
		m1 = slope_axis_along_arm = calculate_slope(imgpts[0][0], crow_robot[1])
		m2 = slope_line_bw_target_and_robot = calculate_slope(crow_robot[1], target_location)
		m3 = -1/m1
		
		# EQUATION OF THE LINE ::  mx - y + b - ma = 0 where (a,b) is the center of robot and m is slope (slope_perp_axis_along_arm)
		# val_1 is the value of line when the coordinates of point at top and middle of aruco marker is put in the equation
		# and val_2 is value corresponding do coordinates of target location's center
		val_1 = (m3*imgpts[0][0][0]) - (-1*imgpts[0][0][1]) + (-1*crow_robot[1][1]) - (m3*crow_robot[1][0]) 
		val_2 = (m3*target_location[0]) - (-1*target_location[1]) + (-1*crow_robot[1][1]) - (m3*crow_robot[1][0])
		
		
		# IT IS OBSERVED THAT SIGN OF THE VALUE OF DIFFERENCE OF SLOPES DEPENDS ON WETHER THE BOT HAS TO TAKE THE RIGHT/LEFT PATH
		# DEPENDING ON HOW THE TWO LINES ARE INCLINED WITH EACH OTHER CODE DETERMINES WEHTER BOT HAS TO TURN LEFT OR RIGHT SO THAT
		# IT COMES ON THE REQUIRED PATH AND SENDS THE CHARACTER TO ROBOT OVER UART COMMUNICATION
		
		# L is for LEFT # R is for RIGHT # check function is called to establish coordinatio b/w python and C code
		if(val_1*val_2 < 0 and loop_counter == 0):
			print("COMMAND :: ROTATE 360")
			ser.write("O".encode())
			check();
		elif(m1 * m2 > 0 or m1*m2==0 or (imgpts[0][0][1]-crow_robot[1][1])*(target_location[1]-crow_robot[1][1]) < 0 ):
			if(m1 - m2 < 0):
				axis_change("LEFT");	#print("LEFT")
				ser.write("L".encode())
				check();
			elif(m1 - m2 > 0):
				axis_change("RIGHT");	#print("RIGHT")
				ser.write("R".encode())
				check();
			else:
				axis_change("RIGHT");	#print("RIGHT")
				ser.write("R".encode())
				check();
		else:
			if(m1 - m2 < 0):
				axis_change("RIGHT");	#print("RIGHT")
				ser.write("R".encode())
				check();
			elif(m1 - m2 > 0):
				axis_change("LEFT");	#print("LEFT")
				ser.write("L".encode())
				check();
			else:
				axis_change("RIGHT");	#print("RIGHT")
				ser.write("R".encode())
				check();
		if(loop_counter == 0):
			loop_counter = 1
				
		_, frame = cap.read()
		detect_positions(frame, False, False)
		rvec = crow_robot[2]; tvec = crow_robot[3]
		markerLength = 100; m = markerLength/2
		pts = np.float32([[0, 5.5*m, 0]])
		point, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
		point[0][0][1] += 0#30
	
	print("LOOP TERMINATED...")
	# N :: robot is allready oriented with required axis
	# K :: move to right node to align robot
	# M :: move to left node to align robot
	# axis_change() is called to refresh the current axis to which robot is oriented
	# sending K or M rotates the robot by 120 degs so there are two axis changes
	
	if(axis == target_axis):
		ser.write("N".encode()); check();
	elif(axis == 1):
		if(target_axis == 2):
			ser.write("L".encode()); check(); 	ser.write("R".encode()); check();
			ser.write("K".encode()); check(); 	axis_change("LEFT"); axis_change("RIGHT"); axis_change("RIGHT");
		elif(target_axis == 3):
			ser.write("R".encode()); check();  ser.write("L".encode()); check(); 
			ser.write("M".encode()); check();  axis_change("RIGHT"); axis_change("LEFT"); axis_change("LEFT");
	elif(axis == 2):
		if(target_axis == 3):
			ser.write("L".encode()); check(); 	ser.write("R".encode()); check();
			ser.write("K".encode()); check();   axis_change("LEFT"); axis_change("RIGHT"); axis_change("RIGHT");
		elif(target_axis == 1):
			ser.write("R".encode()); check();   ser.write("L".encode()); check();
			ser.write("M".encode()); check();   axis_change("RIGHT"); axis_change("LEFT"); axis_change("LEFT");
	elif(axis == 3):
		if(target_axis == 1):
			ser.write("L".encode()); check();  ser.write("R".encode()); check();
			ser.write("K".encode()); check();  axis_change("LEFT"); axis_change("RIGHT"); axis_change("RIGHT");
		elif(target_axis == 2):
			ser.write("R".encode()); check();  ser.write("L".encode()); check();
			ser.write("M".encode()); check();  axis_change("RIGHT"); axis_change("LEFT"); axis_change("LEFT");



"""
Function Name : init_gl()
Input: None
Output: None
Purpose: Initialises various parameters related to OpenGL scene.
"""  
def init_gl():
	global texture_object, texture_background,crow,pitcher_high,pitcher_medium,pitcher_low,pebble_large,pebble_diminished,pitcher_full
	glClearColor(0.0, 0.0, 0.0, 0.0)
	glClearDepth(1.0) 
	glDepthFunc(GL_LESS)
	glEnable(GL_DEPTH_TEST)
	glShadeModel(GL_SMOOTH)   
	glMatrixMode(GL_MODELVIEW)
	glEnable(GL_DEPTH_TEST)
	glEnable(GL_LIGHTING)
	glEnable(GL_LIGHT0)
	texture_background = glGenTextures(1)
	texture_object = glGenTextures(1)
	crow = OBJ('Crow.obj', swapyz = True)
	pitcher_high = OBJ('level3.obj', swapyz = True)
	pitcher_medium = OBJ('level2.obj', swapyz = True)
	pitcher_low = OBJ('level1.obj', swapyz = True)
	pitcher_full = OBJ('level4.obj', swapyz = True)
	pebble_large = OBJ('pebble_d_large.obj', swapyz = True)
	pebble_diminished = OBJ('pebble_diminished.obj', swapyz = True)



"""
Function Name : resize(w,h)
Input: w,h
Output: None
Purpose: Initialises the projection matrix of OpenGL scene
"""
def resize(w,h):
	ratio = 1.0* w / h
	glMatrixMode(GL_PROJECTION)
	glViewport(0,0,w,h)
	gluPerspective(45, ratio, 0.1, 100.0)



"""
Function Name : drawGLScene()
Input: None
Output: None
Purpose: It is the main callback function which is called again and
         again by the event processing loop. In this loop, the webcam frame
         is received and set as background for OpenGL scene. ArUco marker is
         detected in the webcam frame and 3D model is overlayed on the marker
         by calling the overlay() function.
"""
def drawGLScene():
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
	ar_list = []
	ret, frame = cap.read()
	if ret == True:
		draw_background(frame)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()
		ar_list = detect_markers(frame)

		for i in ar_list:
			if i[0] == 10:
				overlay(frame, ar_list, i[0], "rr.png")
			if i[0] == 1:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 2:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 3:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 4:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 5:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 6:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 7:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 8:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 9:
				overlay(frame, ar_list, i[0], None)
			if i[0] == 0:
				overlay(frame, ar_list, i[0], None)
	cv2.imshow('frame', frame)
	cv2.waitKey(1)
	glutPostRedisplay()
	glutSwapBuffers()
        


"""
Function Name : draw_background(img)
Input: img (numpy array)
Output: None
Purpose: Takes image as input and converts it into an OpenGL texture. That
         OpenGL texture is then set as background of the OpenGL scene
"""
def draw_background(img):

	#CONVERTING CAMERA FRAME INTO OPENGL USABLE FORM
	tex_back = cv2.flip(img, 0)                                                             #FLIPS THE IMAGE ABOUT X-AXIS, REQUIRED BECAUSE OPENGL WORKS ON DIFF. COORDINATE
	tex_back = Image.fromarray(tex_back)                                                    #CONVERTS IMAGE INTO PIL SPECIFIC FORMAT
	ix = tex_back.size[0]                                                                   #RETURNS SIZE OF IMAGE
	iy = tex_back.size[1]                                                                   #RETURNS SIZE OF IMAGE IN OTHER DIRECTION
	tex_back = tex_back.tobytes("raw","BGRX", 0, -1)                                        #CONVERTS IMAGE DATA INTO FORM OF INDIVIDUAL BYTES
	
	#DEFINING TEXTURE TYPE FOR BACKGROUND
	glActiveTexture(GL_TEXTURE0)
	glBindTexture(GL_TEXTURE_2D, texture_background)                                        #SETTING TEXTURE ID ASSOSIATED WITH texture_background TO USE 2D TEXTURE
	glEnable(GL_TEXTURE_2D)
	
	#DEFINING TEXTURE PARAMETERS FOR BOTH POSSIBLE CASES i-e THE SIZE OF BACKGROUND ON WHICH TEXTURE HAS TO BE APPLIED IS SMALLER THAN TEXTURE FILE OR LARGER

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)                        #TEXTURE PARAMETERS
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
	
	#ABOVE STEPS WERE TO GET TEXTURE READY, FOLLOWING STEPS DEAL WITH APPLYING THE TEXTURE
	
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_back)  #ASSIGNING tex_back, THE FILE ASSOSIATED WITH FRAME TO THE ACTIVE TEXTURE ID
	
	#CONSTRUCTING A QUAD IN THE OPENGL WINDOW AND THE TEXTURE ASSOSIATED WITH CURRENT TEXTURE ID GETS MAPPED TO THE QUAD.
        
	#glPushMatrix()
	glTranslatef(0.0,0.0,-4.5)                                                              #TRANSLATES THE QUAD IN Z-AXIS TO ADJUST THE SIZE
	glBegin(GL_QUADS)                                                                       #BEGINS THE OPENGL CODE FOR QUAD CONSTRUCTION
	#THE COORDINATES ENTERED IN FOLLOWING STEPS WILL BE RECORDED ACCORDING TO A PRE-DEFINED ORDER AND ACCORDINGLY THE QUAD WILL BE CONSTRUCTED 
	glTexCoord2fv([0.0, 1.0]);
	glVertex3fv([-4.0, -3.0, 0.0])
	glTexCoord2fv([1.0, 1.0]);
	glVertex3fv([ 4.0, -3.0, 0.0])
	glTexCoord2fv([1.0, 0.0]); 
	glVertex3fv( [4.0,  3.0, 0.0])
	glTexCoord2fv([0.0, 0.0]);
	glVertex3fv([-4.0,  3.0, 0.0])
	glEnd( )                                                                                #END OF THE OPENGL CODE TO CONSTRUCT QUAD
	#glPopMatrix()
                                                                                                
	return None



"""
Function Name : init_object_texture(image_filepath)
Input: Image file path
Output: None
Purpose: Takes the filepath of a texture file as input and converts it into OpenGL
         texture. The texture is then applied to the next object rendered in the OpenGL
         scene.
"""
def init_object_texture(image_filepath):

	#THE COMMENTS FOR THE FOLLOWING STEPS ARE SAME AS IN THE ABOVE FUNCTION, THE ONLY DIFFERENCE IS, HERE INSTEAD OF USING CAMERA FRAME WE USE IMAGE FILES
	tex = cv2.imread(image_filepath)
	tex = cv2.flip(tex, 0)
	tex = Image.fromarray(tex)
	ix = tex.size[0]
	iy = tex.size[1]
	tex = tex.tobytes("raw","BGRX", 0, -1)

	glActiveTexture(GL_TEXTURE0)
	glBindTexture(GL_TEXTURE_2D, texture_object)
	glEnable(GL_TEXTURE_2D)
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
	glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex)
	return None



"""
Function Name : overlay(img, ar_list, ar_id)
Input: img (numpy array), aruco_list, aruco_id, texture_file (filepath of texture file)
Output: None
Purpose: Receives the ArUco information as input and overlays the 3D Model of a teapot
		on the ArUco marker. That ArUco information is used to
		calculate the rotation matrix and subsequently the view matrix. Then that view matrix
		is loaded as current matrix and the 3D model is rendered.

		Parts of this code are already completed, you just need to fill in the blanks. You may
		however add your own code in this function.
"""
def overlay(img, ar_list, ar_id, texture_file):
	
	global type;
	
	for x in ar_list:
		if ar_id == x[0]:
			centre, rvec, tvec = x[1], x[2], x[3]

	rmtx = cv2.Rodrigues(rvec)[0]															#RETURNS ROTATION MATRIX FROM THE ROTATION VECTOR (MATRIX CALCULATED USING RODRIGUES FORMULA)
																										#WHICH CONVERTS AXIS-ANGLE REPRESENTATION INTO ROTATION MATRIX
	#IN ORDER TO TRANSFORM FROM THE COORDINATE FRAME OF RENDERED OBJECT TO THE WORLD FRAME WE REQUIRE TRANSFORMATION MATRIX
	#IN THE FOLLOWING STEPS ROTATION MATRIX IS CONVERTED INTO REQUIRED TRANSFORMATION MATRICES IN THE FORMAT OPENGL WORKS

	#ROTATION MATRICES ARE SAME wrt THE OPENCV WINDOW AND OPENGL WINDOW BUT VALUE OF TRANSLATION VECTOR VARIES FOR BOTH WINDOWS
	#HENCE WE HAVE TO MAP THE VALUES OF TRANSLATION VECTOR SUCH THAT RENDERED OBJECT IS POSITIONED PROPERLY IN THE  OPENGL WINDOW AS WELL

	#######################################################  IMPORTANT  ##################
	###MAPPING IS DONE WITH HELP OF LINEAR EQUATIONS
	###i-e VALUES OF tvecs ARE MANIPULATED USING LINEAR EQUATIONS, USING LINEAR EQUATION ENSURES PROPER SIZE OF TEAPOT FOR DIFFERENT POSITIONS OF MARKER IN Z-AXIS
	###AND ALSO PROPERLY MAPS THE TRANSLATIONS IN X-AXIS AND Y-AXIS FOR ALL VALUES OF Z

	#COMBINIG ROTATION MATRIX AND TRANSLATION VECTOR TO GIVE TRANSFORMATION MATRIX

	x = tvec[0][0][0];	y = tvec[0][0][1];	z = tvec[0][0][2];

	if ar_id == 4:
		if type == "LARGE":
			equation_x = (x+54)/80;		equation_y = (y-100)/15;
			a = 5;		b = 2.0;
		else:
			equation_x = (x+54)/70;		equation_y = (y-100)/8;
			a = 7;		b = 2.0;
	elif ar_id == 6:
		equation_x = (x+54)/180;	equation_y = (y-100)/228;
		a = 5;		b = 2.0;
	elif ar_id == 2:
		equation_x = (x + 54)/224;		equation_y = (y-100)/180;
		a = 6;		b = 2.0;
	elif ar_id == 0:
		equation_x = (x+54)/150;		equation_y = (y-100)/350;
		a = 4;		b = 1.0;
	else:
		if(centre[0] > 200 and centre[0] < 490):
			equation_x = (x+54)/224
		elif(centre[0] > 489):
			equation_x = (x + 54)/240
		else:
			equation_x = (x + 54)/250

		if(centre[1] > 155):
			equation_y = (y-100)/228
		elif(centre[1] < 156):
			equation_y = (y-100)/300
		
		a = 4;		b = 1.0;


	view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2], equation_x],                        #MAPPING(i-e manipulating values of tvec) OF VALUES USING LINEAR EQUATIONS
		[rmtx[1][0],rmtx[1][1],rmtx[1][2], equation_y],
		[rmtx[2][0],rmtx[2][1],rmtx[2][2], a],													#(((-0.7*centre[1])+939.6)/219)
		[0.0       ,0.0       ,0.0       ,b]])
	#CONVERTING TRANSFORMATION MATRIX IN OPENGL USABLE FORM BY INVERTING THE REQUIRED AXIS AND CHANGING THE FORMAT FROM ROW MAJOR TO COLUMN MAJOR BY TAKING TRANSPOSE
	view_matrix = view_matrix * INVERSE_MATRIX                                           
	view_matrix = np.transpose(view_matrix)
	
	glPushMatrix()                                  
	glLoadMatrixd(view_matrix)                                                              #LOADS THE VIEW MATRIX IN CURRENT STACK OF MATRICES
	
	if ar_id == 10:
		init_object_texture(texture_file)
		glCallList(crow.gl_list)
	elif ar_id == 0:
		if pickup_data[10] == 0:
			glCallList(pitcher_low.gl_list)
		elif pickup_data[10] == 1:
			glCallList(pitcher_medium.gl_list)
		elif pickup_data[10] == 3:
			glCallList(pitcher_full.gl_list)
		else:
			glCallList(pitcher_high.gl_list)
	elif ar_id == 1:
		if pickup_data[1] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	elif ar_id == 2:
		if pickup_data[2] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	elif ar_id == 3:
		if pickup_data[3] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	elif ar_id == 4:
		if pickup_data[4] == False:
			glCallList(pebble_large.gl_list)
			type = "LARGE"
		else:
			glCallList(pebble_diminished.gl_list)
			type = "DIMINSHED"
	elif ar_id == 5:
		if pickup_data[5] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	elif ar_id == 6:
		if pickup_data[6] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	elif ar_id == 7:
		if pickup_data[7] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	elif ar_id == 8:
		if pickup_data[8] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	elif ar_id == 9:
		if pickup_data[9] == False:
			glCallList(pebble_large.gl_list)
		else:
			glCallList(pebble_diminished.gl_list)
	
	glPopMatrix()
			


'''
* Function Name: check()
* Input: NONE
* Output: NONE 
* Logic: Function runs untl robot sends character '1' to signify the completion of movement 
*		 and while the movement is going on (i-e waiting for '1') glut loop is called for 
*		 AR implementation using glutMainLoopEvent() so as to execute the glut loop only once at a time
* Example Call: check()
'''
def check():
	while(ser.readline().decode() != '1'):
		_,frame = cap.read()
		#glutIdleFunc(drawGLScene)
		glutMainLoopEvent()

		
		
'''
* Function Name: rendering_main()
* Input: NONE
* Output: NONE 
* Logic:  Function to call all required initialisations for AR implementation
* Example Call: rendering_main()
'''		
def rendering_main():
	glutInit()
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION)
	getCameraMatrix()
	glutInitWindowSize(640, 480)
	glutInitWindowPosition(625, 100)
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
	window_id = glutCreateWindow("OpenGL")
	init_gl()
	glutDisplayFunc(drawGLScene)
	glutIdleFunc(drawGLScene)
	glutReshapeFunc(resize)



'''
THE FOLLOWING CODE WILL BE EXECUTED IF THE PYTHON SCRIPT IS BEING RUN AS MAIN SCRIPT i-e NOT CALLED IN SOME OTHER PYTHON PROGRAM
'''
if __name__ == "__main__":
	
	# THE PYTHON CODE INSTEAD OF TAKING ANYTHING AS INPUT ITSELF DECIDES THE AXIS OF MARKERS, NUMBER OF LOCATIONS etc.
	
	global pickup_pos_list, frame, deposition_zone;
	
	ser = serial.Serial("COM19", 9600, timeout=0.005)
	
	getCameraMatrix()
	
	ret, frame = cap.read()
	detect_positions(frame, True, True)
	calculate_euclidean_distances() #chances of error in if blocs inside the function
	orientation_finder()
	
	for i in range(len(pickup_pos_list)):
		pickup_data[pickup_pos_list[i][0]] = False
	
	pickup_data[10] = 0
	
	rendering_main()
	
	init_time = time.time()
	while time.time() - init_time < 5:
		glutMainLoopEvent()
	
	while len(pickup_pos_list) != 0 :	
		
		__,frame = cap.read()
		detect_positions(frame, False, False)
		traversal_function(pickup_pos_list[0][1], pickup_pos_list[0][5])
		pickup_data[pickup_pos_list[0][0]] = True
		
		
		__,frame = cap.read()
		detect_positions(frame, False, False)
		traversal_function(deposition_zone[1], deposition_zone[4])
		pickup_data[10] += 1 # THIS VALUE IS USED TO RENDER APPROPRAIATE MODEL ON BASIS WHETHER OPERATION PERFORMED OR NOT
		
		# REMOVE THE PICK-UP LOCATION COMPLETED FROM THE LIST 
		pickup_pos_list = pickup_pos_list[1:]
		print(axis)
	
	ser.write("S".encode()); check(); 
	glutMainLoop()
	
	
	

