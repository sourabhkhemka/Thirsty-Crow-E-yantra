"""
**************************************************************************
*                  E-Yantra Robotics Competition
*                  ================================
*  This software is intended to check version compatiability of open source software
*  Theme: Thirsty Crow
*  MODULE: Task1.1
*  Filename: detect.py
*  Version: 1.0.0  
*  Date: October 31, 2018
*  
*  Author: e-Yantra Project, Department of Computer Science
*  and Engineering, Indian Institute of Technology Bombay.
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
*     
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import pygame


texture_object = None
texture_background = None
camera_matrix = None
dist_coeff = None
cap = cv2.VideoCapture(0)
INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [ 1.0, 1.0, 1.0, 1.0]])

################## Define Utility Functions Here #######################
"""
Function Name : getCameraMatrix()
Input: None
Output: camera_matrix, dist_coeff
Purpose: Loads the camera calibration file provided and returns the camera and
         distortion matrix saved in the calibration file.
"""
def getCameraMatrix():
        global camera_matrix, dist_coeff
        with np.load('System.npz') as X:
                camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]



########################################################################

############# Main Function and Initialisations ########################
"""
Function Name : main()
Input: None
Output: None
Purpose: Initialises OpenGL window and callback functions. Then starts the event
         processing loop.
"""        
def main():
        glutInit()
        getCameraMatrix()
        glutInitWindowSize(640, 480)
        glutInitWindowPosition(625, 100)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
        window_id = glutCreateWindow("OpenGL")
        init_gl()
        glutDisplayFunc(drawGLScene)
        glutIdleFunc(drawGLScene)
        glutReshapeFunc(resize)
        glutMainLoop()

"""
Function Name : init_gl()
Input: None
Output: None
Purpose: Initialises various parameters related to OpenGL scene.
"""  
def init_gl():
        global texture_object, texture_background
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
"""
Function Name : resize()
Input: None
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
                        if i[0] == 8:
                                overlay(frame, ar_list, i[0],"texture_1.png")
                        if i[0] == 2:
                                overlay(frame, ar_list, i[0],"texture_2.png")
                        if i[0] == 7:
                                overlay(frame, ar_list, i[0],"texture_3.png")
                        if i[0] == 6:
                                overlay(frame, ar_list, i[0],"texture_4.png")

                cv2.imshow('frame', frame)
                cv2.waitKey(1)
        glutSwapBuffers()
        
########################################################################

######################## Aruco Detection Function ######################
"""
Function Name : detect_markers()
Input: img (numpy array)
Output: aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
        centre_2, rvec_2, tvec_2), ()....]
Purpose: This function takes the image in form of a numpy array, camera_matrix and
         distortion matrix as input and detects ArUco markers in the image. For each
         ArUco marker detected in image, paramters such as ID, centre coord, rvec
         and tvec are calculated and stored in a list in a prescribed format. The list
         is returned as output for the function
"""
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

"""
Function Name : draw_background()
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
Function Name : init_object_texture()
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
Function Name : overlay()
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
        for x in ar_list:
                if ar_id == x[0]:
                        centre, rvec, tvec = x[1], x[2], x[3]
                                                                                                
        rmtx = cv2.Rodrigues(rvec)[0]                                                           #RETURNS ROTATION MATRIX FROM THE ROTATION VECTOR (MATRIX CALCULATED USING RODRIGUES FORMULA)
                                                                                                                #WHICH CONVERTS AXIS-ANGLE REPRESENTATION INTO ROTATION MATRIX

        #IN ORDER TO TRANSFORM FROM THE COORDINATE FRAME OF RENDERED OBJECT TO THE WORLD FRAME WE REQUIRE TRANSFORMATION MATRIX
        #IN THE FOLLOWING STEPS ROTATION MATRIX IS CONVERTED INTO REQUIRED TRANSFORMATION MATRICES IN THE FORMAT OPENGL WORKS

        #ROTATION MATRICES ARE SAME wrt THE OPENCV WINDOW AND OPENGL WINDOW BUT VALUE OF TRANSLATION VECTOR VARIES FOR BOTH WINDOWS
        #HENCE WE HAVE TO MAP THE VALUES OF TRANSLATION VECTOR SUCH THAT RENDERED OBJECT IS POSITIONED PROPERLY IN THE  OPENGL WINDOW AS WELL

        #######################################################  IMPORTANT  ##################
        ###MAPPING IS DONE WITH HELP OF LINEAR EQUATIONS
        ###i-e VALUES OF tvecs ARE MANIPULATED USING LINEAR EQUATIONS, USING LINEAR EQUATION ENSURES PROPER SIZE OF TEAPOT FOR DIFFERENT POSITIONS OF MARKER IN Z-AXIS
        ###AND ALSO PROPERLY MAPS THE TRANSLATIONS IN X-AXIS AND Y-AXIS FOR ALL VALUES OF Z

        mult = ((0.2*tvec[0][0][2])+200)/500                                                 #VALUE RETURNED BY LINEAR EQUATION, INDIPENDENT VAR IS THE POSITION OF MARKER IN Z-
        sk = ((3.5*tvec[0][0][2])+1385)/680                                                  #THE VALUES RETURNED BY THESE TWO EQUATIONS ARE USED AHEAD FOR MAPPING

        #COMBINIG ROTATION MATRIX AND TRANSLATION VECTOR TO GIVE TRANSFORMATION MATRIX

        view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2],  sk*(tvec[0][0][0]/(tvec[0][0][2]))],                        #MAPPING(i-e manipulating values of tvec) OF VALUES USING LINEAR EQUATIONS
                        [rmtx[1][0],rmtx[1][1],rmtx[1][2],  (tvec[0][0][1]/100) -  (mult*( ( (1.5*tvec[0][0][1])+375)/360 ) )],
                        [rmtx[2][0],rmtx[2][1],rmtx[2][2], ((2*tvec[0][0][2])-1700)/400],
                        [0.0       ,0.0       ,0.0       ,1.0    ]])

        #CONVERTING TRANSFORMATION MATRIX IN OPENGL USABLE FORM BY INVERTING THE REQUIRED AXIS AND CHANGING THE FORMAT FROM ROW MAJOR TO COLUMN MAJOR BY TAKING TRANSPOSE
        view_matrix = view_matrix * INVERSE_MATRIX                                           
        view_matrix = np.transpose(view_matrix)
        
        init_object_texture(texture_file)                                                       #FUNCTION TO SET THE texture_file TO THE TEXTURE ID
        glPushMatrix()                                  
        glLoadMatrixd(view_matrix)                                                              #LOADS THE VIEW MATRIX IN CURRENT STACK OF MATRICES
        glutSolidTeapot(0.5)                                                                    #FUNCTION TO RENDER A TEAPOT
        glPopMatrix()

########################################################################

if __name__ == "__main__":
        main()
