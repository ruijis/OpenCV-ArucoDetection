import os
import numpy as np
import cv2
import cv2.aruco as aruco
import time
from time import sleep

def calibrate():

    cap = cv2.VideoCapture(0)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # checkerboard of size (9 x 7) is used
    objp = np.zeros((7*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)

    # arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # resizing for faster detection
        frame = cv2.resize(frame, (640, 480))
        # using a greyscale picture, also for faster detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (9,7), None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(frame, (9,7), corners, ret)
            #write_name = 'corners_found'+str(idx)+'.jpg'

        # Display the resulting frame
        cv2.imshow('Calibration',frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(10)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    #create a file to store data
    from lxml import etree
    from lxml.builder import E

    global fname
    with open(fname, "w") as f:
        f.write("{'ret':"+str(ret)+", 'mtx':"+str(list(mtx))+', "dist":'+str(list(dist))+'}')
        f.close()


# Tello Python3 Control Demo
# http://www.ryzerobotics.com/
# 1/1/2018
import threading
import socket
import sys
import time

host = ''
port = 9000
locaddr = (host,port)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tello_address = ('192.168.10.1', 8889)
sock.bind(locaddr)

def recv():
    count = 0
    while True:
        try:
            data, server = sock.recvfrom(1518)
            print(data.decode(encoding="utf-8"))
        except Exception:
            print ('\nExit . . .\n')
            break

receiveThread = threading.Thread(target=recv)
receiveThread.start()


# Send the message to Tello and allow for a delay in seconds
def send(message):
    message = message.encode(encoding="utf-8")
    sock.sendto(message, tello_address)
    # delay 5 seconds
    # time.sleep(5)
    print("SENDING MSG TO TELLO: ", message)


send('command')
send('streamon')
send('takeoff')


# opencvCam.py
#########################################################
#test wheater already calibrated or not
path = os.path.abspath('..')
fname = path + "/res/calibration_parameters.txt"
print(fname)
try:
    f = open(fname, "r")
    f.read()
    f.close()
except:
    calibrate()


# ruijis_read Tello streamon video
cap = cv2.VideoCapture("udp://@0.0.0.0:11111")
# set a tellodata
# tellodata = [0, 0, 0]
start_time = time.time()


#importing aruco dictionary
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

#calibration parameters
f = open(fname, "r")
ff = [i for i in f.readlines()]
f.close()
from numpy import array
parameters = eval(''.join(ff))
mtx = array(parameters['mtx'])
dist = array(parameters['dist'])

# Create absolute path from this module
file_abspath = os.path.join(os.path.dirname(__file__), 'Samples/box.obj')

tvec = [[[0, 0, 0]]]
rvec = [[[0, 0, 0]]]

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250 )
markerLength = 0.25   # Here, our measurement unit is centimetre.
parameters = cv2.aruco.DetectorParameters_create()
parameters.adaptiveThreshConstant = 10

# Control Input
reference = 0.0     # Reference signal
control_LR = 0      # Control input for left/right
control_FB = 0      # Cotnrol input for forward/back
control_UD = 0      # Control input for up/down
control_YA = 0      # Control input for yaw
INTERVAL = 0.05  # update rate for state information
start_time = time.time()


while True:
    try:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if np.all(ids != None):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)

            #print(ids)
            #print(corners)
            #print(rvec)

            for i in range(0, ids.size):
                aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

                # show translation vector on the corner
                font = cv2.FONT_HERSHEY_SIMPLEX
                text = str([round(i,5) for i in tvec[i][0]])
                position = tuple(corners[i][0][0])
                cv2.putText(frame, text, position, font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

                #get tvec, rvec of each id
                print(ids[i])
                print(tvec[i][0])
                print(rvec[i][0])

                ##### Contoller Start
                # Contoller variables
                kx = 1

                # to prevent hickups
                # lastTime = 0.0
                # x = 0

                ref = [0, 0, 10]
                act_dist = tvec[i][0] #green line, red line, blue line
                x_dist = act_dist[2]*100

                # Compute Error and Control Input
                x_error = int(x_dist - ref[2])
                control_FB = x_error * kx # TODO: Forward/Backwards

                ##### Controller End

                if x_error < 60:
                    if x_error < 40:
                        send('land')
                    else:
                        send('flip f')
                else:
                    # control_LR = int(np.clip(control_LR,-100,100))
                    # control_FB = int(np.clip(control_FB,-100,100))
                    # control_UD = int(np.clip(control_UD,-100,100))
                    # control_YA = int(np.clip(control_YA,-100,100))
                    # message = 'rc '+str(control_LR)+' '+str(control_FB)+' '+str(control_UD)+' '+str(control_YA)
                    # send(message)
                    send('forward 20')

                # Wait so make sample time steady
                sleep(0.1)

                #ruiji_ store the coordinates in tello
                # tellodata += tvec[i][0]
                # timedata += start_time-time.time()

            aruco.drawDetectedMarkers(frame, corners)
        else:
            tvec = [[[0, 0, 0]]]
            rvec = [[[0, 0, 0]]]
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # Handle ctrl-c case to quit and close the socket
    except KeyboardInterrupt as e:
        send('land')
        sock.close()
        break

cap.release()
cv2.destroyAllWindows()


# State_data_file_name = 'statedata.txt'
# time_data_file = 'time.txt'
# fileout = open(State_data_file_name, 'a')
# timefileout = open(time_data_file, 'a')
# print('writing data to file')
# np.savetxt(timefileout , timedata, fmt='%7.3f', delimiter = ',')
# np.savetxt(fileout , tellodata, fmt='%7.3f', delimiter = ',')  # need to make telemdata a list
# fileout.close()
