import os
import numpy as np
import cv2
import cv2.aruco as aruco
import time

control_LR = 0      # Control input for left/right
control_FB = 0      # Cotnrol input for forward/back
control_UD = 0      # Control input for up/down
control_YA = 0      # Control input for yaw

def controller(act_dist, id):
    act_dist = tvec[i][0] #green line, red line, blue line
    y_dist = act_dist[0]*100
    x_dist = act_dist[2]*100
   
    # Controller Variables
    ref = [0, 0, 0]
    x_kp = 1.0
    x_ki = 50.0
    x_kd = 0.0

    y_kp = 1.0
    y_ki = 50.0
    y_kd = 0.0
    

    # Control stores
    x_integratedError = 0.0
    y_integratedError = 0.0
    x_errorStore = 0.0
    y_errorStore = 0.0


    # to prevent hickups
    lastTime = 0.0
    lastFB = 0.0
    lastLR = 0.0
    timeStep = 0.1

    # Get data (read sensors)
    presentState = stateQ.get(block=True, timeout=None)  # block if needed until new state is ready
    ptime = presentState[1]     # present time (don't over write time function)
    print(presentState)
    FB = presentState[] #TBD
    LR = presentState[] #TBD
    
    if lastTime > ptime:
        ptime = lastTime
        FB = lastFB
        LR = lastLR

    # Compute Error and Control Input
    y_error = int(y_dist - ref[0])
    y_integratedError = y_integratedError + timeStep * y_error
    y_errorDerivative = (y_error - y_errorStore) / timeStep
    y_errorStore = y_error

    x_error = int(x_dist - ref[2])
    x_integratedError = x_integratedError + timeStep * x_error
    x_errorDerivative = (x_error - x_errorStore) / timeStep
    x_errorStore = x_error

    control_LR = y_kp*y_error + y_ki*y_integratedError + y_kd*y_errorDerivative
    control_FB = x_kp*x_error + x_ki*x_integratedError + x_kd*x_errorDerivative
    
    lastTime = ptime
    lastFB = FB
    lastLR =LR

    # Send Control to quad
    # first check y distance and control LR
    if abs(y_error) > 5:
        control_LR = int(np.clip(control_LR,-100,100))
        send('rc '+str(control_LR)+' '+str(control_FB)+' '+str(control_UD)+' '+str(control_YA))

    # second check x distance and control FB
    elif abs(x_error) < 50:
        send('flip f')
        return True
    else:
        control_FB = int(np.clip(control_FB,-100,100))
        send('rc '+str(control_LR)+' '+str(control_FB)+' '+str(control_UD)+' '+str(control_YA))
    return False

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


# Send the message to Tello
def send(message):
    message = message.encode(encoding="utf-8") 
    sock.sendto(message, tello_address)
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

                # # delay 5 seconds after each command sending
                # time.sleep(5)
                
                #ruiji_ store the coordinates in tello
                # tellodata += tvec[i][0]
                # timedata += start_time-time.time()
            if controller(tvec[i][0], ids[i]):
                send('land')
                break
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
