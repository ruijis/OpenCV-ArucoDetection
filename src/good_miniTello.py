import os
import numpy as np
import cv2
import cv2.aruco as aruco
import time
from time import sleep

import socket
import threading
import sys
import numpy as np
from queue import Queue
from queue import LifoQueue

def end_program():
      print("Program exited")
      stateStop.set()  # set stop variable
      stateThread.join()   # wait for termination of state thread before closing socket
      writeDataFile(State_data_file_name)
      CmdSock.close()  # sockete for commands
      StateSock.close()  # socket for state
      print("sockets and threads closed")

def controller(act_dist, id):
    kx = 0.1
    ky = 0.05
    ref = [36, 0, 100]

    act_dist = tvec[i][0] #green line, red line, blue line

    y_dist = act_dist[0]*100
    x_dist = act_dist[2]*100

    y_error = int(y_dist - ref[0])
    x_error = int(x_dist - ref[2])

    print("Y_ERROR: ", y_error)
    print("X_ERROR: ", x_error)

    # Compute Error and Control Input
    control_FB = x_error * kx
    control_LR = y_error * ky

    # if y_error > 10:
    #     send('right 20')
    # elif y_error < -10:
    #     send('left 20')
    if x_error < 50:
        send('flip f')
        # send('back 100')
        return True
    else:
        # Send Control to quad
        control_UD = 0
        control_YA = 0
        control_LR = int(np.clip(control_LR,-100,100))
        control_FB = int(np.clip(control_FB,-100,100))
        # control_UD = int(np.clip(control_UD,-100,100))
        # control_YA = int(np.clip(control_YA,-100,100))
        print("CONTROL_LR:", control_LR)
        print("CONTROL_FB:", control_FB)

        message = 'rc '+str(control_LR)+' '+str(control_FB)+' '+str(control_UD)+' '+str(control_YA)
        # message = 'forward 20'
        send(message)
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
    sleep(0.1)
    print("SENDING MSG TO TELLO: ", message)


send('command')
send('streamon')
send('takeoff')


########################################################
# TelloClosedLoopDist.py
State_data_file_name = 'statedata.txt'
index = 0
reference = 0.0     # Reference signal
control_LR = 0      # Control input for left/right
control_FB = 0      # Cotnrol input for forward/back
control_UD = 0      # Control input for up/down
control_YA = 0      # Control input for yaw
INTERVAL = 0.05  # update rate for state information
start_time = time.time()
dataQ = Queue()
stateQ = LifoQueue() # have top element available for reading present state by control loop

# IP and port of Tello for commands
tello_address = ('192.168.10.1', 8889)
# IP and port of local computer
local_address = ('', 8889)
# Create a UDP connection that we'll send the command to
CmdSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
CmdSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Bind to the local address and port
CmdSock.bind(local_address)

###################
# socket for state information
local_port = 8890
StateSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
StateSock.bind(('', local_port))
CmdSock.sendto('command'.encode('utf-8'), tello_address)   # command port on Tello

def writeFileHeader(dataFileName):
    fileout = open(dataFileName,'w')
    #write out parameters in format which can be imported to Excel
    today = time.localtime()
    date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
    date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
    fileout.write('"Data file recorded ' + date + '"\n')
    # header information
    fileout.write('  index,   time,    ref,ctrl_LR,ctrl_FB,ctrl_UD,ctrl_YA,  pitch,   roll,    yaw,    vgx,    vgy,    vgz,   templ,   temph,    tof,      h,    bat,   baro,   time,    agx,    agy,    agz\n\r')
    fileout.close()

def writeDataFile(dataFileName):
    fileout = open(State_data_file_name, 'a')  # append
    print('writing data to file')
    while not dataQ.empty():
        telemdata = dataQ.get()
        np.savetxt(fileout , [telemdata], fmt='%7.3f', delimiter = ',')  # need to make telemdata a list
    fileout.close()

def report(str,index):
    telemdata=[]
    telemdata.append(index)
    telemdata.append(time.time()-start_time)
    telemdata.append(reference)
    telemdata.append(control_LR)
    telemdata.append(control_FB)
    telemdata.append(control_UD)
    telemdata.append(control_YA)
    data = str.split(';')
    data.pop() # get rid of last element, which is \\r\\n
    for value in data:
        temp = value.split(':')
        if temp[0] == 'mpry': # roll/pitch/yaw
            temp1 = temp[1].split(',')
            telemdata.append(float(temp1[0]))     # roll
            telemdata.append(float(temp1[1]))     # pitch
            telemdata.append(float(temp1[2]))     # yaw
            continue
        quantity = float(value.split(':')[1])
        telemdata.append(quantity)
    dataQ.put(telemdata)
    stateQ.put(telemdata)
    if (index %100) == 0:
        print(index, end=',')

# Receive the message from Tello
def receive():
  # Continuously loop and listen for incoming messages
  while True:
    # Try to receive the message otherwise print the exception
    try:
      response, ip_address = CmdSock.recvfrom(128)
      print("Received message: " + response.decode(encoding='utf-8'))
    except Exception as e:
      # If there's an error close the socket and break out of the loop
      CmdSock.close()
      print("Error receiving: " + str(e))
      break

# receive state message from Tello
def rcvstate():
    print('Started rcvstate thread')
    index = 0
    while not stateStop.is_set():

        response, ip = StateSock.recvfrom(1024)
        if response == 'ok':
            continue
        report(str(response),index)
        sleep(INTERVAL)
        index +=1
    print('finished rcvstate thread')

# Create and start a listening thread that runs in the background
# This utilizes our receive function and will continuously monitor for incoming messages
receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()

writeFileHeader(State_data_file_name)  # make sure file is created first so don't delay
stateThread = threading.Thread(target=rcvstate)
stateThread.daemon = False  # want clean file close
stateStop = threading.Event()
stateStop.clear()
stateThread.start()

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
                end_program()
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
        end_program()
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
