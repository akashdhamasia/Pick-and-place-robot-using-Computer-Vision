#@Project: Pick and Place
#@Auther: Akash Dhamasia
#@DateCreated: 30-06-2018

#export PYTHONPATH=$PYTHONPATH:/usr/local/lib

import numpy as np
import cv2
import cv2.aruco as aruco
import glob

import pyrealsense2 as rs
import numpy as np
import cv2

import math

import serial # you need to install the pySerial :pyserial.sourceforge.net
import time
# your Serial port should be different!
arduino = serial.Serial('/dev/ttyACM0', 9600)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


#cap = cv2.VideoCapture(0)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
#print(objp)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('calibration_realsense/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,7),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,7), corners2,ret)


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)


def sendcommand(command, align_angle):

    #command = raw_input("Type something..: (on/ off / bye )");
    if command =="initialise":
        print ("Initialising...")
        time.sleep(2) 
        arduino.write('i'.encode())

    elif command =="forward":
        print ("forward")
        arduino.write('f'.encode())
        time.sleep(.1) 
        arduino.flush()

    elif command =="backward":
        print ("backward")
        arduino.write('b'.encode())
        time.sleep(.1) 
        arduino.flush()

    elif command =="rotate_left":
        print ("rotate_left")
        arduino.write('v'.encode())
        time.sleep(.1) 
        arduino.flush()

    elif command =="rotate_right":
        print ("rotate_right")
        arduino.write('r'.encode())
        time.sleep(.1) 
        arduino.flush()

    elif command =="pickup":
        print ("pickingup...")
        time.sleep(1) 
        arduino.write('p'.encode())
        time.sleep(.2) 
        arduino.write(str(align_angle).encode())
        time.sleep(2) 
        arduino.flush()

    elif command =="stop":
        print ("stop, no command")
        #time.sleep(1) 
        arduino.write('s'.encode())

    elif command =="dropup":
        print ("See You!...")
        time.sleep(1) 
        arduino.close()

    else:
        print ("Sorry..type another thing..!")

    return

time.sleep(2) #waiting the initialization...

#while True:
#    print arduino.readline()

sendcommand("initialise", 0)

try:

	timer_threshold = 300
	initialise_object = 0
	timer = 0

	while (True):

		# Wait for a coherent pair of frames: depth and color
		frames = pipeline.wait_for_frames()
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()
		if not depth_frame or not color_frame:
			continue

		# Convert images to numpy arrays
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
		depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

		# Stack both images horizontally
		images = np.hstack((color_image, depth_colormap))

		# Show images
		#cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		#cv2.imshow('RealSense', images)
		#cv2.waitKey(1)

		frame = color_image

		#ret, frame = cap.read()
		#frame = cv2.imread('images/marker_66.jpg')
		#frame = cv2.resize(frame, (100,100))

		# operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		parameters = aruco.DetectorParameters_create()

		#lists of ids and the corners beloning to each id
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

		font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

		if np.all(ids != None):
			rvec,tvec,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
            #(rvec-tvec).any() # get rid of that nasty numpy value array error

			i = 0
			vector_x = []
			vector_y = []
			vector_z = []
			centroid = []

			while (i < ids.size):

				length = 0.1
				axis = np.float32([[0,0,0],[length,0,0], [0,length,0], [0,0,length]]).reshape(-1,3)

				imgpts, jac = cv2.projectPoints(axis, rvec[i], tvec[i], mtx, dist)

				imgpts = np.int32(imgpts).reshape(-1,2)

				cv2.line(frame, tuple(imgpts[0]), tuple(imgpts[1]),[0,0,255],4)  #BGR
				cv2.line(frame, tuple(imgpts[0]), tuple(imgpts[2]),[0,255,0],4)
				cv2.line(frame, tuple(imgpts[0]), tuple(imgpts[3]),[255,0,0],4)

				vector_x.append(imgpts[1] - imgpts[0])
				vector_y.append(imgpts[2] - imgpts[0])
				vector_z.append(imgpts[3] - imgpts[0])
				#aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1) #Draw Axis

				centroid.append((corners[i][0][0][0]+corners[i][0][1][0]+corners[i][0][2][0]+corners[i][0][3][0])/4)
				centroid.append((corners[i][0][0][1]+corners[i][0][1][1]+corners[i][0][2][1]+corners[i][0][3][1])/4)

				i = i + 1

			centroid = np.float32(centroid).reshape(-1,2)

			arm_index = np.where(ids==1)

			if arm_index[0].size > 0:

				armindex = arm_index[0][0]
				object_indexs = np.where(ids!=1)
				#print(object_indexs)

				if object_indexs[0].size > 0:

					if timer >= timer_threshold or initialise_object == 0:
						last_track_id = ids[object_indexs[0][0]][0]
						print("tracking new object ", last_track_id)

					object2track = np.where(ids==last_track_id)
					
					if  object2track[0].size > 0:
						object_index = object2track[0][0]
						centroid_track = centroid[object_index]
						vector_x_object = vector_x[object_index]
						timer = 0
						initialise_object = 1
						print("object detected, tracking object", ids[object_index][0] )


					else:
						print("tracking object not detected, timer initiated ",timer)
						timer = timer + 1
				else:
					print("no object in the scene")

					if initialise_object == 1 and timer < timer_threshold:
						timer = timer + 1
						print("tracking object ",last_track_id, "timer ", timer)
					else:
						print("tracking no object")
			

				if initialise_object == 1 and timer < timer_threshold:

					vector_x[armindex] = vector_x[armindex]/np.linalg.norm(vector_x[armindex])
					vector_x_object = vector_x_object/np.linalg.norm(vector_x_object)
					align_angle = np.arccos(np.clip(np.dot(vector_x[armindex], vector_x_object), -1.0, 1.0))
					align_angle = (align_angle * 180)/(math.pi)
					
					#if align_angle > 90:
					#	align_angle = 180 - align_angle  

					print("angle ", int(align_angle))

					
					print (centroid_track[0], " ", centroid_track[1])
					print (centroid[armindex][0], " ", centroid[armindex][1])

					if (centroid_track[0] - centroid[armindex][0]) > 5:  
						sendcommand("forward", 0) 

					elif (centroid_track[0] - centroid[armindex][0]) < -5:
						sendcommand("backward", 0)

					if (centroid[armindex][1] - centroid_track[1] > 5):
						sendcommand("rotate_left", 0)

					elif (centroid[armindex][1] - centroid_track[1] < -5):
						sendcommand("rotate_right", 0)

					if abs(centroid_track[0] - centroid[armindex][0]) < 5 and abs(centroid[armindex][1] - centroid_track[1] < 5):
						initialise_object = 0
						sendcommand("pickup", int(align_angle))
						time.sleep(22)
					
				else:
					sendcommand("stop", 0)
					print("no object found")
				
						
			aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers

			###### DRAW ID #####
			cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


		# Display the resulting frame
		cv2.imshow('frame',frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

finally:

    # Stop streaming
    pipeline.stop()
    # When everything done, release the capture
    #cap.release()
    cv2.destroyAllWindows()