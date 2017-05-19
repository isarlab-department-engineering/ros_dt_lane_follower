#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('lane_detection')
import rospy
import sys
from std_msgs.msg import Int32
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import numpy as np

def detect(img):

	# start time
	start_time = cv2.getTickCount()

	# Gaussian Filter to remove noise
	img = cv2.medianBlur(img,5)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	# print img.shape = (200,350,3)
	rows,cols,channels = img.shape
	
	# ROI
	roi_mask = np.zeros(img.shape,dtype=np.uint8)
	roi_mask[10:rows,0:cols] = 255
	street = cv2.bitwise_and(img,roi_mask)

	stop_roi_mask = np.zeros(gray.shape,dtype=np.uint8)
	stop_roi_mask[100:rows,150:250] = 255

	right_roi_mask = np.zeros(gray.shape,dtype=np.uint8)
	right_roi_mask[rows/3:rows,220:360] = 255
	right_roi = cv2.bitwise_and(img,img,right_roi_mask)

	left_roi_mask = np.zeros(gray.shape,dtype=np.uint8)
	left_roi_mask[rows/3:rows,0:180] = 255
	left_roi = cv2.bitwise_and(img,img,left_roi_mask)

	# define range of color in HSV
	hsv = cv2.cvtColor(street,cv2.COLOR_BGR2HSV)

	sensitivity = 60 # range of sensitivity=[90,150]
	lower_white = np.array([0,0,255-sensitivity])
	upper_white = np.array([255,sensitivity,255])

	white_mask = cv2.inRange(hsv,lower_white,upper_white)
	white_mask = cv2.erode(white_mask, None, iterations=2)
	white_mask = cv2.dilate(white_mask, None, iterations=2)
	
	lower_red = np.array([150,70,50])
	upper_red = np.array([200,255,255])

	red_mask = cv2.inRange(hsv,lower_red,upper_red)
	red_mask = cv2.erode(red_mask, None, iterations=2)
	red_mask = cv2.dilate(red_mask, None, iterations=2)

	lower_yellow = np.array([10,100,100]) #0,100,100
	upper_yellow = np.array([30,255,255]) #80,255,255

	yellow_mask = cv2.inRange(hsv,lower_yellow,upper_yellow)
	yellow_mask = cv2.erode(yellow_mask, None, iterations=2)
	yellow_mask = cv2.dilate(yellow_mask, None, iterations=2)

	# mask AND original img
	whitehsvthresh = cv2.bitwise_and(right_roi,right_roi,mask=white_mask)
	yellowhsvthresh = cv2.bitwise_and(street,street,mask=yellow_mask)
	redhsvthresh = cv2.bitwise_and(street,street,mask=red_mask)

	# Canny Edge Detection 
	right_edges = cv2.Canny(whitehsvthresh,100,200)
	left_edges = cv2.Canny(yellowhsvthresh,100,200)

	right_edges = cv2.bitwise_and(right_edges,right_roi_mask)
	left_edges = cv2.bitwise_and(left_edges,left_roi_mask)

	red_edges_hsv = cv2.Canny(redhsvthresh,100,200)
	red_edges = cv2.bitwise_and(red_edges_hsv,stop_roi_mask)
	
	# Probabilistic Hough Transform
#	minLength=50
#	maxGap=10
#	right_lines = cv2.HoughLinesP(right_edges,1,np.pi/180,30,minLength,maxGap)
#	left_lines = cv2.HoughLinesP(left_edges,1,np.pi/180,30,minLength,maxGap)
#	red_lines = cv2.HoughLinesP(red_edges,1,np.pi/180,100,minLength,maxGap)
#
#	w = 205 # da controllare
#	lw = 20 # da controllare
#	ly = 15 # da controllare
#	i = 0
#	j = 0
#	d = []
#	phi = []
#	if right_lines is not None:
#		for x in range(0,len(right_lines)):
#			for x1,y1,x2,y2 in right_lines[x]:
#				d_i = ((x1+x2)/2)-(w/2)
#				if x2>x1:
#					d_i = d_i - lw
#				d.insert(i,d_i)
#				a = x2-x1
#				if x2<x1:
#					a = -a
#				phi.insert(j,(np.pi)/2 - np.arctan(a/(y2-y1)))
#				i+1
#				j+1
#				rospy.loginfo("Right lane: ")
#				rospy.loginfo(d)
#				
#	if left_lines is not None:
#		for x in range(0,len(left_lines)):
#			for x1,y1,x2,y2 in left_lines[x]:	
#				d_i = ((x1+x2)/2)+(w/2)
#				if x2>x1:
#					d_i = d_i + ly
#				d.insert(i,d_i)
#				a = x2-x1
#				if x2<x1:
#					a = -a
#				phi.insert(j,(np.pi)/2) - np.arctan2((x2-x1)/(y2-y1))
#				i+1
#				j+1
#				rospy.loginfo("Left lane: ")
#				rospy.loginfo(d)
##	rospy.loginfo(d)
##	rospy.loginfo(phi)
#
##	bufferx_right = []
##	i=0
##	j=0
##	mdx=[]
 ##       if lines_right is not None:
  ##              for x in range(0,len(lines_right)):
   ##                    for x1,y1,x2,y2 in lines_right[x]:
    ##                            if x2!=x1:
 ##	                                m=(y2-y1)/(float(x2-x1))
  ##      	                        #alpha=np.arctan(m)
##				mdx.insert(j,m)
 ##               	        bufferx_right.insert(i,x1)
  ##                      	i+1
   ##                             bufferx_right.insert(i,x2)
    ##                            i+1
##				j+1
##	bufferx_left = []
##	i=0
##	j=0
##	msx=[]
 ##       if lines_left is not None:
  ##              for x in range(0,len(lines_left)):
   ##                    for x1,y1,x2,y2 in lines_left[x]:
    ##                            if x2!=x1:
     ##                                   m=(y2-y1)/(float(x2-x1))
      ##                                  #alpha=np.arctan(m)
       ##                         msx.insert(j,m)
	##			bufferx_left.insert(i,x1)
         ##                       i+1
          ##                      bufferx_left.insert(i,x2)
           ##                     i+1
	##			j+1
##        x=0
 ##       mx_right=0
  ##      for j in range(0,len(bufferx_right)):
   ##             x+=bufferx_right[j]
##	if len(bufferx_right)!=0:
##	        mx_right=x/len(bufferx_right)
##
##	x=0
##	mx_left=0
##	for k in range(0,len(bufferx_left)):
##		x+=bufferx_left[k]
##	if len(bufferx_left)!=0:
##		mx_left=x/len(bufferx_left)
##
##	mx=(mx_right+mx_left)/2
##
##	x=0
##	m_right = 0
##	for j in range(0,len(mdx)):
##		x+=mdx[j]
##	if len(mdx)!=0:
##		m_right=x/len(mdx)
##
##	x=0
##	m_left=0
###	for k in range(0,len(msx)):
###		x+=msx[k]
#	if len(msx)!=0:
#		m_left=x/(len(msx))
#
#	m = (m_right+m_left)/2	
#		
#	if lines_right is not None and lines_left is not None:
#		if (mx<=250 and mx>=150):
#			return "forward"
#		elif mx>250:
#			return "left"
#		elif mx<150:
#			return "right"
#	elif lines_left is None and lines_right is not None:
#		if mdx>0.8:
#			return "forward"
#		else:
#			return "left"
#	elif lines_right is None and bufferx_left is not None:
#		if msx>0.8:
#			return "forward"
#		else:
#			return "right"
#	else:
#		return "x"

	# Standard Hough Transform
	right_lines = cv2.HoughLines(right_edges,0.8,np.pi/180,40)
	left_lines = cv2.HoughLines(left_edges,0.8,np.pi/180,35)
	red_lines = cv2.HoughLines(red_edges,1,np.pi/180,30)
	
	xm = cols/2
	ym = rows
	
	# Draw right lane
	x = []
	i = 0
	if right_lines is not None:
		right_lines = np.array(right_lines[0])
		for rho, theta in right_lines:
                        a=np.cos(theta)
                        b=np.sin(theta)
                        x0,y0=a*rho,b*rho
                        y3 = 140
			x3 = int(x0+((y0-y3)*np.sin(theta)/np.cos(theta)))
			x.insert(i,x3)
			i+1

	if len(x) != 0:
		xmin = x[0]
		for k in range(0,len(x)):
			if x[k] < xmin and x[k] > 0:
				xmin = x[k]
		kr = int(np.sqrt(((xmin-xm)*(xmin-xm))+((y3-ym)*(y3-ym))))
	else:
		kr = 0
		xmin = 0

	# Draw left lane
	x = []
	i = 0

	if left_lines is not None:
		left_lines = np.array(left_lines[0])
		for rho, theta in left_lines:
                        a=np.cos(theta)
                        b=np.sin(theta)
                        x0,y0=a*rho,b*rho
			y3 = 140
			x3 = int(x0+((y0-y3)*np.sin(theta)/np.cos(theta)))
			x.insert(i,x3)
			i+1

        if len(x) != 0:
                xmax = x[0]
                for k in range(0,len(x)):
                        if x[k] > xmax and x[k]<cols:
                                xmax = x[k]
                kl = int(np.sqrt(((xmax-xm)*(xmax-xm))+((y3-ym)*(y3-ym))))
        else:
                kl = 0
		xmax = 0

	error = kr - kl

	#end time
	end_time = cv2.getTickCount()

	time_count= (end_time - start_time) / cv2.getTickFrequency()
#	rospy.loginfo(time_count)

	if red_lines is not None:
		rospy.loginfo("STOP")
		return 154 #stop
	elif right_lines is not None and left_lines is not None:
        	rospy.loginfo(error)
		if error > 150:
			error = 150
		elif error < -150:
			error = -150

		return error

	elif left_lines is not None and right_lines is None:
		rospy.loginfo("Turn Right")
		rospy.loginfo(kl)
		return 152 #turn right

	elif left_lines is None and right_lines is not None:
		rospy.loginfo("Turn Left")
		return 153 #turn let
	elif left_lines is None and right_lines is None:
		rospy.loginfo("No line")
		return 155 #no line found
	else:
		return 155 #no line found

def lane_detection():
	pub = rospy.Publisher('lane_detection', Int32, queue_size=10) #ros-lane-detection
	rospy.init_node('lane-detection',anonymous=True)

	camera = PiCamera() # Raspberry Pi Camera
	camera.resolution = (350,200)
	camera.framerate = 30 #50
	camera.contrast = 40 #30
	camera.saturation = 100 #20
	camera.brightness = 30 #40
	camera.sharpness = 0
	camera.start_preview()
	time.sleep(1)
	rawCapture = PiRGBArray(camera)
	
	rate = rospy.Rate(30) # publisher frequency
	bridge = CvBridge()

	while not rospy.is_shutdown():
		camera.capture(rawCapture, format='bgr', use_video_port=True)
		rospy.loginfo("Sending an Image Message")
		info = detect(rawCapture.array)
		pub.publish(info)
		rawCapture.truncate(0)
#		rate.sleep()

if __name__ == '__main__':
	try:
		lane_detection()
	except rospy.ROSInterruptException:
		pass
