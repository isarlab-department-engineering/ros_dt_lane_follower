#!/usr/bin/env python
from __future__ import print_function
import rospy, sys, cv2, time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

I = 0
last_error = 0
twistMessage = Twist()
twistMessage.linear.x = 0
twistMessage.linear.y = 0
twistMessage.linear.z = 0
twistMessage.angular.x = 0
twistMessage.angular.y = 0
twistMessage.angular.z = 0
pub = rospy.Publisher("follow_topic",Twist,queue_size=1)

def calculatePID(error,Kp,Ki,Kd):
	global last_error, I
	
	P = error
	if P > 100:
		P = 100
	elif P < -100:
		P = -100

	I = I + error
	
	if I > 300:
		I = 300
	elif I < -300:
		I = -300

	if error < 10 and error > -10:
		I = I - I/2

	D = error - last_error

	PID = int(Kp*P + Ki*I + Kd*D)

	last_error = error
	
	return PID

def turnOffMotors():
	# DEBUG VERSION TO FIX
	twistMessage.linear.x = 0
	twistMessage.linear.y = 0
	pub.publish(twistMessage)

def setSpeed(speed1,speed2):
	if speed1 == 0 and speed2 == 0:
		turnOffMotors()
	else:
		twistMessage.linear.x = speed1
		twistMessage.linear.y = speed2
		pub.publish(twistMessage)

def callback(data):

	error = data.data
	speed2 = 120
	motorBalance = -3
	speed1 = speed2 + motorBalance

	PID = calculatePID(error,0.6,0.0005,0.005)
#	rospy.loginfo(error)

	if error == 0:
		setSpeed(speed1,speed2)

	elif (error > 0 and error < 150):
		setSpeed(speed1+PID,speed2)

	elif (error < 0):
		setSpeed(speed1,speed2-PID)

	elif error == 152:
		setSpeed(speed1,speed2/2)
#		setSpeed(speed1,70)

	elif error == 153:
		setSpeed(speed1/2,speed2)
#		setSpeed(70,speed2)

	else:
		if error == 154:
			time.sleep(0.5)
		turnOffMotors()

def lane_controller():
	rospy.init_node('lane_controller', anonymous=True)
	rospy.Subscriber('lane_detection', Int32, callback)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	lane_controller()
