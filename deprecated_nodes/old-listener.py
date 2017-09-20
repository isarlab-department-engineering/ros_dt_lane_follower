#!/usr/bin/env python

### DEPRECATED !!!! 

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int32
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit
import RPi.GPIO as GPIO

I=0
last_error = 0
def calcPID(error, Kp, Ki, Kd):
       	global last_error
 	global I
#	        I = 0
       	P = error
       	if P > 100:
               	P = 100
       	elif P < -100:
               	P = -100

        I = I + error
#        if I > 500 or I < -500:
	if I > 300:
		I = 300
#			if error < 10 and error > -10:
#				I = I - I/2
	elif I < -300:
		I = -300
#			if error < 10 and error > -10:
#				I = I - I/2
	if error < 10 and error > -10:
		I = I - I/2         
#		if error < 10 and error > -10 :
 #               	I = I - I/2

	D = error - last_error
	PID = int(Kp*P + Ki*I + Kd*D)
	last_error = error

	return PID

class lane_detection:
		
	def __init__(self):

		#motor HAT setup
		self.mh = Adafruit_MotorHAT(addr=0x60)

		# LED setup
#		GPIO.setmode(GPIO.BCM)
#		GPIO.setup(15, GPIO.OUT)
#		GPIO.setwarnings(False)

		def turnOffMotors():
			self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
			self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
			atexit.register(turnOffMotors)

		#setup 2 motors
		self.m1 = self.mh.getMotor(1)
		self.m2 = self.mh.getMotor(2)

		#setup motors' speed
		self.speed = 120
		self.motorBalance = -3#3
		self.m1.setSpeed(self.speed + self.motorBalance) #left motor
		self.m2.setSpeed(self.speed) #right motor

		rospy.Subscriber("lane_detection", Int32, self.callback)

#	def calc_pd(error,Kp,Kd):
#		global last_error
#		P = error
#		D = error - last_error
#		PD = Kp*P + Kd*D
#		last_error = error	
#
#		return PD

	def callback(self,data):

		rospy.loginfo(rospy.get_caller_id() + " Movement direction: %s", data.data)

		input = data.data
		self.m1.setSpeed(self.speed + self.motorBalance)
		self.m2.setSpeed(self.speed)

		PID = calcPID(input,0.5,0.0005,0.005)
		print(PID)
		if input == 0:
			self.m1.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.run(Adafruit_MotorHAT.FORWARD)
			rospy.loginfo("Forward")
		elif(input > 0 and input < 150):
	                self.m1.run(Adafruit_MotorHAT.FORWARD)
        	        self.m2.run(Adafruit_MotorHAT.FORWARD)
#			if (PID > 25):
#				PID = 25
			self.m1.setSpeed(self.speed+self.motorBalance+PID)
			rospy.loginfo(PID)
		elif(input < 0):
	                self.m1.run(Adafruit_MotorHAT.FORWARD)
        	        self.m2.run(Adafruit_MotorHAT.FORWARD)
#			if (input < -25):
#				input = -25
			self.m2.setSpeed(self.speed-PID)
			rospy.loginfo(PID)
		elif(input == 152):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.setSpeed(self.speed/2)	#55
#			self.m2.setSpeed(self.speed-20)		
		elif(input == 153):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.run(Adafruit_MotorHAT.FORWARD)
                        self.m1.setSpeed(self.speed/2)
#			self.m1.setSpeed(self.speed-20)
		elif(input == 154):
#			self.m1.run(Adafruit_MotorHAT.RELEASE)
#			self.m2.run(Adafruit_MotorHAT.RELEASE)
			time.sleep(0.5)
                        self.m1.run(Adafruit_MotorHAT.RELEASE)
                        self.m2.run(Adafruit_MotorHAT.RELEASE)
		elif(input == 155):
			self.m1.run(Adafruit_MotorHAT.RELEASE)
			self.m2.run(Adafruit_MotorHAT.RELEASE)
			rospy.loginfo("No line found")

		if(input == "veldx"):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
			self.m2.run(Adafruit_MotorHAT.FORWARD)
			self.m1.setSpeed(self.speed+5)

                if(input == "velsx"):
                        self.m1.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.setSpeed(self.speed+5)
	
		if(input == "velsx+"):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.setSpeed(self.speed+10)
		if(input == "veldx+"):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
                        self.m2.run(Adafruit_MotorHAT.FORWARD)
                        self.m1.setSpeed(self.speed+10)

		#move forward
		if(input == "forward"):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
			self.m2.run(Adafruit_MotorHAT.FORWARD)
#			GPIO.output(15, True)

		#move backward
		if(input == "backward"):
			self.m1.run(Adafruit_MotorHAT.BACKWARD)
			self.m2.run(Adafruit_MotorHAT.BACKWARD)
#			GPIO.output(15, True)
	
		#turn right
		if(input == "turn_right"):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
			self.m2.run(Adafruit_MotorHAT.FORWARD)
			self.m2.setSpeed(self.speed-40)
#			self.m2.run(Adafruit_MotorHAT.RELEASE)
#			time.sleep(0.075)
#			self.m2.run(Adafruit_MotorHAT.FORWARD)
#			GPIO.output(15, True)
	
		#turn left
		if(input == "turn_left"):
			self.m1.run(Adafruit_MotorHAT.FORWARD)
			self.m1.setSpeed(self.speed + self.motorBalance - 40)
#			self.m1.run(Adafruit_MotorHAT.RELEASE)
			self.m2.run(Adafruit_MotorHAT.FORWARD)
#			time.sleep(0.075)
#			self.m1.run(Adafruit_MotorHAT.FORWARD)
#			GPIO.output(15, True)

		#stop
		if(input == "stop"):
			self.m1.run(Adafruit_MotorHAT.RELEASE)
			self.m2.run(Adafruit_MotorHAT.RELEASE)
			time.sleep(0.2)
			self.m1.run(Adafruit_MotorHAT.FORWARD)
			self.m2.run(Adafruit_MotorHAT.FORWARD)
#			GPIO.output(15, True)
	
		#no line found
		if(input == "end"):
			self.m1.run(Adafruit_MotorHAT.RELEASE)
			self.m2.run(Adafruit_MotorHAT.RELEASE)
#			GPIO.output(15, False)

		#shutdown
		if(input == "shutdown"):
			self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
			self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)

#		GPIO.cleanup()

def main(args):
	lane_det = lane_detection()
	rospy.init_node('lane_detection', anonymous=True)
	#rospy.Subscriber("lane_detection",String,callback)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
