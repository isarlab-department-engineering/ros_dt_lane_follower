import time

class PID:

	def __init__(self, P=2.0,I=0.0,D=0.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

		self.Kp = P
		self.Ki = I
		self.Kd = D
		self.Derivator = Derivator
		self.Integrator = Integrator
		self.Integrator_max = Integrator_max
		self.Integrator_min = Integrator_min

		self.set_point = 0.0
		self.error = 0.0

#		self.sample_time = 0.00
#		self.current_time = time.time()
#		self.last_time = self.current_time
#
#		self.clear()
#
#	def clear(self):
#		# Clear PID computations and coefficients
#		self.SetPoint = 0.0
#		self.PTerm = 0.0
#		self.ITerm = 0.0
#		self.DTerm = 0.0
#		self.last_error = 0.0
#
#		#Windup Guard
#		self.init_error = 0.0	
#		self.windup_guard = 20.0
#	
#		self.output = 0.0

	def update(self, current_value):
		# Calculates PID value for given reference feedback
		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * (self.error - self.Derivator)	
		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min

		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def SetPoint(self,set_point):
		self.set_point = set_point
		self.Integrator = 0
		self.Derivator = 0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp = P

	def setKi(self,I):
		self.Ki = I

	def setKd(self,D):
		self.Kd = D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error
	
	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator
