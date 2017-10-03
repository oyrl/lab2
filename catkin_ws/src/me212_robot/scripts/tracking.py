#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT
import rospy
from std_msgs.msg import Float64MultiArray
from math import pi, radians
class Tracking:
	def __init__(self):
		self.node_name = rospy.get_name()	
		self.state = 1
		self.trig = None
		self.motorhat = Adafruit_MotorHAT(addr= 0x60)
		self.leftMotor 	= self.motorhat.getMotor(1)
		self.rightMotor = self.motorhat.getMotor(2)
		self.right_pwm = 60
		self.left_pwm = 60
		self.leftMotor.setSpeed(self.left_pwm)
		self.rightMotor.setSpeed(self.right_pwm)
		self.subPosition=rospy.Subscriber("/serial_node/odometry",Float64MultiArray,self.cbPosition)

		rospy.on_shutdown(self.custom_shutdown)
		rospy.loginfo("[%s] Initialized!" %self.node_name)
	def cbPosition(self,msg):
		x     = msg.data[0]
		y     = msg.data[1]
		theta = msg.data[2]
		theta = theta % (2* pi)
		print x,y,theta

		# stages: 1) straight line,
		#         2) semi-circle
		#         3) straight line again.

		# stages 1
		if (self.state==1):
			if (x<=1):
				self.leftMotor.run(1)
				self.rightMotor.run(1)
			if (x>1):
				self.state=2
				self.leftMotor.setSpeed(self.left_pwm)		# 60
				self.rightMotor.setSpeed(self.left_pwm*2)	# 120

		# stages 2
		if (self.state==2):
			if (theta<=pi):
				self.leftMotor.setSpeed(self.left_pwm)		# 60
				self.rightMotor.setSpeed(self.left_pwm*2)	# 120
			if (theta>pi):
				self.state=3

		# stages 3
		if (self.state==3):
			if (x>=0):
				self.leftMotor.setSpeed(self.left_pwm)
				self.rightMotor.setSpeed(self.right_pwm)

			else:		# stop
				# self.state = 
				self.leftMotor.run(4)
				self.rightMotor.run(4)



	def custom_shutdown(self):
		self.leftMotor.run(4)
		self.rightMotor.run(4)
		del self.motorhat

if __name__ == '__main__':
	rospy.init_node('tracking', anonymous = False)
	Track = Tracking()
	rospy.spin()
