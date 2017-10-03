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
		#print x,y,theta

		# stages: 1) straight line,
		#         2) semi-circle
		#         3) straight line again.

		print self.state,x,theta

		# stages 1
		if (self.state == 1):
			if (x < 1):
				self.leftMotor.run(1)
				self.rightMotor.run(1)
			elif (1 <= x):
				self.state = 2

				angle_err = theta

				self.leftMotor.setSpeed(self.left_pwm)		# 60
				self.rightMotor.setSpeed(self.left_pwm * 2)	# 120
			else:
				print "Something wrong in state 1"

		# stages 2
		if (self.state == 2):
			# clac the angle error
			if(theta >= angle_err):
				angle = theta - angle_err
			else:
				angle = theta + 2 * pi - angle_err

			if (angle < pi):
				print "Here is in state 2"
			elif (pi <= angle):
				self.state = 3
				self.leftMotor.setSpeed(self.left_pwm)
				self.rightMotor.setSpeed(self.right_pwm)
			else:
				print "Something wrong in state 2"

		# stages 3
		if (self.state == 3):
			if (x > 0):
				print "Here is in state 3"
			elif(0 >= x):		# stop
				self.state = 1	# set to default value
				self.leftMotor.run(4)
				self.rightMotor.run(4)
			else:
				print "Something wrong in state 3"


	def custom_shutdown(self):
		self.leftMotor.run(4)
		self.rightMotor.run(4)
		del self.motorhat

if __name__ == '__main__':
	rospy.init_node('tracking', anonymous = False)
	Track = Tracking()
	rospy.spin()
