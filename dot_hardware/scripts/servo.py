#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import Point

import math
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

serObj = None

	
def coor2ang(x, y):
	x = min(max(x, -1.5), 1.5)
	y = min(max(y, -1.5), 1.5)
	ang1 = math.sin(x)
	ang2 = math.sin(y)
	return ang1, ang2

class ServoControlRos:
	m = 0.5
	d1 = 0.001
	d2 = 0.001
	new_goal = False
	def __init__(self,pi_shift=False):
		self.p = Point()
		self.factory = PiGPIOFactory()
		self.servo1 = Servo(14, initial_value=0, pin_factory=self.factory)
		self.servo2 = Servo(15, initial_value=0, pin_factory=self.factory)		
		if pi_shift:
			self.val1 = -self.m
		else:
			self.val1 = 0
		self.val2 = 0
		self.tar1 = self.val1
		self.tar2 = self.val2

	def setTarget(self,x,y):
		self.tar1, self.tar2 = coor2ang(x, y)
	def control(self):
		self.val1 = 0.8*self.val1 + 0.2*self.tar1
		self.val2 = 0.8*self.val2 + 0.2*self.tar2
		self.servo1.value = self.val1
		self.servo2.value = self.val2
	def is_reached(self):
		if(abs(self.tar1-self.val1) + abs(self.tar2-self.val2))<0.05:
			return True
		else:
			return False

def instruct_callback(msg):
	global serObj
	# print("new_Data")
	serObj.p = msg
	serObj.p.z = 0
	mag = math.sqrt(serObj.p.x**2 + serObj.p.y**2)
	if mag < 0.1: ## check this parameter carefully
		serObj.new_goal = False
	else: 
		serObj.new_goal = True
	serObj.p.x /= mag
	serObj.p.y /= mag



def main_loop():
	global serObj

	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		#print("I am in loop")
		try:
			if serObj.new_goal:
				serObj.setTarget(serObj.p.x,serObj.p.y)
				serObj.control()
				if(serObj.is_reached()):
					serObj.new_goal = False
			else: 
# 				val1+=d1
# 				val2+=d2
# 				if(val1>=m or val1<=(-m)):
# 					d1*=-1
# 				else:
# 					servo1.value = val1
# 				if(val2>=m or val2<=(-m)):
# 					d2*=-1
# 				else:
# 					servo2.value = val2
				serObj.setTarget(0,0)
				serObj.control()
			
			rate.sleep()
		except rospy.ROSInterruptException:
			rospy.logerr("Shutting Down Servo Server")
		except rospy.ROSTimeMovedBackwardsException:
			rospy.logerr("Here goes time")
	
	serObj.servo1.detach()
	serObj.servo2.detach()
	
if __name__ == "__main__":
	rospy.init_node('servo_server', anonymous=True)

	serObj = ServoControlRos()
	rospy.Subscriber("servo_dir", Point, instruct_callback )
	main_loop()
