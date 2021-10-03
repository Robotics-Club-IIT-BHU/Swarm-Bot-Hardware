#!/usr/bin/env python
 
import rospy
from geometry_msgs.msg import Point

import math
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

factory = None
servo1 = None
servo2 = None

new_goal = False
	
m = 0.5

val1=-m
val2=0

d1 = 0.001
d2 = 0.001

p = Point()
p.x = 0
p.y = 0
p.z = 0

def instruct_callback(msg):
	global p
	print("new_Data")
	p = msg
	p.z = 0
	mag = math.sqrt(p.x**2 + p.y**2)
	if mag < 0.1: ## check this parameter carefully
		new_goal = False
	else: 
		new_goal = True
	p.x /= mag
	p.y /= mag
	
def coor2ang(x, y):
	x = min(max(x, -1.5), 1.5)
	y = min(max(y, -1.5), 1.5)
	ang1 = sin(x)
	ang2 = sin(y)
	return ang1, ang2

def init():
	global factory, servo1, servo2

	factory = factory or PiGPIOFactory()
	servo1 = servo1 or Servo(14, initial_value=0, pin_factory=factory)
	servo2 = servo2 or Servo(15, initial_value=0, pin_factory=factory)


def main_loop():
	global factory, servo1, servo2, p, m, val1, val2, d1, d2

	rate = rospy.Rate(50)
	while True or not rospy.is_shutdown():
		print("I am in loop")
		try:
			if new_goal:
				v1, v2 = coor2ang(p.x, p.y)
				servo1.value = v1
				servo2.value = v2
				#new_goal = False
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
				servo1.value = 0
				servo2.value = 0
			
			rate.sleep()
		except rospy.ROSInterruptException:
			rospy.logerr("Shutting Down Servo Server")
		except rospy.ROSTimeMovedBackwardsException:
			rospy.logerr("Here goes time")
	
	servo1.detach()
	servo2.detach()
	rospy.spin()
if __name__ == "__main__":
	rospy.init_node('servo_server', anonymous=True)

	init()
	rospy.Subscriber("servo_dir", Point, instruct_callback )
	main_loop()
