#!/usr/bin/env python
 
from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()

servo1 = Servo(14, initial_value=0, pin_factory=factory)
servo2 = Servo(15, initial_value=0, pin_factory=factory)

m = 0.5

val1=-m
val2=0

d1 = 0.005
d2 = 0.005
try:
	while True:
		sleep(0.005)
		val1+=d1
		val2+=d2
		if(val1>=m or val1<=(-m)):
			d1*=-1
		else:
			servo1.value = val1
		if(val2>=m or val2<=(-m)):
			d2*=-1
		else:
			servo2.value = val2
except KeyboardInterrupt:
	print("Program Stopped")

servo1.detach()
servo2.detach()
