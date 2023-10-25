#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

TRIG = 13
ECHO = 12
TiltPin = 40 #change this
BuzzerPin = 16    # pin11
BtnPin = 33 #GPIO 13?

# when the button is pressed for the first time
# this function will be called to measure the distance
# to the bottom of the cup/pitcher and store it
def calibrate():
	global height 
	height = distance()
	print('calibrated: ', height)


def on():
    GPIO.output(BuzzerPin, GPIO.LOW)

def off():
    GPIO.output(BuzzerPin, GPIO.HIGH)

def beep(x):
    on()
    time.sleep(x)
    off()
    time.sleep(x)


def setup():
	global first_pressed
	first_pressed = False
	
	global isTilted
	isTilted = False
	
	
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(TRIG, GPIO.OUT)
	GPIO.setup(ECHO, GPIO.IN)
	GPIO.setmode(GPIO.BOARD)		# Numbers GPIOs by physical location
	GPIO.setup(BuzzerPin, GPIO.OUT)	# Set pins' mode is output
	global Buzz						# Assign a global variable to replace GPIO.PWM 
	Buzz = GPIO.PWM(BuzzerPin, 440)	# 440 is initial frequency.
	
	GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(TiltPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
	
	GPIO.add_event_detect(BtnPin, GPIO.BOTH, callback=detect_button, bouncetime=200)
	GPIO.add_event_detect(TiltPin, GPIO.BOTH, callback=detect_tilt, bouncetime=200)

	
def button_output(x):
	global first_pressed
	global isTilted
	
	if x == 0 and not(isTilted):
		if(not(first_pressed)):
			first_pressed = True
			calibrate()	
		else:
			global height
			dis = distance()
			ratio = (height - dis) / height
			count = 0
			if(ratio <= 0.25):
				count = 1
				# Buzz.start(50)
				print('count =', count)
			elif(ratio <= 0.5):
				count = 2
				# Buzz.start(50)
				print('count =', count)
			elif(ratio <= 0.75):
				count = 3
				# Buzz.start(50)
				print('count =', count)
			else:
				count = 4
				# Buzz.start(50)
				print('count =', count)
			for i in range(0, count):
				Buzz.start(50)
				time.sleep(1)
				Buzz.stop()
				time.sleep(0.5)

	'''
	global first_pressed
	if x == 0:
		if(not(first_pressed)):
			first_pressed = True
			calibrate()	
		else:
			dis = distance()
			if(dis < 10 and dis > 2):
				Buzz.start(50)
			else:
				Buzz.stop()
		
		'''
		
		
def tilt_output(x):
	print('tilt output =', x)
	global isTilted
	
	if(x == 0):
		isTilted = True
	else:
		isTilted = False

def detect_button(chn):
	button_output(GPIO.input(BtnPin))
	
def detect_tilt(chn):
	tilt_output(GPIO.input(TiltPin))

def distance():
	GPIO.output(TRIG, 0)
	time.sleep(0.000002)

	GPIO.output(TRIG, 1)
	time.sleep(0.00001)
	GPIO.output(TRIG, 0)

	
	while GPIO.input(ECHO) == 0:
		a = 0
	time1 = time.time()
	while GPIO.input(ECHO) == 1:
		a = 1
	time2 = time.time()

	during = time2 - time1
	return during * 340 / 2 * 100

def loop():
	while True:
		dis = distance()
		print(dis, 'cm')
		print(' ')
		time.sleep(0.3)
		# if(dis < 10 and dis > 2):
			# Buzz.start(30)
		# else:
			# Buzz.stop()

def destroy():
	Buzz.stop()					# Stop the buzzer
	GPIO.output(BuzzerPin, 1)	
	GPIO.cleanup()

if __name__ == "__main__":
	setup()
	try:
		loop()
	except KeyboardInterrupt:
		destroy()
