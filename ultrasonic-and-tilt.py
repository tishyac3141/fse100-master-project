#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import smbus2 as smbus
import math

TRIG = 13 #27
ECHO = 12 #18
TiltPin = 40 #change this
BuzzerPin = 16    # pin11
BtnPin = 33 #GPIO 13?

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

global height
height = 0

global hasAlerted
hasAlerted = False

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
    
def store_values(x_rot, y_rot):
	global current_x_rot
	global current_y_rot
	
	current_x_rot = x_rot
	current_y_rot = y_rot


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
	
	# global isTilted
	# isTilted = False
	
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(TRIG, GPIO.OUT)
	GPIO.setup(ECHO, GPIO.IN)
	GPIO.setmode(GPIO.BOARD)		# Numbers GPIOs by physical location
	GPIO.setup(BuzzerPin, GPIO.OUT)	# Set pins' mode is output
	global Buzz						# Assign a global variable to replace GPIO.PWM 
	Buzz = GPIO.PWM(BuzzerPin, 440)	# 440 is initial frequency.
	
	GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	# GPIO.setup(TiltPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
	
	GPIO.add_event_detect(BtnPin, GPIO.BOTH, callback=detect_button, bouncetime=200)
	# GPIO.add_event_detect(TiltPin, GPIO.BOTH, callback=detect_tilt, bouncetime=200)
	

	
def button_output(x):	
	global first_pressed
	# global isTilted
	global current_x_rot
	global current_y_rot
	
	print("x_rotation = ", current_x_rot)
	print("y_rotation = ", current_y_rot)
	
	if x == 0 and abs(current_x_rot) <= 15 and abs(current_y_rot) <= 15:
		if(not(first_pressed)):
			first_pressed = True
			calibrate()	
		else:
			global height
			dis = distance()
			ratio = (height - dis) / height
			count = 0
			if(ratio <= 0.23):
				count = 1
				# Buzz.start(50)
				print('count =', count)
			elif(ratio <= 0.58):
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
	
	if(x == 1):
		isTilted = True
	else:
		isTilted = False

def detect_button(chn):
	print('detected button')
	button_output(GPIO.input(BtnPin))
	
def detect_tilt(chn):
	tilt_output(GPIO.input(TiltPin))

def distance():
	print('in distance()')
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
		
		time.sleep(0.1)
		gyro_xout = read_word_2c(0x43)
		gyro_yout = read_word_2c(0x45)
		gyro_zout = read_word_2c(0x47)

		# print ("gyro_xout : ", gyro_xout, " scaled: ", (gyro_xout / 131))
		# print ("gyro_yout : ", gyro_yout, " scaled: ", (gyro_yout / 131))
		# print ("gyro_zout : ", gyro_zout, " scaled: ", (gyro_zout / 131))

		accel_xout = read_word_2c(0x3b)
		accel_yout = read_word_2c(0x3d)
		accel_zout = read_word_2c(0x3f)

		accel_xout_scaled = accel_xout / 16384.0
		accel_yout_scaled = accel_yout / 16384.0
		accel_zout_scaled = accel_zout / 16384.0

		# print ("accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled)
		# print ("accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled)
		# print ("accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled)
		
		global current_x_rot
		global current_y_rot
		global hasAlerted

		store_values(get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled), get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
		
		global height
		if(height != 0):
			dis = distance()
			ratio = (height - dis) / height
		
			if(abs(current_x_rot) <= 20 and abs(current_y_rot) <= 20 and ratio >= 0.85):
				hasAlerted = True
				Buzz.start(50)
				
		time.sleep(0.75)
		Buzz.stop()
		
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
