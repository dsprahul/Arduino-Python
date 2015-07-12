Python Script for Arduino IDE
======
Arduino code import tool for Python based on [Firmata protocol](https://github.com/firmata/arduino)  and [PyMata](https://github.com/MrYsLab/PyMata)


## Overview:
The host computer and Arduino will communicate via Firmata protocol.
Firmware available in 'ArduinoSketch' folder should be loaded on to Arduino.
PyMata should be installed next as per instructions provided is 'documentaion' folder.
Arduino Sketches should be written in 'Arduino_python_ide.py'.


## Interface:
```python 
import numpy as np
import time
import math
import serial
import sys
import signal
from PyMata.pymata import PyMata


# ... initiating Arduino ...
# create a PyMata instance
board = PyMata("/dev/ttyACM0")

# ... Basis definitions for Stepper, servo, analog/digital io ... 

# Servo
def servo_init(SERVO_MOTOR) :
  	# send the arduino a firmata reset
	board.reset()
	# servo attached to this pin
	SERVO_MOTOR = 5
	# configure the servo
	board.servo_config(SERVO_MOTOR)

def servo(SERVO_MOTOR, angle):
	# move the servo to 'angle' degrees
	board.analog_write(SERVO_MOTOR, angle)

# Stepper
def stepper_init(N):
	# send the arduino a firmata reset
	board.reset()
	# configure the stepper to use pins 9.10,11,12 and specify 'N' steps per revolution
	firmata.stepper_config(N, [12, 11, 10, 9])
	time.sleep(.5)
	# ask Arduino to return the stepper library version number to PyMata
	firmata.stepper_request_library_version()
	time.sleep(.5)
	print("Stepper Library Version",)
	print(firmata.get_stepper_version())

def stepper(speed,n):
	# move motor #0 'n' steps forward at a speed of 'speed'
	firmata.stepper_step(speed, n)

# Callback function (temp)
force = 0
def print_analog(data):
	global force
	force = data[2]
	
	
	

# digitalWrite()
def digitalWrite(BOARD_LED, val) :
	# Setting pinMode for pin BOARD_LED
	board.set_pin_mode(BOARD_LED, board.OUTPUT, board.DIGITAL)
	# Set the output to val
	board.digital_write(BOARD_LED, val)

# analogRead()
def analog_init(analogPin) :
	board.set_pin_mode(analogPin, board.INPUT, board.ANALOG)

def analogRead( analogPin):
	force = board.analog_read( analogPin)
	return force

# delay()
def delay(num) :
	num = float (num)
	time.sleep(num/1000.0)

# handler definition
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!!!!')
    if board is not None:
        board.reset()
    sys.exit(0)

# And throw error if necessary 
signal.signal(signal.SIGINT, signal_handler)





# ... Write your Arduino Sketch in infinite loop here ...

# .....!!!!! Setup() here !!!!!.....
servo_init(5)
analog_init(1)
delay(100)




# .....!!!!! loop() here !!!!!.....
print "Entering loop(), get ready for some action "
while (True) :
 
 # Blink example
 digitialWrite(13,1)
 delay(1000) # time in ms, just like as in Arduino IDE
 digitialWrite(13,0)
 delay(1000)
 
 

		
