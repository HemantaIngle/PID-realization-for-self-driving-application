#!/usr/bin/env python
import RPi.GPIO as GPIO, time
Program 1- motorcontrol.py
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16,GPIO.OUT) # Steps
GPIO.setup(11,GPIO.OUT) # Direction
p =GPIO.PWM(16,300) #500hz waveform on 16 ie steps
GPIO.setwarnings(False)

#var=1
#while var == 1:
def spinmotor (direction, num_steps):
        GPIO.output(11, direction)
        while num_steps>0:
                p.start(1) #Start providing the pulse
                time.sleep(0.1)
                num_steps-= 1 #no of stepps = no of steps-1
        p.stop() #stop the loop
        GPIO.cleanup()
        return True
direction_input = raw_input('input o or c:\n' )
num_steps =input('No of Steps:\n')
if direction_input =='c':
        spinmotor(False, num_steps)
else:
        spinmotor(True, num_steps)

#var=1
#while var==1:
#       direction_input=1
#       num_steps =input=250
#       if direction_input == 1:
#       spinmotor(False, num_steps)
#       else:
#               spinmotor(True,num_steps"



Program 2- Stepper2.py

# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15


# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

# Or create an ADS1015 ADC (12-bit) instance.
#adc = Adafruit_ADS1x15.ADS1015()

# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1

print('Reading ADS1x15 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>6}|'.format(*range(4)))
print('-' * 37)
# Main loop.
while True:
    # Read all the ADC channel values in a list.
    values = [0]*4
    for i in range(4):
        # Read the specified ADC channel using the previously set gain value.
        values[i] = adc.read_adc(i, gain=GAIN)
        # Note you can also pass in an optional data_rate parameter that controls
        #


ADC conversion time (in samples/second). Each chip has a different
        # set of allowed data rate values, see datasheet Table 9 config register
        # DR bit values.
        #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        # Each value will be a 12 or 16 bit signed integer value depending on the
        # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
    # Print the ADC values.
    print('| {0:>6} |'.format(*values))
    # Pause for half a second.
time.sleep(0.5)





import RPi.GPIO as GPIO, time
import sys

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)

p = GPIO.PWM(16, 100)
GPIO.output(11,GPIO. LOW)
p.start(1)
time.sleep(1)

GPIO.output(11,GPIO. HIGH)
p.start(1)
time.sleep(1)



 
#def SpinMotor(dire):
#    p.ChangeFrequency(300)
#    GPIO.output(11,dire)





PROGRAM #3
sudo python countingsteps.py left 1600
#Step 0: Preamble
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#Program Title  : countingsteps.py 
#Code Written by: Salty Scott
#Current Project: www.rowboboat.com
#This code is a very basic example of using python to control a spark fun
# easy driver.  The spark fun easy driver that I am using in this example
# is connected to a 42HS4013A4 stepper motor and my raspberry pi.  Pin 23
# is the direction control and pin 24 is the step control.  I am using
# these components in the www.rowboboat.com project version 2.0 and I
# hope someone finds this a useful and simple example.
# This program expects two arguments: direction and steps
# Example usage: sudo python easy_stepper.py left 1600
# The above example would turn a 200 step motor one full revolution as by
# default the easy driver 4.4 is in 1/8 microstep mode.
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
#Step 1: Import necessary libraries 
#------------------------------------------------------------------------
#------------------------------------------------------------------------
import sys
import RPi.GPIO as gpio #https://pypi.python.org/pypi/RPi.GPIO more info
import time
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 #Step 2: Read arguments 
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#read the direction and number of steps; if steps are 0 exit 
try: 
    direction = sys.argv[1]
    steps = int(float(sys.argv[2]))
except:
    direction = 'left' ## new line to set default direction
    steps = 0
 
#print which direction and how many steps 
print("You told me to turn %s %s steps.") % (direction, steps)
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
#Step 3: Setup the raspberry pi's GPIOs
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#use the broadcom layout for the gpio
gpio.setmode(gpio.BCM)
#GPIO23 = Direction
#GPIO24 = Step
gpio.setup(23, gpio.OUT)
gpio.setup(24, gpio.OUT)
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 4: Set direction of rotation
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#set the output to true for left and false for right
if direction == 'left':
    gpio.output(23, True)
elif direction == 'right':
    gpio.output(23, False)
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 5: Setup step counter and speed control variables
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#track the numebr of steps taken
StepCounter = 0
 
#waittime controls speed
WaitTime = 0.01
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 6: Let the magic happen
#------------------------------------------------------------------------
#------------------------------------------------------------------------
# Start main loop
while StepCounter < steps:
 
    #turning the gpio on and off tells the easy driver to take one step
    gpio.output(24, True)
    gpio.output(24, False)
    StepCounter += 1
 
    #Wait before taking the next step...this controls rotation speed
    time.sleep(WaitTime)
#------------------------------------------------------------------------
#------------------------------------------------------------------------
 
 
#Step 7: Clear the GPIOs so that some other program might enjoy them
#------------------------------------------------------------------------
#------------------------------------------------------------------------
#relase the GPIO
gpio.cleanup()
#------------------------------------------------------------------------
#------------------------------------------------------------------------




NEW PROGRAM

import RPi.GPIO as GPIO, time
import sys

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)

p = GPIO.PWM(16, 100)

GPIO.output(11,GPIO. LOW)
p.start(1)
time.sleep(1) #1s

GPIO.output(11,GPIO. HIGH)
p.start(1)
time.sleep(1) #1s

PROGRAM FOR ADC


import time
import Adafruit_ADS1x15
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1

print('Reading ADS1x15 values, press Ctrl-C to quit...')
print('| {0:>6}|'.format(*range(4)))
print('-' * 37)

threshold = 5

while True:
    # Read all the ADC channel values in a list.
    #values = [0]*1 #array of 4 elements
    #for i in range(4):
        # Read the specified ADC channel using the previously set gain value.
        #values[i] = adc.read_adc(i, gain=GAIN)
        # Note you can also pass in an optional data_rate parameter that controls
        # the ADC conversion time (in samples/second). Each chip has a different
        # set of allowed data rate values, see datasheet Table 9 config register
        # DR bit values.
        #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        # Each value will be a 12 or 16 bit signed integer value depending on the
        # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
    # Print the ADC values.
    values = adc.read_adc(i, gain=GAIN)
    #print('| {0:>6} |'.format(*values))

    if(raw_input() == 'q')
	break


    # Pause for half a second.
    print('ADC value', values)
    if (values >= threshold):
	GPIO.output(11,GPIO. HIGH)
	p.start(1)
	time.sleep(1) #1s
    elif (values < threshold):
        GPIO.output(11,GPIO. LOW)
	p.start(1)
	time.sleep(1) #1s
time.sleep(0.5)



WORKING-


import Adafruit_ADS1x15
import RPi.GPIO as GPIO, time
import sys

adc = Adafruit_ADS1x15.ADS1115()
time.sleep(0.5)
GAIN = 1
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
p = GPIO.PWM(16, 300)
#def SpinMotor(dir):
#   GPIO.output(11,dir)
#   p.start(1)
#   time.sleep(0.1)
#   return True

print('Reading ADS1x15 values, press Ctrl-C to quit...')
print('{0:>6}'.format(*range(1)))

value = [0]*1

while True:
        value[0] = adc.read_adc(0, gain=GAIN)
        print('{0:>6}'.format(*value))
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11, GPIO.OUT)
        if value[0]>=6000:
                GPIO.output(11, GPIO.HIGH)
        else:
                GPIO.output(11, GPIO.LOW)
        p.start(1)
        time.sleep(1)
p.stop()
GPIO.cleanup()


