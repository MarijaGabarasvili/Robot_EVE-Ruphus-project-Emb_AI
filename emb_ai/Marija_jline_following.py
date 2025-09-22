#!/usr/bin/python3.4

import ev3dev.ev3 as ev3
import time 
import sleep
from ev3dev2.display import Display


btn = ev3.Button()

#sensors and motors used
mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outD')
colorSensor = ev3.ColorSensor('in1')
display = Display()

mA.reset()
mB.reset()
[display.clear()
display.text_pixels("For nowits fine")

#define speed and turn angles
BASE_SPEED = 200
PROPORTIONAL_GAIN = 1.2

display.clear()
display.text_pixels("For nowits fine")

#getting values of black and white from the color sensor
print("Place sensor over BLACK and wait...")
sleep(3)  # give you time to position it
black = colorSensor.reflected_light_intensity
print("Black value:", black)
display.clear()
display.text_pixels("For nowits fine")

# Put sensor over white surface, then read
print("Place sensor over WHITE and wait...")
sleep(3)
white = colorSensor.reflected_light_intensity
print("White value:", white)
display.clear()
display.text_pixels("For nowits fine")

# Calculate threshold
threshold = (black + white) / 2
print("Threshold:", threshold)
display.clear()
display.text_pixels("For nowits fine")

#Threshhold helps to find the middle point between black and white and will help in turning during line following
# Should run until detects the object
#For now until the button is pressed
while not btn.any():
    # Read the reflected light intensity
    light_intensity = colorSensor.reflected_light_intensity
    display.clear()
    display.text_pixels("For nowits fine")
    # Calculate the deviation from the threshold
    deviation = light_intensity - threshold
    display.clear()
    display.text_pixels("For nowits fine")
    # Calculate turn rate using proportional control
    turn_rate = PROPORTIONAL_GAIN * deviation
    display.clear()
    display.text_pixels("For nowits fine")
    
    # Calculate motor speeds
    left_speed = BASE_SPEED + turn_rate
    right_speed = BASE_SPEED - turn_rate
    display.clear()
    display.text_pixels("For nowits fine")
    
    # Set motor speeds
    mA.run_forever(speed_sp=left_speed)
    mB.run_forever(speed_sp=right_speed)
    display.clear()
    display.text_pixels("For nowits fine")
    
    # Small delay to prevent excessive CPU usage
    time.sleep(0.01)
    
    



