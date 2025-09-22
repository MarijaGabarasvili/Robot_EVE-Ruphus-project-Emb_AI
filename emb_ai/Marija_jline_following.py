#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time 
from ev3dev2.display import Display


btn = ev3.Button()

#sensors and motors used
mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outD')
colorSensor = ev3.ColorSensor('in1')
display = Display()

mA.reset()
mB.reset()


#define speed and turn angles
BASE_SPEED = 50
PROPORTIONAL_GAIN = 2


#getting values of black and white from the color sensor
black = 5
white = 50


# Calculate threshold
threshold = (black + white) / 2
print("Threshold:", threshold)


#Threshhold helps to find the middle point between black and white and will help in turning during line following
# Should run until detects the object
#For now until the button is pressed
while not btn.any():
    # Read the reflected light intensity
    light_intensity = colorSensor.reflected_light_intensity
    # Calculate the deviation from the threshold
    deviation = light_intensity - threshold

    
    # Calculate turn rate using proportional control
    turn_rate = PROPORTIONAL_GAIN * deviation    
    
    # Calculate motor speeds
    left_speed = BASE_SPEED + turn_rate
    right_speed = (BASE_SPEED - turn_rate)*1.2
    
    display.clear()
    display.text_pixels("For now its fine")
    
    # Set motor speeds
    mA.run_forever(speed_sp=right_speed*(-1))
    mB.run_forever(speed_sp=left_speed*(-1))
    
    display.clear()
    display.text_pixels("For now its fine")
    
    # Small delay to prevent excessive CPU usage
    time.sleep(0.01)
    
# Stop the motors
mA.stop(stop_action="brake")
mB.stop(stop_action="brake")

    
    



