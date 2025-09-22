import ev3dev.ev3 as ev3
import time 
import sleep

btn = ev3.Button()

mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outB')

#getting values of black and white from the color sensor
colorSensor = ev3.ColorSensor('in2')
if(colorSensor == ev3.ColorSensor.COLOR_black):
    black = colorSensor.reflected_light_intensity
if(colorSensor == ev3.ColorSensor.COLOR_white):
    white = colorSensor.reflected_light_intensity






BASE_SPEED = 30
TURN_SPEED = 80

TouchSensor = ev3.TouchSensor('in3')