#!/usr/bin/python3.4

import ev3dev.ev3 as ev3
import time 
import sleep

btn = ev3.Button()

mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outB')

THRESHOLD_LEFT = 30
THRESHOLD_RIGHT = 350

BASE_SPEED = 30
TURN_SPEED = 80

TouchSensor = ev3.TouchSensor('in3')