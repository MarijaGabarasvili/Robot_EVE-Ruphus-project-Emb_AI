#!/usr/bin/python3
import ev3dev.ev3 as ev3
import time

#Hardware setup
btn = ev3.Button()
mL = ev3.LargeMotor('outA')      # left motor
mR = ev3.LargeMotor('outD')      # right motor
csR = ev3.ColorSensor('in2')     # right color sensor
csL = ev3.ColorSensor('in1')     # left color sensor
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'
gyro = ev3.GyroSensor('in4')     # gyro sensor
gyro.mode = 'GYRO-RATE' #for resetting
time.sleep(0.1)  # wait for sensor to initialize
gyro.mode = 'GYRO-ANG' #switches measurments to degrees

#motors reset
mL.reset()
mR.reset()

