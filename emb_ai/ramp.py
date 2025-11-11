#!/usr/bin/python3
import ev3dev.ev3 as ev3

# -------- Hardware --------
gyro = ev3.ColorSensor('in4')     # right sensor (IN4-et használja)
gyro.mode = 'GYRO-RATE'
