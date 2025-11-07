#!/usr/bin/env python3

import ev3dev.ev3 as ev3
import time

gyro = ev3.GyroSensor()
gyro.mode = 'GYRO-ANG'
mL = ev3.LargeMotor('outA')
mR = ev3.LargeMotor('outD')

BASE_SPEED = 200
BOOST_SPEED = 300
ANGLE_THRESHOLD = 5  # degrees
FLAT_THRESHOLD = 2  # degrees
