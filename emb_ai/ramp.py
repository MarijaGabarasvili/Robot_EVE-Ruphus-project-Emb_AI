#!/usr/bin/python3
import ev3dev.ev3 as ev3
import time

# -------- Hardware --------
gyro = ev3.GyroSensor('in4')     # right sensor (IN4-et használja)
gyro.mode = 'GYRO-RATE'
time.sleep(0.1)  # wait for sensor to initialize
gyro.mode = 'GYRO-ANG'
mL = ev3.LargeMotor('outA')      # left motor
mR = ev3.LargeMotor('outD')     # right motor

BASE_SPEED = 150
UP_SPEED = 300
DOWN_SPEED = 120
POLARITY = -1
ANGLE_THRESHOLD = 10  # degrees
start = time.time()
RUN_TIME = 60 + start  # seconds

def go_up():
    mL.run_forever(speed_sp=UP_SPEED*POLARITY)
    mR.run_forever(speed_sp=UP_SPEED*POLARITY)
    # return

def go_down():
    mL.run_forever(speed_sp=DOWN_SPEED*POLARITY)
    mR.run_forever(speed_sp=DOWN_SPEED*POLARITY)
    # return

while start < RUN_TIME:
    angle = gyro.value()
    if angle > ANGLE_THRESHOLD:
        go_down()
    elif angle < 0:
        go_up()
    else:
        mL.run_forever(speed_sp=BASE_SPEED*POLARITY)
        mR.run_forever(speed_sp=BASE_SPEED*POLARITY)
