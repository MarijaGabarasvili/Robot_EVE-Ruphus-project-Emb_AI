#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time 


btn = ev3.Button()

#sensors and motors used
mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outD')
colorSensor_right = ev3.ColorSensor('in4')
colorSensor_left = ev3.ColorSensor('in1')

mA.reset()
mB.reset()


#define speed and turn angles
BASE_SPEED = 100
TURN_SPEED = 180      # Speed reduction when correcting


#getting values of black and white from the color sensor
black = 5
white = 50


# Calculate threshold
THRESHOLD = (black + white) / 2
print("Threshold:", THRESHOLD)



while not btn.any():
    left_val = colorSensor_left.value()
    right_val = colorSensor_right.value()
    
    mA.run_forever(speed_sp=BASE_SPEED)
    mB.run_forever(speed_sp=BASE_SPEED)

    if left_val < THRESHOLD:
        # Line is under left sensor → turn LEFT
        mA.run_forever(speed_sp=BASE_SPEED)   # Right motor slower
        mB.run_forever(speed_sp=TURN_SPEED)   # Left motor faster
    if right_val < THRESHOLD :
        # Line is under right sensor → turn RIGHT
        mA.run_forever(speed_sp=TURN_SPEED)
        mB.run_forever(speed_sp=BASE_SPEED)
    else:
        # Both on same color → go straight
        mA.run_forever(speed_sp=BASE_SPEED)
        mB.run_forever(speed_sp=BASE_SPEED)

    time.sleep(0.01)
    
# Stop the motors
mA.stop(stop_action="brake")
mB.stop(stop_action="brake")

    
    



