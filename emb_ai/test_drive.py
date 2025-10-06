#!/usr/bin/env python3

import ev3dev.ev3 as ev3

mL  = ev3.LargeMotor('outA') # left motor
mR  = ev3.LargeMotor('outD') # right motor

mL.run_forever(speed_sp=100)
mR.run_forever(speed_sp=100) #moves forward