#!/usr/bin/env python3

import ev3dev.ev3 as ev3
import time
from ev3dev2.sound import Sound


mG = ev3.MediumMotor('outC')
ultrasonic = ev3.UltrasonicSensor('in3') # ultrasonic sensor
sound = Sound()



# while True:
#     sound.speak("Distance is " + str(ultrasonic.distance_centimeters) + " centimeters")
#     time.sleep(0.5)

# maybe a setup for robot grabber
mG.stop_action = "coast" # gripper motor
mG.run_to_abs_pos(position_sp=1500, speed_sp=300) # open gripper
time.sleep(4)