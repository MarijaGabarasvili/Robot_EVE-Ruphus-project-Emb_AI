#!/usr/bin/env python3

import ev3dev.ev3 as ev3
import time
from ev3dev2.sound import Sound


mG = ev3.MediumMotor('outC')
ultrasonic = ev3.UltrasonicSensor('in3') # ultrasonic sensor
sound = Sound()


while ultrasonic.distance_centimeters > 20:
    sound.speak("Can is not detected")
    
sound.speak("Can is detected")

#maybe a setup for robot grabber
# mG.stop_action = "coast" # gripper motor
# mG.run_to_abs_pos(position_sp=1500, speed_sp=300) # open gripper
# time.sleep(2)
# mG.run_to_abs_pos(position_sp=-2000, speed_sp=300) # close gripper
# time.sleep(2)

