#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time
from ev3dev2.sound import Sound


mL  = ev3.LargeMotor('outA')      # left motor
mR  = ev3.LargeMotor('outD')      # right motor
mG = ev3.MediumMotor('outC')     # gripper motor

ultrasonic = ev3.UltrasonicSensor('in3') # ultrasonic sensor
sound = Sound()

#Resetting motors 
mL.stop(stop_action="brake")
mR.stop(stop_action="brake")
mG.stop(stop_action="brake")

#So it already starts searching for a can with an open gripper
mG.stop_action = "coast" # it stops if there is resistance in motors
mG.run_to_abs_pos(position_sp=1500, speed_sp=300) # open gripper
time.sleep(4)

# Detecting can
def can_detect():
    sound.speak("Searching for can")
    can = False
    #s_time = time.time()
    #spin_duration = 2 # how much seconds to spin before moving forward
    while not can:
        # Firstly turn on 45 degrees one side to look for a can
        mL.run_to_rel_pos(position_sp=100, speed_sp=200)
        mR.run_to_rel_pos(position_sp=-100, speed_sp=200)
        mL.wait_while('running')
        mR.wait_while('running')
        time.sleep(0.1) #moves forward for 2 seconds
        if ultrasonic.distance_centimeters < 20:
            can = True
            break
        # Turns theother side for 45 degrees to detect a can
        mL.run_to_rel_pos(position_sp=-200, speed_sp=200)
        mR.run_to_rel_pos(position_sp=100, speed_sp=200)
        mL.wait_while('running')
        mR.wait_while('running')
        time.sleep(0.1) #moves forward for 0.1 seconds
        if ultrasonic.distance_centimeters < 20:
            can = True
            break
        # Turn back to the original position
        mL.run_to_rel_pos(position_sp=100, speed_sp=200)
        mR.run_to_rel_pos(position_sp=-100, speed_sp=200)
        mL.wait_while('running')
        mR.wait_while('running')
        time.sleep(0.1) #moves forward for 0.1 seconds
        mL.run_forever(speed_sp=200)
        mR.run_forever(speed_sp=200)
        time.sleep(1) # moves forward for 1 seconds

    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    sound.speak("Can detected")

    # drive toward can
    while ultrasonic.distance_centimeters > 5: # drive until 5 cm away from can
        mL.run_forever(speed_sp=200)
        mR.run_forever(speed_sp=200)
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    grab_can()

def grab_can():
    sound.speak("Grabbing can")
    mG.run_to_abs_pos(position_sp=-2000, speed_sp=300) # close gripper
    time.sleep(5)
    sound.speak("Can grabbed")

def move_back():
    exec(open("final_line_follower.py").read())
        

can_detect()
    