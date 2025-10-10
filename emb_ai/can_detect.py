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

distance_v = [] #list to store distances and than compare them  to find the smalest one, to center the car

# Detecting can
def can_detect():
    sound.speak("Searching for can")
    for i in range(-4, 5):
        #turning from -90 to 90 degrees
        mL.run_to_rel_pos(position_sp=i*90, speed_sp=100)
        mR.run_to_rel_pos(position_sp=-i*90, speed_sp=100)
        mL.wait_while('running')
        mR.wait_while('running')
        time.sleep(0.1)
        
        #measuring distance and adding to a list
        dist = ultrasonic.distance_centimeters
        distance_v.append(dist)

        #find the best step and best dist
    b_step, b_dist = min(enumerate(distance_v), key=lambda x: x[1])
    return b_step, b_dist
        
    

def grab_can():
    sound.speak("Grabbing can")
    mG.run_to_abs_pos(position_sp=-2000, speed_sp=300) # close gripper
    time.sleep(5)
    sound.speak("Can grabbed")

def move_back():
    exec(open("final_line_follower.py").read())

def move_can():
    step, dist = can_detect()
    #turning to the can
    mL.run_to_rel_pos(position_sp=step*90, speed_sp=100)
    mR.run_to_rel_pos(position_sp=-step*90, speed_sp=100)
    mL.wait_while('running')
    mR.wait_while('running')
    if dist > 8: # if dist is more than 8 cm, than move forward
        time.sleep(0.5)
        mL.run_forever(speed_sp=200)
        mR.run_forever(speed_sp=200)

try:
    move_can() # searches for can and than moves to it
    grab_can() #grabs can
    #hope for the best
    
except KeyboardInterrupt:
    print("Stopping all motors...")
    for m in (mL, mR, mG):
        m.stop(stop_action='brake')
    sound.speak("Emergency stop")
    