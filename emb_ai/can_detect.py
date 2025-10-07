#!/usr/bin/python3


#Logic:
#Turn around after finishing line following
#Turn around until the can is detected
#Move forward until the can is close enough
#Grasp the can
#Go back to the line and start following again

import ev3dev.ev3 as ev3
import time
from ev3dev2.sound import Sound


mL  = ev3.LargeMotor('outA')      # left motor
mR  = ev3.LargeMotor('outD')      # right motor
mG = ev3.MediumMotor('outC')     # gripper motor

ultrasonic = ev3.UltrasonicSensor('in3') # ultrasonic sensor
MODE_US_DIST_CM = 'US-DIST-CM' #measures distance in cm
ultrasonic.mode = MODE_US_DIST_CM
sound = Sound()

#So it already starts searching for a can with an open gripper
mG.stop_action = "coast" # gripper motor
mG.run_to_abs_pos(position_sp=1500, speed_sp=300) # open gripper
time.sleep(1)



# Detecting can
def can_detect():
    sound.speak("Searching for can")
    #s_time = time.time()
    #spin_duration = 2 # how much seconds to spin before moving forward
    while ultrasonic.distance_centimeters > 10:
        # Firstly turn on 45 degrees one side to look for a can
        mL.run_to_rel_pos(position_sp=100, speed_sp=200)
        mR.run_to_rel_pos(position_sp=-100, speed_sp=200)
        time.sleep(1) #moves forward for 0.5 seconds
        # Turns theother side for 45 degrees to detect a can
        mL.run_to_rel_pos(position_sp=-200, speed_sp=200)
        mR.run_to_rel_pos(position_sp=100, speed_sp=200)
        time.sleep(1) #moves forward for 0.5 seconds
        
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    sound.speak("Can detected")

def grab_can():
    sound.speak("Grabbing can")
    mG.run_to_abs_pos(position_sp=-1900, speed_sp=300) # close gripper
    time.sleep(1)
    sound.speak("Can grabbed")
    
    
can_detect()
grab_can()
    