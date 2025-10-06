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
sound = Sound()


# Detecting can
def can_detected():
    sound.speak("Searching for can")
    s_time = time.time()
    spin_duration = 2 # how much seconds to spin before moving forward
    while ultrasonic.value()>100:
        el_time = time.time() - s_time
        # if el_time < spin_duration:
        #     mL.run_forever(speed_sp=100)
        #     mR.run_forever(speed_sp=-100) #spins
        # else:
        mL.run_forever(speed_sp=100)
        mR.run_forever(speed_sp=100) #moves forward
        time.sleep(1) #moves forward for 0.5 seconds
        s_time = time.time() #reset timer
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    sound.speak("Can detected")

def grab_can():
    sound.speak("Grabbing can")
    mG.run_to_abs_pos(position_sp=500, speed_sp=300) # open gripper
    time.sleep(1)
    mG.run_to_abs_pos(position_sp=-200, speed_sp=300) # close gripper
    time.sleep(1)
    sound.speak("Can grabbed")
    
    
can_detected()
grab_can()
    