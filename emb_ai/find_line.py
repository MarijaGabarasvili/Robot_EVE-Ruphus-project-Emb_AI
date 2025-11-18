#!/usr/bin/python3
import ev3dev.ev3 as ev3
import time
import subprocess

# -------- Hardware --------
btn = ev3.Button()
mL  = ev3.LargeMotor('outA')      # left motor
mR  = ev3.LargeMotor('outD')      # right motor
csR = ev3.ColorSensor('in4')      # right sensor
csL = ev3.ColorSensor('in1')      # left sensor
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.stop(stop_action="brake")
mR.stop(stop_action="brake")


 

BLACK, WHITE = 10, 50
THRESHOLD    = (BLACK + WHITE) / 4.0
DETECT_MARG  = 15 
BASE_SPEED   = 70   

vL = csL.value()
vR = csR.value()
    
left_on  = (vL < THRESHOLD + DETECT_MARG)
right_on = (vR < THRESHOLD + DETECT_MARG)

while not (left_on or right_on):
    
    vL = csL.value()
    vR = csR.value()
    
    left_on  = (vL < THRESHOLD + DETECT_MARG)
    right_on = (vR < THRESHOLD + DETECT_MARG)
    
    
    mL.run_to_rel_pos(position_sp=60, speed_sp=BASE_SPEED)
    mR.run_to_rel_pos(position_sp=-60, speed_sp=BASE_SPEED)
    mL.wait_while('running')
    mR.wait_while('running')
    time.sleep(1.5)
    
    vL = csL.value()
    vR = csR.value()
    
    left_on  = (vL < THRESHOLD + DETECT_MARG)
    right_on = (vR < THRESHOLD + DETECT_MARG)
        
    if left_on or right_on:
        break

        # Step 2: Turn a little to the right
    mL.run_to_rel_pos(position_sp=-120, speed_sp=BASE_SPEED)
    mR.run_to_rel_pos(position_sp=120, speed_sp=BASE_SPEED)
    mL.wait_while('running')
    mR.wait_while('running')
    time.sleep(1.5)
    
    vL = csL.value()
    vR = csR.value()
    
    left_on  = (vL < THRESHOLD + DETECT_MARG)
    right_on = (vR < THRESHOLD + DETECT_MARG)
        
    if left_on or right_on:
        break

    # Step 3: Drive forward a little further
    mL.run_to_rel_pos(position_sp=60, speed_sp=BASE_SPEED)
    mR.run_to_rel_pos(position_sp=-60, speed_sp=BASE_SPEED)
    mL.wait_while('running')
    mR.wait_while('running')
    time.sleep(1.5)
        
    if left_on or right_on:
        break
        
    mL.run_forever(speed_sp=BASE_SPEED)
    mR.run_forever(speed_sp=BASE_SPEED)
    time.sleep(1.5)
        
    if left_on or right_on:
        break

subprocess.run(['python3', 'final_line_follower.py'])

        

    
    



