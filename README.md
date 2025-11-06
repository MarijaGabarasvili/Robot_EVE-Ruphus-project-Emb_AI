# Robot_EVE-Ruphus-project-Emb_AI
LEGO Mindstorm project for Embodied AI class. The main task of the robot is to climb the ramp following the ramp and retreat the object back from the second level of the ramp.

## PORTS FOR MOTORS AND SENSORS
Sensors:  
in3 - ultrasonic sensor for can detection  
in4 - right color sensor for line detection  
in1 - left color sensor for line detetction  

Motors:  
outA - left motor to drive  
outD - right motor to drive  
outC - gripper motor to get the obj  

## TO DO:
1) Implement code for line following✅
2) Test on the track✅  
3) Submit the 1 check in by Thursday✅
4) Add sensor for object detection and to measure how far the object is✅
5) Search for can and drive towards it✅
6) Grabing can✅
7) Getting back on line✅
8) Ramp climbing

## Problems which we found:
* The final_line_follower.py is not working anymore (check after dealing with ramp)
* The speed on mR is now 6 and mL is now 13

## The process:
1. Line following task is compleatedusing the PID technique. During the developing fase it was deided to switch from 1 color sensor to 2 for stability and more smooth line following. The design of the RobotEVE/Ruphus also was changed, cause the position of sensors was stopping him from climbing up the ramp.
2. The design of RobotEVE/Ruphus was updated once again, so all the sensors could fit. The gripper was also changed according to the task and now resambles the crab hand. According to the testing the sensors and the hand should not stop it from going on the ramp. Main task was to implement the can detetction and grasping. 
