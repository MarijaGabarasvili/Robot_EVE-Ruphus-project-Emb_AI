#!/usr/bin/python3

import ev3dev.ev3 as ev3
from time import sleep, time

# Motors
left_motor = ev3.LargeMotor('outA')
right_motor = ev3.LargeMotor('outD')
gripper = ev3.MediumMotor('outC')

# Sensor
us = ev3.UltrasonicSensor('in3')
us.mode = 'US-DIST-CM'

# Sound
sound = ev3.Sound()


# Open gripper function

def open_gripper(x):
    gripper.run_to_rel_pos(position_sp=x, speed_sp=200, stop_action='hold')
    gripper.wait_while('running')
    # sound.speak('Gripper opened')
    # sound.speak('Open pos: '+ str(gripper.position))
    # sleep(4)
    return


# Close gripper safely

def close_gripper():
    start_pos = gripper.position
    start_time = time()

    while True:
        gripper.run_forever(speed_sp=-200)
        sleep(2)
        if gripper.is_stalled:
            gripper.stop()
            break
        if gripper.position - start_pos < -1200:  # physical end of travel
            gripper.stop()
            break
        if time() - start_time > 5:  # safety timeout
            gripper.stop()
            break

    gripper.stop()
    # sound.speak("Gripped")
    sleep(1)
    return


# Move forward to relative position

def move_forward(cm, speed=100):
    DEGREES_PER_CM = 20.45  # rotation of the wheel in oder to cover 1cm distance
    rotation = cm * DEGREES_PER_CM
    left_motor.run_to_rel_pos(position_sp=rotation, speed_sp=speed, stop_action="brake")
    right_motor.run_to_rel_pos(position_sp=rotation, speed_sp=speed, stop_action="brake")
    left_motor.wait_while('running')
    right_motor.wait_while('running')


# Rotate robot by a given angle

def rotate(angle_deg, speed):
    """
    Rotates robot in place by the given angle (in degrees).
    Positive = right turn, Negative = left turn.
    """
    # Ratio adjusted for robot’s wheelbase
    DEGREES_PER_ROBOT_DEGREE = 1.98
    rotation_degrees = angle_deg * DEGREES_PER_ROBOT_DEGREE 

    left_motor.run_to_rel_pos(position_sp=rotation_degrees,
                              speed_sp=speed,
                              stop_action="brake")
    right_motor.run_to_rel_pos(position_sp=-rotation_degrees,
                              speed_sp=speed,
                              stop_action="brake")
    left_motor.wait_while('running')
    right_motor.wait_while('running')
    return


# Sweep around to detect objects/obstacles

def sweep(detection_distance, rotation, rotating_speed, sweep_time, found, ref_search):
    last_seen = time()
    current_time = last_seen
    while current_time <= (last_seen + sweep_time) and not found:
        rotate(rotation, rotating_speed)
        distance_cm = us.value() / 10
        if distance_cm <= detection_distance:
            left_motor.stop()
            right_motor.stop()
            # sound.speak('Detection!')
            # sleep(1)
            # check if it found the object or the wall
            if ref_search == False:
                n = check_for_wall(40, 35)
                if n == 1:
                    # sound.speak('wall!')
                    # sleep(1)
                    shift_direction(-90, 40)
                    break
                elif n == 0:
                    found = True
                    break
                else:
                    continue
            else:
                found = True
                break
        current_time = time()

    return found


# Object search function

def search_for_can(detection_distance, rotation, rotating_speed, sweep_time=2, ref_search=False):
    found = False
    while True:
        distance_cm = us.value() / 10

        if distance_cm <= detection_distance:
            left_motor.stop()
            right_motor.stop()
            # sound.speak('Detection!')
            # sleep(1)
            # check if it found the obstacle or the wall
            if ref_search == False:
                n = check_for_wall(40, 35)
                # sleep(1)
                if n == 1:
                    # sound.speak('wall!')
                    # sleep(1)
                    shift_direction(-90, 40)
                elif n == 0:
                    found = True
                    break
                else:
                    shift_direction(15, 40)
                    continue
            else:
                found = True
                break

        # Sweep left
        found = sweep(detection_distance, -rotation, rotating_speed, sweep_time, found, ref_search)

        # Return to center
        found = sweep(detection_distance, rotation, rotating_speed, sweep_time, found, ref_search)


        # Sweep right
        found = sweep(detection_distance, rotation, rotating_speed, sweep_time, found, ref_search)
        
       
        # Return to center again
        found = sweep(detection_distance, -rotation, rotating_speed, sweep_time, found, ref_search)


        if found:
            break

        left_motor.run_forever(speed_sp=90)
        right_motor.run_forever(speed_sp=90)
        sleep(1.2)

        left_motor.stop()
        right_motor.stop()
    
    return found


# Shift direction 

def shift_direction(shift_rotation, rotating_speed):

    rotate(shift_rotation, rotating_speed)
    sleep(2)

    # MORE FUNCTIONALITIES WILL BE IMPLEMENTIED. THIS IS WHY IN A FUNCTION

    return 


# Detect wall function

def check_for_wall(scan_angle, speed):
    wall = -1

    rotating_degrees = scan_angle/6

    # distance from the obstacle 
    distance_center = us.value()/10
    sleep(1)
    # sound.speak('center' + str(distance_center))
    # sleep(4)

    # sweep around the obstacle for a given angle and average the distances of 7 equidistant points
    rotate(-scan_angle/2, speed)
    distance_sum = 0
    distance = [0,0,0,0,0,0,0]
    for i in range(6):
        distance[i] = us.value()/10
        sleep(1)
        distance_sum += distance[i]

        rotate(rotating_degrees, speed)
        sleep(1)
    distance_sum  += us.value()/10
    sleep(1)

    # rotate back to center
    rotate(-scan_angle/2, speed)

    # print(distance)

    av_distance = distance_sum/7
    # sound.speak('average' + str(round(av_distance)))
    # sleep(4)

    if av_distance > (distance_center - 2) and av_distance < (distance_center + 2):
        wall = 1
    elif av_distance > (distance_center + 3):
        wall = 0
    return wall


def main():

    # the gripper needs to be initially CLOSED
 
    # setting closed position as the starting reference position of the gripper
    gripper.reset()

    # open gripper
    open_gripper(950)

    # sound.speak('Searching for can')
    sleep(4)

    # Searching for can (rotating bigger angles)
    if search_for_can(20, 12, 50, 1.5):
        sleep(1) 

        # searching with smaller angles and slower (finer detection)
        # sound.speak('Refine search')
        # sleep(3)
        search_for_can(16, -10, 25, 2, True)

        # Approach forward
        while us.value() / 10 > 5:
            left_motor.run_forever(speed_sp=50)
            right_motor.run_forever(speed_sp=50)
        left_motor.stop()
        right_motor.stop()

        search_for_can(5, -10, 18, 2, True)
        move_forward(2, 40)

        # Close gripper
        close_gripper()


    rotate(180,50)


if __name__ == "__main__":
    main()