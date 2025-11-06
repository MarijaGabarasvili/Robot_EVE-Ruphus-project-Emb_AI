#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower
import ev3dev.ev3 as ev3
import time
import subprocess

# -------- Hardware --------
btn = ev3.Button()
mL = ev3.LargeMotor('outA')       # left motor
mR = ev3.LargeMotor('outD')       # right motor
csR = ev3.ColorSensor('in4')      # right sensor
csL = ev3.ColorSensor('in1')      # left sensor
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.stop(stop_action="brake")
mR.stop(stop_action="brake")

print("Left:", csL.value(), "Right:", csR.value())

# -------- Tuning --------
BASE_SPEED = 70           # forward speed
MAX_SPEED = 150           # clamp
BLACK, WHITE = 10, 50     # quick manual calibration
THRESHOLD = (BLACK + WHITE) / 4.0
DETECT_MARG = 15          # how close to threshold counts as "on edge"
SEARCH_TURN = 120         # search spin speed

# PID gains
Kp, Ki, Kd = 2.5, 0.0, 1.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_speeds(l, r):
    mL.run_forever(speed_sp=clamp(int(l), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r), -MAX_SPEED, MAX_SPEED))

mode = "right"      # which edge we follow
integral = 0.0
prev_err = 0.0
prev_t = time.time()
off_l = None

# --- NEW VARIABLES ---
search_start_l = None  # time when search started
search_phase = None    # "left" or "right"
# ---------------------
line = True

print("Threshold:", THRESHOLD)

try:
    while line:
        t = time.time()
        dt = max(1e-3, t - prev_t)

        vL = csL.value()
        vR = csR.value()

        # Detect line presence
        left_on = (vL < THRESHOLD + DETECT_MARG)
        right_on = (vR < THRESHOLD + DETECT_MARG)

        if left_on and not right_on:
            mode = "left"
        elif right_on and not left_on:
            mode = "right"
        # else: keep previous mode

        # PID error
        if mode == "right":
            err = THRESHOLD - vR
        else:
            err = vL - THRESHOLD

        integral += err * dt
        deriv = (err - prev_err) / dt
        u = -(Kp * err + Ki * integral + Kd * deriv)

        left_cmd = BASE_SPEED + u
        right_cmd = BASE_SPEED - u

        # Check if line lost
        neither_on = (not left_on and not right_on)
        if neither_on:
            if off_l is None:
                off_l = time.time()
                set_speeds(0, 0)
                search_start_l = time.time()
                search_phase = "left"

            elapsed_off = time.time() - off_l

            if elapsed_off < 2:
                if search_phase == "left":
                    # Turn left for 1 second
                    set_speeds(-SEARCH_TURN, SEARCH_TURN)
                    if time.time() - search_start_l >= 1:
                        search_phase = "right"
                        search_start_l = time.time()
                elif search_phase == "right":
                    # Turn right for 1 second
                    set_speeds(SEARCH_TURN, -SEARCH_TURN)
            else:
                # After 2s total (1s left + 1s right) stop and break
                set_speeds(0, 0)
                break
        else:
            # Line found again
            off_l = None
            set_speeds(left_cmd, right_cmd)

        prev_err = err
        prev_t = t
        time.sleep(0.01)

finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    ev3.Sound.speak("Done").wait()
