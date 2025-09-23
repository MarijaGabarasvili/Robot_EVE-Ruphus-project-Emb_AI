#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower
import ev3dev.ev3 as ev3
import time

# -------- Hardware --------
btn = ev3.Button()
mL  = ev3.LargeMotor('outA')      # left motor
mR  = ev3.LargeMotor('outD')      # right motor
csR = ev3.ColorSensor('in4')      # right sensor
csL = ev3.ColorSensor('in1')      # left sensor
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.reset(); mR.reset()

# -------- Tuning --------
BASE_SPEED   = 70           # forward speed
MAX_SPEED    = 150           # clamp
BLACK, WHITE = 5, 50         # quick manual calibration
THRESHOLD    = (BLACK + WHITE) / 2.0
DETECT_MARG  = 8             # how close to threshold counts as "on edge"
SEARCH_TURN  = 120           # search spin speed

# PID gains (start here)
Kp, Ki, Kd   = 2.0, 0.0, 1.2

def clamp(v, lo, hi): return max(lo, min(hi, v))
def set_speeds(l, r):
    mL.run_forever(speed_sp=clamp(int(l), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r), -MAX_SPEED, MAX_SPEED))

mode = "right"      # "right" or "left" (which edge we follow)
integral = 0.0
prev_err = 0.0
prev_t   = time.time()

print("Threshold:", THRESHOLD)

try:
    while not btn.any():
        t  = time.time()
        dt = max(1e-3, t - prev_t)

        vL = csL.value()
        vR = csR.value()

        # Decide which sensor to follow
        left_on  = (vL < THRESHOLD - DETECT_MARG)
        right_on = (vR < THRESHOLD - DETECT_MARG)

        if left_on and not right_on:
            if mode != "left":   # reset PID when switching sides
                integral = 0.0; prev_err = 0.0
            mode = "left"
        elif right_on and not left_on:
            if mode != "right":
                integral = 0.0; prev_err = 0.0
            mode = "right"
        # if both or none -> keep previous mode



        # Compute PID using the active sensor
        meas = vR if mode == "right" else vL
        err  = THRESHOLD - meas
        integral += err * dt
        deriv = (err - prev_err) / dt

        u = Kp*err + Ki*integral + Kd*deriv

        # Same motor mix works for both sides with this error definition:
        # positive u -> turn right (left faster)
        left_cmd  = BASE_SPEED + u
        right_cmd = BASE_SPEED - u

        # If neither sensor sees the line, gently search toward last side
        neither_on = (not left_on and not right_on)
        if neither_on:
            if mode == "right":
                set_speeds(+SEARCH_TURN, -SEARCH_TURN)  # spin right
            else:
                set_speeds(-SEARCH_TURN, +SEARCH_TURN)  # spin left
            integral = 0.0; prev_err = 0.0
        else:
            set_speeds(left_cmd, right_cmd)

        prev_err = err
        prev_t   = t
        time.sleep(0.01)

finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    ev3.Sound.speak("Done").wait()
