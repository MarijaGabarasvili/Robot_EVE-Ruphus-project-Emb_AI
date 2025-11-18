#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower — adaptive sampling

import ev3dev.ev3 as ev3
import time
from collections import deque  # Import deque for efficient history management

# -------- Hardware setup --------
btn = ev3.Button()
mL = ev3.LargeMotor('outA')      # left motor
mR = ev3.LargeMotor('outD')      # right motor
csR = ev3.ColorSensor('in2')     # right color sensor
csL = ev3.ColorSensor('in1')     # left color sensor
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.reset()
mR.reset()

# -------- Base tuning --------
BASE_SPEED = 250                  # base forward speed
MAX_SPEED = 150                   # maximum motor speed

# --- Independent calibration values ---
WHITE_L, BLACK_L = 20, 10          # left sensor calibration
WHITE_R, BLACK_R = 20, 10          # right sensor calibration
THRESHOLD_L = (BLACK_L + WHITE_L) / 2.0
THRESHOLD_R = (BLACK_R + WHITE_R) / 2.0

DETECT_MARG = 2.5                 # margin for line detection
SEARCH_TURN = 120                 # turning speed when searching for the line

# PID control gains
Kp, Ki, Kd = -0.5, 0.0, -1.0

# -------- Adaptive sampling settings --------
TRACK_SLEEP = 0.02           # sampling interval when on the line
LOST_SLEEP_START = 0.3
LOST_SLEEP_MAX = 0.75
LOST_BACKOFF_RATE = 0.5

STRAIGHT_TIME = 0.3
GAP_FORWARD_TIME = 0.08



def clamp(v, lo, hi):
    """Limit the value v to the range [lo, hi]."""
    return max(lo, min(hi, v))


def set_speeds(l, r):
    """Send speed commands to both motors (with clamping)."""
    mL.run_forever(speed_sp=clamp(int(l * -1.0), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r * -1.0), -MAX_SPEED, MAX_SPEED))


# -------- Initial state --------
mode = "right"      # which edge we follow ("right" or "left")
integral = 0.0
prev_err = 0.0
prev_t = time.time()

state = "FOLLOW"            # "FOLLOW", "GAP_COAST", "FORWARD_CHECK", "TURN_SEARCH"
forward_until = None        # end time for the straight phases
lost_start = None           # time when we started being lost (for adaptive sleep)
sleep_time = TRACK_SLEEP


# Last side where we saw the line: -1 = left, +1 = right, 0 = unknown
last_side = 0

# --- HISTORY STORAGE (New) ---
HISTORY_SIZE = 3  # or even 2
history_err = deque([0.0] * HISTORY_SIZE, maxlen=HISTORY_SIZE)
recent_n = 3
n = min(recent_n, len(history_err))
avg_err = sum(list(history_err)[-n:]) / n


# -------- Main loop --------
try:
    while not btn.any():
        t = time.time()
        dt = max(1e-3, t - prev_t)

        # Read both color sensors
        vL = csL.value()
        vR = csR.value()

        print("L: {:5.1f}  R: {:5.1f}".format(vL, vR))  # Removed to de-clutter

        # Determine if sensors see the dark line
        left_on = (vL < THRESHOLD_L - DETECT_MARG)
        right_on = (vR < THRESHOLD_R - DETECT_MARG)
        neither_on = (not left_on and not right_on)
        both_on = (left_on and right_on)

        # Switch which side we follow if necessary
        # This section remains important for defining 'meas' and 'target_threshold'
        if left_on and not right_on:
            if mode != "left":
                integral = 0.0
                prev_err = 0.0
            mode = "left"  # Follow right edge when left sensor is on line
            last_side = -1   # line seen on left
        elif right_on and not left_on:
            if mode != "right":
                integral = 0.0
                prev_err = 0.0
            mode = "right"   # Follow left edge when right sensor is on line
            last_side = +1   # line seen on right
  
        # Choose which sensor to use for control
        if mode == "right":
            meas = vR
            target_threshold = THRESHOLD_R
        else:
            meas = vL
            target_threshold = THRESHOLD_L

        # -------------------- STATE MACHINE --------------------

        if state == "FOLLOW":
            if not neither_on:
                # --- Normal PID line following ---
                err = target_threshold - meas
                integral += err * dt
                deriv = (err - prev_err) / dt
                u = Kp * err + Ki * integral + Kd * deriv

                left_cmd = BASE_SPEED + u
                right_cmd = BASE_SPEED - u
                set_speeds(left_cmd, right_cmd)

                # --- HISTORY UPDATE (New) ---
                # Only store the error if we are successfully following the line
                history_err.append(err)
                # Note: deque automatically handles maxlen, so we just append.

                lost_start = None
                sleep_time = TRACK_SLEEP

            else:
                # Just lost the line: Immediately enter short forward coast
                forward_until = t + GAP_FORWARD_TIME   # Coast for a small gap
                lost_start = t                         # Remember when we got lost
                state = "GAP_COAST"
                set_speeds(BASE_SPEED, BASE_SPEED)     # Keep motors running straight
                sleep_time = TRACK_SLEEP

        elif state == "GAP_COAST":
            if not neither_on:
                # Found the line again while going straight → go back to FOLLOW
                state = "FOLLOW"
                lost_start = None
                sleep_time = TRACK_SLEEP

            else:
                if t < forward_until:
                    # Still within the gap time → go straight
                    set_speeds(BASE_SPEED, BASE_SPEED)
                    sleep_time = TRACK_SLEEP
                else:
                    # Stop and prepare for the longer check
                    mL.stop(stop_action="brake")
                    mR.stop(stop_action="brake")
                    forward_until = t + STRAIGHT_TIME
                    state = "FORWARD_CHECK"
                    sleep_time = TRACK_SLEEP

        elif state == "FORWARD_CHECK":
            if not neither_on:
                # Found the line again while going straight → go back to FOLLOW
                state = "FOLLOW"
                lost_start = None
                sleep_time = TRACK_SLEEP

            else:
                if t < forward_until:
                    # Still within the STRAIGHT_TIME window → go straight
                    set_speeds(BASE_SPEED, BASE_SPEED)
                    sleep_time = TRACK_SLEEP
                else:
                    # Longer forward check failed and still no line → start turning search
                    state = "TURN_SEARCH"
                    sleep_time = LOST_SLEEP_START

        elif state == "TURN_SEARCH":
            if not neither_on:
                # While turning, we found the line again → back to FOLLOW
                state = "FOLLOW"
                lost_start = None
                sleep_time = TRACK_SLEEP

            else:
                # --- Compute average of last few errors ---
                if len(history_err) > 0:
                    n = min(3, len(history_err))               # use last 3 samples
                    avg_err = sum(list(history_err)[-n:]) / n
                else:
                    avg_err = 0.0

                deadband = 0.5

                # --- Use recent error trend to decide direction ---
                if avg_err > deadband:
                    # recent errors say: turn right
                    set_speeds(+SEARCH_TURN, -SEARCH_TURN)

                elif avg_err < -deadband:
                    # recent errors say: turn left
                    set_speeds(-SEARCH_TURN, +SEARCH_TURN)

                else:
                    # --- Use fallback if not enough info ---
                    if last_side == +1:
                        set_speeds(+SEARCH_TURN, -SEARCH_TURN)
                    elif last_side == -1:
                        set_speeds(-SEARCH_TURN, +SEARCH_TURN)
                    else:
                        # fallback to mode
                        if mode == "right":
                            set_speeds(+SEARCH_TURN, -SEARCH_TURN)
                        else:
                            set_speeds(-SEARCH_TURN, +SEARCH_TURN)

                # adaptive sleep
                if lost_start is None:
                    lost_start = t
                lost_secs = t - lost_start
                sleep_time = min(
                    LOST_SLEEP_START + LOST_BACKOFF_RATE * lost_secs,
                    LOST_SLEEP_MAX
                )
        # -------------------------------------------------------

        # Note: prev_err is still calculated using the current `meas` and `target_threshold`
        prev_err = target_threshold - meas if not neither_on else 0.0
        prev_t = t

        time.sleep(sleep_time)

# -------- End of program --------
finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
