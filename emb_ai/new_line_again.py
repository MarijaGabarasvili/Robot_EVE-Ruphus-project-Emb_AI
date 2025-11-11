#!/usr/bin/python3
  # EV3dev (ev3dev.ev3) dual-sensor PID line follower
import ev3dev.ev3 as ev3
import time

# -------- Hardware --------
btn = ev3.Button()
mL = ev3.LargeMotor('outA')      # left motor
mR = ev3.LargeMotor('outD')      # right motor
csR = ev3.ColorSensor('in2')     # right sensor
csL = ev3.ColorSensor('in1')     # left sensor
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.reset()
mR.reset()

# -------- Tuning --------
BASE_SPEED = 150
MAX_SPEED  = 150

# --- FÜGGETLEN KALIBRÁCIÓ ---
# These values MUST be measured by you!
WHITE_L, BLACK_L = 30, 5     # Bal szenzor
WHITE_R, BLACK_R = 26, 4     # Jobb szenzor

THRESHOLD_L = (BLACK_L + WHITE_L) / 2.0
THRESHOLD_R = (BLACK_R + WHITE_R) / 2.0

POLARITY = -1 # motors are reversed

# PID gains
Kp, Ki, Kd = 2.5, 0.0, 1.0

# --- Dotted Line Tuning ---
SEARCH_TURN    = 120    # search spin speed
DETECT_MARG    = 5
GAP_COAST_TIME = 0.5    # (SECONDS) How long to "coast" straight over a gap
LINE_LOST_TIME = 3.0    # (SECONDS) How long to search before giving up

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_speeds(l, r):
    mL.run_forever(speed_sp=clamp(int(l*POLARITY), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r*POLARITY), -MAX_SPEED, MAX_SPEED))

mode = "right"
integral = 0.0
prev_err = 0.0
prev_t = time.time()

off_l = None  # Timer for when the line was lost

try:
    while not btn.any():
        t = time.time()
        dt = max(1e-3, t - prev_t)

        vL = csL.value()
        vR = csR.value()

        # Sensor line detection
        left_on  = (vL < THRESHOLD_L - DETECT_MARG)
        right_on = (vR < THRESHOLD_R - DETECT_MARG)
        neither_on = (not left_on and not right_on)

        # Select which sensor to follow
        if left_on and not right_on:
            if mode != "left":  # reset PID
                integral = 0.0
                prev_err = 0.0
            mode = "left"
        elif right_on and not left_on:
            if mode != "right": # reset PID
                integral = 0.0
                prev_err = 0.0
            mode = "right"
        
        # Select measurement and target based on mode
        if mode == "right":
            meas = vR
            target_threshold = THRESHOLD_R
        else:
            meas = vL
            target_threshold = THRESHOLD_L

        # PID calculation
        err = target_threshold - meas
        integral += err * dt
        deriv = (err - prev_err) / dt
        u = Kp * err + Ki * integral + Kd * deriv

        left_cmd  = BASE_SPEED + u
        right_cmd = BASE_SPEED - u

        if neither_on:
            # === IN A GAP or OFF LINE ===
            if off_l is None:
                # We just lost the line, start the timer
                off_l = time.time()
                
            time_in_gap = time.time() - off_l

            if time_in_gap < GAP_COAST_TIME:
                # --- Phase 1: Coasting ---
                # Assume it's a gap, just drive straight.
                set_speeds(BASE_SPEED, BASE_SPEED)
                
                # We're not using PID, so reset it for when we find the line
                integral = 0.0
                prev_err = 0.0
                
            elif time_in_gap < LINE_LOST_TIME:
                # --- Phase 2: Searching ---
                # The "gap" was too long, we're probably lost. Start spinning.
                if mode == "right":
                    set_speeds(+SEARCH_TURN, -SEARCH_TURN)  # spin right
                else:
                    set_speeds(-SEARCH_TURN, +SEARCH_TURN)  # spin left
            else:
                # --- Phase 3: Give Up ---
                print("Line lost, stopping")
                break # Stop the program
                
        else:
            # === ON THE LINE ===
            # We see the line, so apply normal PID steering
            set_speeds(left_cmd, right_cmd)
            
            # Reset the gap timer, since we are on the line
            off_l = None 

        prev_err = err
        prev_t = t
        # NO time.sleep() HERE! Let the loop run as fast as possible.

finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    ev3.Sound.speak("Done").wait()