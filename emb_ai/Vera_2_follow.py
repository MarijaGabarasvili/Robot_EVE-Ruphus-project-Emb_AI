#!/usr/bin/python3
# EV3dev dual-sensor PID line follower (works on dashed/broken lines)
import ev3dev.ev3 as ev3
import time

btn = ev3.Button()
mL  = ev3.LargeMotor('outA')
mR  = ev3.LargeMotor('outD')
csR = ev3.ColorSensor('in4')
csL = ev3.ColorSensor('in1')
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.reset(); mR.reset()

# ---- Tuning ----
BASE_SPEED   = 180
MAX_SPEED    = 450
BLACK, WHITE = 5, 50
THRESHOLD    = (BLACK + WHITE) / 2.0
DETECT_MARG  = 8
SEARCH_TURN  = 120
GAP_TIMEOUT  = 0.25     # <== NEW: how long to keep going straight across a gap

# PID gains
Kp, Ki, Kd = 4.0, 0.0, 1.2

def clamp(v, lo, hi): return max(lo, min(hi, v))
def set_speeds(l, r):
    mL.run_forever(speed_sp=clamp(int(l), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r), -MAX_SPEED, MAX_SPEED))

mode       = "right"
integral   = 0.0
prev_err   = 0.0
prev_t     = time.time()
gap_start  = None      # <== NEW: remembers when both sensors lost the line

print("Threshold:", THRESHOLD)

try:
    while not btn.any():
        t  = time.time()
        dt = max(1e-3, t - prev_t)

        vL = csL.value()
        vR = csR.value()

        left_on  = (vL < THRESHOLD - DETECT_MARG)
        right_on = (vR < THRESHOLD - DETECT_MARG)
        neither  = (not left_on and not right_on)  # <== NEW: detect gap

        # --- side-switch logic (unchanged) ---
        if left_on and not right_on:
            if mode != "left":
                integral = 0.0; prev_err = 0.0
            mode = "left"
        elif right_on and not left_on:
            if mode != "right":
                integral = 0.0; prev_err = 0.0
            mode = "right"

        if neither:                             # <== NEW: GAP HANDLING
            # Start or continue a gap
            if gap_start is None:
                gap_start = t
            gap_time = t - gap_start

            if gap_time < GAP_TIMEOUT:
                # short gap -> keep driving straight
                set_speeds(BASE_SPEED, BASE_SPEED)
            else:
                # long gap -> begin slow search toward last side
                if mode == "right":
                    set_speeds(+SEARCH_TURN, -SEARCH_TURN)
                else:
                    set_speeds(-SEARCH_TURN, +SEARCH_TURN)

            integral = 0.0; prev_err = 0.0   # freeze PID during gap

        else:                                 # <== back to normal PID if line is found
            gap_start = None
            meas = vR if mode == "right" else vL
            err  = THRESHOLD - meas
            integral += err * dt
            deriv = (err - prev_err) / dt
            u = Kp*err + Ki*integral + Kd*deriv

            left_cmd  = BASE_SPEED + u
            right_cmd = BASE_SPEED - u
            set_speeds(left_cmd, right_cmd)

            prev_err = err

        prev_t = t
        time.sleep(0.01)

finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    ev3.Sound.speak("Done").wait()
