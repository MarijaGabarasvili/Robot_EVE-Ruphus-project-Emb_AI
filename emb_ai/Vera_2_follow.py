#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower with gap-bridge then search
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
BASE_SPEED   = 150           # forward speed
MAX_SPEED    = 450           # clamp
BLACK, WHITE = 5, 50         # quick manual calibration
THRESHOLD    = (BLACK + WHITE) / 2.0
DETECT_MARG  = 8             # how close to threshold counts as "on edge"
SEARCH_TURN  = 120           # search spin speed

# PID gains (start here)
Kp, Ki, Kd   = 2.0, 0.0, 0.6

# --- Dotted-line helpers ---
BRIDGE_TIME    = 0.25        # seconds to go forward after losing line
BRIDGE_SPEED_F = 0.85        # fraction of BASE_SPEED during bridge (0..1)

def clamp(v, lo, hi): return max(lo, min(hi, v))
def set_speeds(l, r):
    mL.run_forever(speed_sp=clamp(int(l), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r), -MAX_SPEED, MAX_SPEED))

mode = "right"      # "right" or "left" (which edge we follow)
integral = 0.0
prev_err = 0.0
prev_t   = time.time()

# Gap-handling/search state
bridging = False
bridge_start_t = 0.0
searching = False
last_u = 0.0

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

        neither_on = (not left_on and not right_on)

        if not neither_on:
            # Normal follow: clear gap/search states and drive PID
            bridging = False
            searching = False
            set_speeds(left_cmd, right_cmd)

        else:
            # Lost the line
            if not bridging and not searching:
                # Start tiny forward bridge using last steering
                bridging = True
                bridge_start_t = t
                # Optional: don't let integral explode during the gap
                integral = 0.0; prev_err = 0.0

            if bridging:
                # Keep moving forward a bit with last steering
                set_speeds(BRIDGE_SPEED_F * (BASE_SPEED + last_u),
                           BRIDGE_SPEED_F * (BASE_SPEED - last_u))

                # Re-check after the grace period; if still nothing, begin search
                if (t - bridge_start_t) >= BRIDGE_TIME:
                    bridging = False
                    searching = True
                    # reset PID memory before search to avoid stale terms
                    integral = 0.0; prev_err = 0.0

            elif searching:
                # Spin toward the last-followed side to reacquire
                if mode == "right":
                    set_speeds(+SEARCH_TURN, -SEARCH_TURN)  # spin right
                else:
                    set_speeds(-SEARCH_TURN, +SEARCH_TURN)  # spin left

        last_u  = u
        prev_err = err
        prev_t   = t
        time.sleep(0.01)

finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    ev3.Sound.speak("Done").wait()
