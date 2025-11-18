#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower — adaptive sampling
import ev3dev.ev3 as ev3
import time

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
MAX_SPEED  = 150                  # maximum motor speed

# --- Independent calibration values ---
# You must measure these values for your robot (for white and black)
WHITE_L, BLACK_L = 22, 10          # left sensor calibration
WHITE_R, BLACK_R = 22, 10          # right sensor calibration
THRESHOLD_L = (BLACK_L + WHITE_L) / 2.0
THRESHOLD_R = (BLACK_R + WHITE_R) / 2.0

DETECT_MARG = 2.5                 # margin for line detection
SEARCH_TURN = 120                 # turning speed when searching for the line

# PID control gains
Kp, Ki, Kd = -0.5, 0.0, -1.0

# -------- Adaptive sampling settings --------
# When the line is visible → fast updates
# When the line is lost → slower updates (to save CPU and prevent jitter)
TRACK_SLEEP        = 0.02   # sampling interval when on the line
LOST_SLEEP_START   = 0.3    # 0.12   Delay of the first loop when off the line 
LOST_SLEEP_MAX     = 1.5   # 0.75 # maximum delay when line is lost for a long time
LOST_BACKOFF_RATE  = 1.0   # 0.5 # how fast the delay increases per second
# When off the line, only "act" every Nth loop
OFFLINE_SKIP = 3           # take a "sample" (update logic) every 3rd loop when off line



def clamp(v, lo, hi):
    """Limit the value v to the range [lo, hi]."""
    return max(lo, min(hi, v))

def set_speeds(l, r):
    """Send speed commands to both motors (with clamping)."""
    mL.run_forever(speed_sp=clamp(int(l * -1.0), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r * -1.0), -MAX_SPEED, MAX_SPEED))

# -------- Initial state --------
mode = "right"     # which edge we follow ("right" or "left")
integral = 0.0
prev_err = 0.0
prev_t = time.time()
lost_start = None  # when the robot last lost the line (None = currently on line)
offline_counter = 0   # counts loops while off the line


# -------- Main loop --------
try:
    while not btn.any():
        t = time.time()
        dt = max(1e-3, t - prev_t)

        # Read both color sensors
        vL = csL.value()
        vR = csR.value()
        print("L: {:5.1f}  R: {:5.1f}".format(vL, vR))

        # Determine if sensors see the dark line
        left_on  = (vL < THRESHOLD_L - DETECT_MARG)
        right_on = (vR < THRESHOLD_R - DETECT_MARG)
        neither_on = (not left_on and not right_on)

        # Switch which side we follow if necessary
        if left_on and not right_on:
            if mode != "left":
                integral = 0.0
                prev_err = 0.0
            mode = "left"
        elif right_on and not left_on:
            if mode != "right":
                integral = 0.0
                prev_err = 0.0
            mode = "right"

        # Choose which sensor to use for control
        if mode == "right":
            meas = vR
            target_threshold = THRESHOLD_R
        else:
            meas = vL
            target_threshold = THRESHOLD_L

        # If the line is visible by at least one sensor
        if not neither_on:
            # --- PID control ---
            err = target_threshold - meas
            integral += err * dt
            deriv = (err - prev_err) / dt
            u = Kp * err + Ki * integral + Kd * deriv

            left_cmd  = BASE_SPEED + u
            right_cmd = BASE_SPEED - u
            set_speeds(left_cmd, right_cmd)

            # Reset the "line lost" timer and use fast sampling
            lost_start = None
            sleep_time = TRACK_SLEEP

        else:
            # --- Line is lost ---
            offline_counter += 1

            # Only update behavior every OFFLINE_SKIP-th loop
            if offline_counter % OFFLINE_SKIP == 0:
                # Spin gently toward the last known side
                if mode == "right":
                    set_speeds(+SEARCH_TURN, -SEARCH_TURN)  # spin right
                else:
                    set_speeds(-SEARCH_TURN, +SEARCH_TURN)  # spin left

                # Reset PID state (to prevent large jumps)
                integral = 0.0
                prev_err = 0.0

                # Gradually slow down the sampling rate the longer the line is lost
                if lost_start is None:
                    lost_start = t
                    sleep_time = LOST_SLEEP_START
                else:
                    lost_secs = t - lost_start
                    sleep_time = min(
                        LOST_SLEEP_START + LOST_BACKOFF_RATE * lost_secs,
                        LOST_SLEEP_MAX
                    )

        prev_err = target_threshold - meas if not neither_on else 0.0
        prev_t = t

        # Wait before next loop iteration (adaptive)
        time.sleep(sleep_time)

# -------- End of program --------
finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")