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
WHITE_L, BLACK_L = 30, 5          # left sensor calibration
WHITE_R, BLACK_R = 26, 4          # right sensor calibration
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
LOST_SLEEP_START   = 0.12   # starting delay when line is first lost
LOST_SLEEP_MAX     = 0.50   # maximum delay when line is lost for a long time
LOST_BACKOFF_RATE  = 0.05   # how fast the delay increases per second

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

# -------- Main loop --------
try:
    while not btn.any():
        t = time.time()
        dt = max(1e-3, t - prev_t)

        # Read both color sensors
        vL = csL.value()
        vR = csR.value()

        # Determine if sensors see the dark line
        left_on  = (vL < THRESHOLD_L - DETECT_MARG)
        right_on = (vR < THRESHOLD_R - DETECT_MARG)
        both_on = (left_on and right_on)       
        neither_on = (not left_on and not right_on)
    
        # --- CORNER EXECUTION ---
        if corner_mode:
            elapsed_corner = t - corner_start_t
            
            if elapsed_corner < CORNER_TURN_DURATION:
                # Execute the sharp, fixed-time turn
                if corner_dir == "right":
                    # Sharp right spin (Right motor reversed)
                    set_speeds(BASE_SPEED, -BASE_SPEED) 
                else: 
                    # Sharp left spin (Left motor reversed)
                    set_speeds(-BASE_SPEED, BASE_SPEED)
                
                # Update time and skip the rest of the loop
                prev_t = t
                time.sleep(TRACK_SLEEP)
                continue
            else:
                # Corner turn is over, exit corner mode
                corner_mode = False
                corner_start_t = None


        # --- MODE SWITCHING & CORNER DETECTION ---
        if not neither_on:
            
            # 1. Corner Detection (Both sensors see the line)
            if both_on:
                if corner_start_t is None:
                    # Start timing the 'both on' condition
                    corner_start_t = t
                
                # Check if corner is confirmed
                if t - corner_start_t >= CORNER_DETECT_TIME:
                    # Corner Confirmed! Set direction and enter corner mode
                    corner_mode = True
                    # The direction to turn is the opposite of the current mode
                    corner_dir = "left" if mode == "right" else "right"
                    
                    # Reset PID state for a fresh start on the new line
                    integral = 0.0
                    prev_err = 0.0
                    
                    # Jump to the 'continue' above to start the sharp turn immediately
                    prev_t = t
                    time.sleep(TRACK_SLEEP)
                    continue 
                # If not confirmed, fall through to PID control on the current mode

            else: # Only one sensor sees the line (Normal line following)
                corner_start_t = None # Reset corner timer if only one sensor is active

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

            # Choose which sensor to use for control (Current mode)
            if mode == "right":
                meas = vR
                target_threshold = THRESHOLD_R
            else:
                meas = vL
                target_threshold = THRESHOLD_L

            # --- PID control (Executed when not in corner_mode and on line) ---
            err = target_threshold - meas
            integral += err * dt
            deriv = (err - prev_err) / dt
            u = Kp * err + Ki * integral + Kd * deriv

            left_cmd = BASE_SPEED + u
            right_cmd = BASE_SPEED - u
            set_speeds(left_cmd, right_cmd)

            # Reset the 'line lost' timer and use fast sampling
            lost_start = None
            sleep_time = TRACK_SLEEP

        else:
            # --- Line is lost ---
            
            # Reset corner timer (if the robot loses the line while approaching a corner)
            corner_start_t = None 
            
            # Spin gently toward the last known side (using the current mode)
            if mode == "right":
                set_speeds(+SEARCH_TURN, -SEARCH_TURN) # spin right
            else:
                set_speeds(-SEARCH_TURN, +SEARCH_TURN) # spin left

            # Reset PID state
            integral = 0.0
            prev_err = 0.0

            # Gradually slow down the sampling rate (Adaptive sleep)
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
    ev3.Sound.speak("Done").wait()
