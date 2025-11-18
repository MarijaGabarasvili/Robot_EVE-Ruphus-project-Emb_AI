#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower — with zigzag + extra spin search
import ev3dev.ev3 as ev3
import time

# THIS HANDLES THE RAMP WELL

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
BASE_SPEED = 250                  # base forward speed (used on straight parts)
MAX_SPEED  = 150                  # maximum motor speed (absolute)

# --- Independent calibration values ---
WHITE_L, BLACK_L = 22, 10         # left sensor calibration
WHITE_R, BLACK_R = 22, 10         # right sensor calibration
THRESHOLD_L = (BLACK_L + WHITE_L) / 2.0
THRESHOLD_R = (BLACK_R + WHITE_R) / 2.0

DETECT_MARG = 2.5                 # margin for line detection
SEARCH_TURN = 120                 # turning speed when searching for the line

# PID control gains
Kp, Ki, Kd = -0.5, 0.0, -1.0

# Sampling time when following the line
TRACK_SLEEP = 0.02

def clamp(v, lo, hi):
    """Limit the value v to the range [lo, hi]."""
    return max(lo, min(hi, v))

def set_speeds(l, r):
    """Send speed commands to both motors (with clamping)."""
    mL.run_forever(speed_sp=clamp(int(l * -1.0), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r * -1.0), -MAX_SPEED, MAX_SPEED))

def line_status():
    """
    Read sensors and return:
    (left_on, right_on, neither_on, both_on, vL, vR)
    """
    vL = csL.value()
    vR = csR.value()
    left_on  = (vL < THRESHOLD_L - DETECT_MARG)
    right_on = (vR < THRESHOLD_R - DETECT_MARG)
    neither_on = (not left_on and not right_on)
    both_on = (left_on and right_on)
    return left_on, right_on, neither_on, both_on, vL, vR

def search_zigzag(max_cycles=3):
    """
    When the line is lost:
    - Do a zigzag search up to max_cycles times:
        left turn + forward, right turn + forward, ...
    - If the line is found, return True.
    - If not found after all cycles, return False.
    """
    global mode, integral, prev_err

    ZIG_TURN_TIME = 0.4   # how long to turn each time
    ZIG_FWD_TIME  = 0.20   # how long to move forward each time

    for cycle in range(max_cycles):
        # --- 1) Turn left in place, looking for the line ---
        end = time.time() + ZIG_TURN_TIME
        while time.time() < end and not btn.any():
            left_on, right_on, neither_on, both_on, vL, vR = line_status()
            if left_on or right_on:
                if left_on and not right_on:
                    mode = "left"
                elif right_on and not left_on:
                    mode = "right"
                integral = 0.0
                prev_err = 0.0
                return True

            set_speeds(-SEARCH_TURN, +SEARCH_TURN)   # spin left
            time.sleep(TRACK_SLEEP)

        # --- 2) Go slightly forward, looking for the line ---
        end = time.time() + ZIG_FWD_TIME
        while time.time() < end and not btn.any():
            left_on, right_on, neither_on, both_on, vL, vR = line_status()
            if left_on or right_on:
                if left_on and not right_on:
                    mode = "left"
                elif right_on and not left_on:
                    mode = "right"
                integral = 0.0
                prev_err = 0.0
                return True

            set_speeds(BASE_SPEED, BASE_SPEED)
            time.sleep(TRACK_SLEEP)

        # --- 3) Turn right in place, longer, looking for the line ---
        end = time.time() + ZIG_TURN_TIME * 2.0
        while time.time() < end and not btn.any():
            left_on, right_on, neither_on, both_on, vL, vR = line_status()
            if left_on or right_on:
                if left_on and not right_on:
                    mode = "left"
                elif right_on and not left_on:
                    mode = "right"
                integral = 0.0
                prev_err = 0.0
                return True

            set_speeds(+SEARCH_TURN, -SEARCH_TURN)   # spin right
            time.sleep(TRACK_SLEEP)

        # --- 4) Go slightly forward again ---
        end = time.time() + ZIG_FWD_TIME
        while time.time() < end and not btn.any():
            left_on, right_on, neither_on, both_on, vL, vR = line_status()
            if left_on or right_on:
                if left_on and not right_on:
                    mode = "left"
                elif right_on and not left_on:
                    mode = "right"
                integral = 0.0
                prev_err = 0.0
                return True

            set_speeds(BASE_SPEED, BASE_SPEED)
            time.sleep(TRACK_SLEEP)

    return False   # never found the line

def search_spin_last_side(duration=3.0):
    """
    After zigzag fails:
    - Spin in place toward the last line side (stored in `mode`) for `duration` seconds.
    - If the line is found, return True.
    - If not found after duration, return False.
    """
    global mode, integral, prev_err

    end = time.time() + duration
    while time.time() < end and not btn.any():
        left_on, right_on, neither_on, both_on, vL, vR = line_status()
        if left_on or right_on:
            # Found the line while spinning
            if left_on and not right_on:
                mode = "left"
            elif right_on and not left_on:
                mode = "right"
            integral = 0.0
            prev_err = 0.0
            return True

        # Spin toward the last known side (mode)
        if mode == "right":
            set_speeds(+SEARCH_TURN, -SEARCH_TURN)
        else:
            set_speeds(-SEARCH_TURN, +SEARCH_TURN)

        time.sleep(TRACK_SLEEP)

    return False


# -------- Initial state --------
mode = "right"     # which edge we follow ("right" or "left")
integral = 0.0
prev_err = 0.0
prev_t = time.time()

running = True

# -------- Main loop --------
try:
    while running and not btn.any():
        t = time.time()
        dt = max(1e-3, t - prev_t)

        left_on, right_on, neither_on, both_on, vL, vR = line_status()
        print("L: {:5.1f}  R: {:5.1f}".format(vL, vR))

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

        if not neither_on:
            # --- PID line following ---
            err = target_threshold - meas
            integral += err * dt
            deriv = (err - prev_err) / dt
            u = Kp * err + Ki * integral + Kd * deriv

            # Dynamic speed for sharper turns
            abs_err = abs(err)
            base_speed_now = BASE_SPEED

            if both_on:
                base_speed_now = 160
            elif abs_err > 8:
                base_speed_now = 190
            elif abs_err > 4:
                base_speed_now = 220

            left_cmd  = base_speed_now + u
            right_cmd = base_speed_now - u
            set_speeds(left_cmd, right_cmd)

            sleep_time = TRACK_SLEEP

        else:
            # --- Line lost: first try zigzag search ---
            found = search_zigzag(max_cycles=1)
            if not found:
                print("Zigzag failed, spinning toward last side...")
                found_spin = search_spin_last_side(duration=3.5)
                if not found_spin:
                    print("Line not found after zigzag + spin. Stopping.")
                    running = False
                    break
                else:
                    sleep_time = TRACK_SLEEP
            else:
                sleep_time = TRACK_SLEEP

        prev_err = target_threshold - meas if not neither_on else 0.0
        prev_t = t

        time.sleep(sleep_time)

# -------- End of program --------
finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
