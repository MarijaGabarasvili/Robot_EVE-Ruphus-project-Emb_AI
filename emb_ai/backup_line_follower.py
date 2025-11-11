#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower
import ev3dev.ev3 as ev3
import time

# -------- Hardware --------
btn = ev3.Button()
mL = ev3.LargeMotor('outA')      # left motor
mR = ev3.LargeMotor('outD')      # right motor
# Fontos: ellenőriztem a szenzor portokat, hogy megegyezzenek a motorokkal
csR = ev3.ColorSensor('in2')     # right sensor (IN2-et használja)
csL = ev3.ColorSensor('in1')     # left sensor (IN1-et használja)
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.reset()
mR.reset()

# -------- Tuning --------
BASE_SPEED = 150                  # forward speed
MAX_SPEED  = 150                  # clamp

# --- FÜGGETLEN KALIBRÁCIÓ ---
# Ezeket az értékeket neked kell megmérned!
WHITE_L, BLACK_L = 30, 5         # Bal szenzor: Fehér (60), Fekete (8)
WHITE_R, BLACK_R = 26, 4        # Jobb szenzor: Fehér (75), Fekete (15)

# Független küszöbértékek kiszámítása
THRESHOLD_L = (BLACK_L + WHITE_L) / 2.0  # Bal szenzor célértéke
THRESHOLD_R = (BLACK_R + WHITE_R) / 2.0  # Jobb szenzor célértéke

DETECT_MARG = 2.5                  # how close to threshold counts as "on edge"
SEARCH_TURN = 120                # search spin speed

# PID gains
Kp, Ki, Kd = -0.5, 0.0, -1.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_speeds(l, r):
    mL.run_forever(speed_sp=clamp(int(l*(-1.0)), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r*(-1.0)), -MAX_SPEED, MAX_SPEED))

mode = "right"    # "right" or "left" (which edge we follow)
integral = 0.0
prev_err = 0.0
prev_t = time.time()

# Added to time the turn 
off_l = None        # time when left sensor went off line
search_start_l = None  # time when search started
search_phase = None    # "left" or "right"
line = True             # main loop control


try:
    while not btn.any():
        t = time.time()
        dt = max(1e-3, t - prev_t)

        vL = csL.value()
        vR = csR.value()

        # Sensor line detection with independent thresholds
        left_on  = (vL < THRESHOLD_L - DETECT_MARG)
        right_on = (vR < THRESHOLD_R - DETECT_MARG)

        # Select which sensor to follow
        if left_on and not right_on:
            if mode != "left":   # reset PID when switching sides
                integral = 0.0
                prev_err = 0.0
            mode = "left"
        elif right_on and not left_on:
            if mode != "right":  # reset PID when switching sides
                integral = 0.0
                prev_err = 0.0
            mode = "right"
        # if both or none -> keep previous mode

        # Select measurement and target threshold based on mode
        if mode == "right":
            meas = vR
            target_threshold = THRESHOLD_R
        else:
            meas = vL
            target_threshold = THRESHOLD_L

        # PID
        err = target_threshold - meas
        integral += err * dt
        deriv = (err - prev_err) / dt
        u = Kp * err + Ki * integral + Kd * deriv

        # Motor control
        left_cmd  = BASE_SPEED + u
        right_cmd = BASE_SPEED - u

        # If neither sensor sees the line, gently search toward last side
        neither_on = (not left_on and not right_on)
        if neither_on:
            if mode == "right":
                set_speeds(+SEARCH_TURN, -SEARCH_TURN)  # spin right
            else:
                set_speeds(-SEARCH_TURN, +SEARCH_TURN)  # spin left
            integral = 0.0
            prev_err = 0.0
        else:
            set_speeds(left_cmd, right_cmd)

        prev_err = err
        prev_t = t
        time.sleep(0.45)

finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    ev3.Sound.speak("Done").wait()
