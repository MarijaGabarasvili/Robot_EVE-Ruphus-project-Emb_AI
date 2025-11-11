#!/usr/bin/python3
# EV3dev (ev3dev.ev3) dual-sensor PID line follower
import ev3dev.ev3 as ev3
import time

# -------- Hardware --------
btn = ev3.Button()
mL = ev3.LargeMotor('outA')      # bal motor
mR = ev3.LargeMotor('outD')      # jobb motor
csR = ev3.ColorSensor('in2')     # jobb szenzor
csL = ev3.ColorSensor('in1')     # bal szenzor sciwtvh the logic !!!!!!!
csR.mode = 'COL-REFLECT'
csL.mode = 'COL-REFLECT'

mL.reset()
mR.reset()

# -------- Tuning --------
BASE_SPEED = 150                 # előre sebesség
MAX_SPEED  = 150                 # sebesség korlát

# --- FÜGGETLEN KALIBRÁCIÓ ---
WHITE_L, BLACK_L = 30, 5         # Bal szenzor kalibrált értékek
WHITE_R, BLACK_R = 26, 4         # Jobb szenzor kalibrált értékek

THRESHOLD_L = (BLACK_L + WHITE_L) / 2.0  # Bal szenzor célértéke
THRESHOLD_R = (BLACK_R + WHITE_R) / 2.0  # Jobb szenzor célértéke

DETECT_MARG = 5                  # tűrés a vonal észleléséhez
SEARCH_TURN = 120                # keresési forgási sebesség

MOTOR_FLIP = -1.0
# Az új időzítésekhez a SEARCH_TIME_LIMIT már nem kell, 
# a fázisok időtartama a logikában van.

# PID gains
Kp, Ki, Kd = -0.5, -0.0, -1.0
 
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_speeds(l, r):
    mL.run_forever(speed_sp=clamp(int(l*MOTOR_FLIP), -MAX_SPEED, MAX_SPEED))
    mR.run_forever(speed_sp=clamp(int(r*MOTOR_FLIP), -MAX_SPEED, MAX_SPEED))

# --- ÚJ VÁLTOZÓK AZ ELVESZETT VONAL KERESÉSÉHEZ ---
# Ezt kell tárolni a vonal elvesztése előtt!
last_turn_dir = "straight" # Kezdeti érték: "left", "right" vagy "straight"
search_start_time = None   # Keresés kezdő időpontja
search_phase = None        # Aktuális keresési fázis: 1, 2, vagy 3

mode = "right" 
integral = 0.0
prev_err = 0.0
prev_t = time.time()

try:
    while not btn.any():
        t = time.time()
        dt = max(1e-3, t - prev_t)

        vL = csL.value()
        vR = csR.value()

        # Sensor line detection (a fekete a vonal)
        left_on  = (vL < THRESHOLD_L - DETECT_MARG)
        right_on = (vR < THRESHOLD_R - DETECT_MARG)
        on_line = left_on or right_on

        # --- Fő vonalkövető logika (amíg a vonalon van) ---
        if on_line:
            # Visszaállítjuk a keresési állapotot, ha megtaláltuk a vonalat
            search_start_time = None
            search_phase = None
            
            # PID számítás és mód kiválasztás (mint a régi kódban)
            if left_on and not right_on:
                if mode != "left": 
                    integral = 0.0
                    prev_err = 0.0
                mode = "left"
                last_turn_dir = "left" # FONTOS: rögzítjük az utolsó irányt
            elif right_on and not left_on:
                if mode != "right": 
                    integral = 0.0
                    prev_err = 0.0
                mode = "right"
                last_turn_dir = "right" # FONTOS: rögzítjük az utolsó irányt
            # ha mindkettő látja vagy egyik sem: nem módosítjuk a módot/irányt

            # Kiválasztjuk a mérést és a célértéket
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
            set_speeds(left_cmd, right_cmd)

        # --- ELVESZETT VONAL KERESÉSE ---
        else: # neither_on (sem a bal, sem a jobb szenzor nem látja a vonalat)
            
            if search_start_time is None:
                # Keresés indítása: első fázis az utolsó irányba
                search_start_time = time.time()
                
                # Meghatározzuk, hogy melyik fázissal kezdünk
                if last_turn_dir == "right":
                    # Utolsó irány: Jobb -> Keresés: Jobbra 3mp (FÁZIS 1)
                    search_phase = 1 # Jobbra
                else: # last_turn_dir == "left" vagy "straight"
                    # Utolsó irány: Bal -> Keresés: Balra 3mp (FÁZIS 1)
                    search_phase = 1 # Balra
                
            elapsed_search = time.time() - search_start_time

            # Keresési FÁZIS 1: Előző irányba (3 másodperc)
            if search_phase == 1:
                # Keresési irány az utolsó irányba
                if last_turn_dir == "right":
                    set_speeds(SEARCH_TURN, -SEARCH_TURN) # Jobbra forgás
                else:
                    set_speeds(-SEARCH_TURN, SEARCH_TURN) # Balra forgás
                
                if elapsed_search >= 3.0:
                    # Idő letelt, átlépés a FÁZIS 2-be
                    search_phase = 2
                    search_start_time = time.time() # Időmérő újraindítása

            # Keresési FÁZIS 2: Ellenkező irányba (6 másodperc)
            elif search_phase == 2:
                # Keresési irány az utolsó iránnyal ellentétes
                if last_turn_dir == "right":
                    set_speeds(-SEARCH_TURN, SEARCH_TURN) # Balra forgás
                else:
                    set_speeds(SEARCH_TURN, -SEARCH_TURN) # Jobbra forgás
                
                if elapsed_search >= 6.0:
                    # Idő letelt, átlépés a FÁZIS 3-ba
                    search_phase = 3
                    search_start_time = time.time() # Időmérő újraindítása

            # Keresési FÁZIS 3: Vissza a kiinduló irányba (3 másodperc) és leállás
            elif search_phase == 3:
                # Keresési irány a legelső irányba (FÁZIS 1)
                if last_turn_dir == "right":
                    set_speeds(SEARCH_TURN, -SEARCH_TURN) # Jobbra forgás
                else:
                    set_speeds(-SEARCH_TURN, SEARCH_TURN) # Balra forgás
                
                if elapsed_search >= 3.0:
                    # Idő letelt, megállás
                    ev3.Sound.speak("Search failed. Stopping.").wait()
                    set_speeds(0, 0)
                    break # Kilépés a ciklusból

        prev_err = err
        prev_t = t
        # A vonalkövetéshez a time.sleep(0.5) túl lassú, 
        # PID ciklusban 0.001-0.01s a reális, de maradhat, ha ez volt a cél.
        # time.sleep(0.5) 

finally:
    mL.stop(stop_action="brake")
    mR.stop(stop_action="brake")
    ev3.Sound.speak("Done").wait()