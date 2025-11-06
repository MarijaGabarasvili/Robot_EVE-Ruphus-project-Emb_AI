#!/usr/bin/python3
# Behavior-based (competitive/subsumption) structure to EV3 (ev3dev.ev3)

import ev3dev.ev3 as ev3
import time
from math import copysign

# ---------- Hardver ----------
mL  = ev3.LargeMotor('outA')
mR  = ev3.LargeMotor('outD')
gr  = ev3.MediumMotor('outC')

csL = ev3.ColorSensor('in1'); csL.mode = 'COL-REFLECT'
csR = ev3.ColorSensor('in4'); csR.mode = 'COL-REFLECT'
us  = ev3.UltrasonicSensor('in3'); us.mode = 'US-DIST-CM'
btn = ev3.Button()

# ---------- Base parameters  ----------
BASE_SPEED   = 70
MAX_SPEED    = 200
BLACK, WHITE = 10, 50
THRESHOLD    = (BLACK + WHITE) / 4.0
DETECT_MARG  = 15
KP, KI, KD   = 2.5, 0.0, 1.0

def clamp(v, lo, hi): return max(lo, min(hi, v))

# ---------- Behavior interface  ----------
class Behavior:
    def __init__(self, name, priority):
        self.name = name
        self.priority = priority     #bigger value = higher priority
        self.active = False
        self.last_proposal = (0, 0)  # (left_speed, right_speed)

    def sense(self, S): ...
    def think(self, S): ...
    def propose(self, S):
        # Returns: (active:bool, priority:int, (l,r))
        return self.active, self.priority, self.last_proposal

# ---------- Sensor pack  ----------
class SensePack:
    def __init__(self):
        self.t = time.time()
        self.dt = 0.02
        self.vL = 0
        self.vR = 0
        self.left_on  = False
        self.right_on = False
        self.us_cm = 999
        self.stop_pressed = False

    def read(self, prev_t):
        self.t  = time.time()
        self.dt = max(1e-3, self.t - prev_t)
        self.vL = csL.value()
        self.vR = csR.value()
        self.left_on  = (self.vL < THRESHOLD + DETECT_MARG)
        self.right_on = (self.vR < THRESHOLD + DETECT_MARG)
        self.us_cm    = us.value() / 10.0
        self.stop_pressed = btn.backspace or btn.down or btn.enter

# ---------- Behavior ----------

class EmergencyStop(Behavior):
    def __init__(self): super().__init__("EmergencyStop", 100)
    def sense(self, S): pass
    def think(self, S):
        too_close = (S.us_cm < 3.0)  # 3 cm alatt vész
        self.active = S.stop_pressed or too_close
        self.last_proposal = (0, 0)

class GripBehavior(Behavior):
    # Simple state machine: OPEN -> (if can_close_flag) -> CLOSING -> CLOSED 
    def __init__(self):
        super().__init__("Grip", 90)
        self.state = "OPEN"
        gr.reset()
        # Open at the start of the program (non-blocking, short pulses)
        gr.run_to_rel_pos(position_sp=900, speed_sp=200, stop_action='hold')

    def sense(self, S): pass
    def think(self, S):
        # Ezt tedd feltételhez: ha közel van a doboz és „center”
        # If it's close enough to an object, we can try to close the gripper
        # Here is just an example: if closer than 5 cm, close.
        if self.state == "OPEN" and S.us_cm < 5.0:
            self.state = "CLOSING"
            gr.run_forever(speed_sp=-200)

        if self.state == "CLOSING":
            # Simple stall detection: if the gripper is stalled
            if gr.is_stalled:
                gr.stop()
                self.state = "CLOSED"

        self.active = (self.state in ("CLOSING",))
        self.last_proposal = (0, 0)  # while closing, don't move


class ApproachCan(Behavior):
    def __init__(self):
        super().__init__("ApproachCan", 80)
    def sense(self, S): pass
    def think(self, S):
        # If an object is detected within 30 cm, approach it slowly
        # but stop if it's very close (7 cm)
        near = (S.us_cm < 30.0)
        very_close = (S.us_cm < 7.0)
        self.active = near and not very_close
        v = clamp(BASE_SPEED//2, -MAX_SPEED, MAX_SPEED)
        self.last_proposal = (v, v)

class DetectCan(Behavior):
    def __init__(self):
        super().__init__("DetectCan", 70)
        self.dir = 1  # sweep direction
        self.timer = 0.0
    def sense(self, S): pass
    def think(self, S):
        # Active if no line is detected (so it doesn't interfere with LineFollower) and object is not too close
        no_line = (not S.left_on and not S.right_on)
        far = (S.us_cm > 7.0)
        self.active = no_line and far
        if not self.active:
            self.last_proposal = (0,0)
            return
        # sweep : change direction every 0.5 seconds
        if self.timer <= S.t:
            self.dir *= -1
            self.timer = S.t + 0.5
        turn = 80 * self.dir
        self.last_proposal = ( turn, -turn )

class LineFollower(Behavior):
    def __init__(self):
        super().__init__("LineFollower", 60)
        self.integral = 0.0
        self.prev_err = 0.0
    def sense(self, S): pass
    def think(self, S):
        left_on, right_on = S.left_on, S.right_on
        # Active if at least one sensor detects the line
        self.active = left_on or right_on
        if not self.active:
            self.last_proposal = (0,0); return

        # Pick a side to follow based on which sensor sees the line
        if right_on and not left_on:
            err = THRESHOLD - S.vR
            sign = +1   # pozitive correction to turn right
        else:
            err = S.vL - THRESHOLD
            sign = -1

        self.integral += err * S.dt
        deriv = (err - self.prev_err) / S.dt
        u = -(KP*err + KI*self.integral + KD*deriv) * sign

        left_cmd  = BASE_SPEED + u
        right_cmd = BASE_SPEED - u
        self.last_proposal = (left_cmd, right_cmd)
        self.prev_err = err

class FindLine(Behavior):
    def __init__(self): super().__init__("FindLine", 40)
    def sense(self, S): pass
    def think(self, S):
        # Active if neither sensor sees the line
        self.active = (not S.left_on and not S.right_on)
        if self.active:
            self.last_proposal = ( 120, -120 )  # kereső forgás
        else:
            self.last_proposal = (0,0)

class Cruise(Behavior):
    def __init__(self): super().__init__("Cruise", 10)
    def sense(self, S): pass
    def think(self, S):
        # Base "do nothing" forward motion
        self.active = True
        self.last_proposal = ( 20, 20 )

# ---------- Arbitrator ---------- Decides what "to do" based on behaviors
class Arbitrator:
    def __init__(self, behaviors):
        # Higher priority behaviors come first
        self.behaviors = sorted(behaviors, key=lambda b: -b.priority)

    def choose(self, S):
        winners = []
        for b in self.behaviors:
            b.sense(S)
            b.think(S)
            active, prio, (l,r) = b.propose(S)
            if active:
                winners.append((prio, l, r, b.name))
                # subsumption: the first active highest priority wins
                return (l, r, b.name)
        # if no behavior is active, stop
        return (0, 0, "None")

# ---------- Main loop ----------
def set_speeds(l, r):
    mL.run_forever(speed_sp=int(clamp(l, -MAX_SPEED, MAX_SPEED)))
    mR.run_forever(speed_sp=int(clamp(r, -MAX_SPEED, MAX_SPEED)))

def main():
    bhs = [
        EmergencyStop(),
        GripBehavior(),
        ApproachCan(),
        DetectCan(),
        LineFollower(),
        FindLine(),
        Cruise()
    ]
    arb = Arbitrator(bhs)
    prev_t = time.time()

    ev3.Sound.speak("Behavior control start").wait()
    try:
        while True:
            S = SensePack()
            S.read(prev_t)
            l, r, who = arb.choose(S)
            set_speeds(l, r)
            # optional: status print
            # print(f"{who:>12} | L={int(l):4d} R={int(r):4d} | vL={S.vL} vR={S.vR} us={S.us_cm:.1f}")
            prev_t = S.t
            time.sleep(0.02)  # 50 Hz
    finally:
        set_speeds(0,0)
        ev3.Sound.speak("Done").wait()

if __name__ == "__main__":
    main()
