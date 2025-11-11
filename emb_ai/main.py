#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time
import subprocess

subprocess.run(['python3', 'find_line.py'])
subprocess.run(['python3', 'detect_can.py'])
subprocess.run(['python3', 'find_line.py'])
