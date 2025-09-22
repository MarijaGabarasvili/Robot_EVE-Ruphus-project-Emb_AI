import ev3dev.ev3 as ev3
import time 
import sleep

btn = ev3.Button()

#sensors and motors used
mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outB')
colorSensor = ev3.ColorSensor('in2')

#define speed and turn angles
BASE_SPEED = 30
TURN_SPEED = 80
PROPORTIONAL_GAIN = 1.2

#getting values of black and white from the color sensor
print("Place sensor over BLACK and wait...")
sleep(3)  # give you time to position it
black = colorSensor.reflected_light_intensity
print("Black value:", black)

# Put sensor over white surface, then read
print("Place sensor over WHITE and wait...")
sleep(3)
white = colorSensor.reflected_light_intensity
print("White value:", white)

# Calculate threshold
threshold = (black + white) / 2
print("Threshold:", threshold)

#Threshhold helps to find the middle point between black and white and will help in turning during line following
# Should run until detects the object
#For now until the button is pressed
while not btn.any():
    # Read the reflected light intensity
    light_intensity = colorSensor.reflected_light_intensity
    # Calculate the deviation from the threshold
    deviation = light_intensity - threshold
    # Calculate turn rate using proportional control
    turn_rate = PROPORTIONAL_GAIN * deviation
    
    # Calculate motor speeds
    left_speed = BASE_SPEED + turn_rate
    right_speed = BASE_SPEED - turn_rate
    
    # Set motor speeds
    mA.run_forever(speed_sp=left_speed)
    mB.run_forever(speed_sp=right_speed)
    
    # Small delay to prevent excessive CPU usage
    time.sleep(0.01)


