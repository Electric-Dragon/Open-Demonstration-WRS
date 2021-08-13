import classify as cs
from ultrasonic import USSensor as us
from motor import Motor as m
import RPi.GPIO as GPIO
import time
import sys

# sensor1 = us()

motor1 = m(18,16,22)
motor2 = m(11,13,15)

cs.main(sys.argv[1:])

print(cs.getResults())