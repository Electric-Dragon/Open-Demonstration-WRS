import classify as cs
from ultrasonic import USSensor as us
from motor import Motor as m
import RPi.GPIO as GPIO
import time
import sys

searching = True
found = False
dropping = False
taskDone = False

motor1 = m(18,16,22)
motor2 = m(11,13,15)
motor3 = m(19,21,23)

cs.main(sys.argv[1:])

while not taskDone:

    predictions = cs.getResults()

    clothes = []

    if 'cloth' in predictions and searching:
        clothes = predictions[predictions['label']=='cloth']
        sortedResults = sorted(clothes, key=lambda x: (clothes[x]['width']))
        print(sortedResults)

GPIO.cleanup()