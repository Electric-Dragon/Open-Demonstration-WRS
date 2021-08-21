from motor import Motor as m
import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

class Grabber:
    def __init__(self, motor1: m, motor2: m):
        self.motor1 = motor1
        self.motor2 = motor2
        self.motor1.changePWM(25)
        self.motor2.changePWM(25)

    def grab(self):
        self.motor2.clockwise()
        sleep(1.5)
        self.motor2.stop()
        self.motor1.clockwise()
        sleep(0.3)
        self.motor1.stop()
        self.motor2.antiClockwise()
        sleep(1)
        self.motor2.stop()

    def drop(self):
        self.motor1.antiClockwise()
        sleep(0.2)
        self.motor1.stop()