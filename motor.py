import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

class Motor:
    def __init__(self, in1, in2, en):
        self.in1 = in1
        self.in2 = in2
        self.en = en
        GPIO.setup(self.in1,GPIO.OUT)
        GPIO.setup(self.in2,GPIO.OUT)
        GPIO.setup(self.en,GPIO.OUT)
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)
        p=GPIO.PWM(self.en,1000)
        p.start(75)

    def antiClockwise(self):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.HIGH)

    def clockwise(self):
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.LOW)

    def stop(self):
        GPIO.output(self.in1,GPIO.LOW)
        GPIO.output(self.in2,GPIO.LOW)

    def changePWM(self, num):
        p = GPIO.PWM(self.en, 1000)
        p.ChangeDutyCycle(num)

if __name__ == '__main__':
    motor1 = Motor(16,18,22)
    motor2 = Motor(11,13,15)

    motor1.antiClockwise()
    motor2.clockwise()

    sleep(2)
    motor1.stop()
    motor2.stop()
    GPIO.cleanup()