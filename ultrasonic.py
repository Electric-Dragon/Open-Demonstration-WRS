#Libraries
import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)

#set GPIO Pins
# GPIO_TRIGGER = 18
# GPIO_ECHO = 24

#set GPIO direction (IN / OUT)
# GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
# GPIO.setup(GPIO_ECHO, GPIO.IN)

class USSensor:
    def __init__(self, trigger, echo):
        self.trigger = trigger
        self.echo = echo

    def distance(self):
        GPIO.output(self.trigger, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)

        start = time.time()
        stop = time.time()

        while GPIO.input(self.echo) == 0:
            start = time.time()

        # save time of arrival
        while GPIO.input(self.echo) == 1:
            stop = time.time()

        # time difference between start and arrival
        timeElapsed = stop - start
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (timeElapsed * 34300) / 2

        return distance

if __name__ == '__main__':
    # try:
    #     while True:
    #         dist = distance()
    #         print ("Measured Distance = %.1f cm" % dist)
    #         time.sleep(1)

    #     # Reset by pressing CTRL + C
    # except KeyboardInterrupt:
    #     print("Measurement stopped by User")
        GPIO.cleanup()