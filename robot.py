from ultrasonic import USSensor as us
from motor import Motor as m
from grabber import Grabber as g
import RPi.GPIO as GPIO
import time
import sys
import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner

GPIO.setmode(GPIO.BOARD)

runner = None
show_camera = True
searching = True
found = False
dropping = False
taskDone = False

in1 = 16
in2 = 18
en = 22

in3 = 11
in4 = 13
en2 = 15

in5 = 19
in6 = 21
en3 = 23

in7 = 29
in8 = 31
en4 = 33

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
p=GPIO.PWM(en,1000)
p.start(57)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
p2=GPIO.PWM(en2,1000)
p2.start(75)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(in5,GPIO.OUT)
GPIO.setup(in6,GPIO.OUT)
GPIO.setup(en3,GPIO.OUT)
GPIO.output(in5,GPIO.LOW)
GPIO.output(in6,GPIO.LOW)
p3=GPIO.PWM(en3,1000)
p3.start(25)

GPIO.setup(in7,GPIO.OUT)
GPIO.setup(in8,GPIO.OUT)
GPIO.setup(en4,GPIO.OUT)
GPIO.output(in7,GPIO.LOW)
GPIO.output(in8,GPIO.LOW)
p4=GPIO.PWM(en4,1000)
p4.start(25)

trig = 36
echo = 37
GPIO.setup(trig,GPIO.OUT)
GPIO.setup(echo,GPIO.IN)


def now():
    return round(time.time() * 1000)

def get_webcams():
    port_ids = []
    for port in range(5):
        print("Looking for a camera in port %s:" %port)
        camera = cv2.VideoCapture(port)
        if camera.isOpened():
            ret = camera.read()[0]
            if ret:
                backendName =camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print('width',w,'height',h)
                print("Camera %s (%s x %s) found in port %s " %(backendName,h,w, port))
                port_ids.append(port)
            camera.release()
    return port_ids

def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def help():
    print('python classify.py <modelPath> <Camera port ID, only required when more than 1 camera is present>')

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "h", ["--help"])
    except getopt.GetoptError:
        help()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            help()
            sys.exit()

    if len(args) == 0:
        help()
        sys.exit(2)

    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

    if searching or dropping:
        with ImageImpulseRunner(modelfile) as runner:
            try:
                count = 0
                model_info = runner.init()
                print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
                labels = model_info['model_parameters']['labels']
                if len(args)>= 2:
                    videoCaptureDeviceId = int(args[1])
                else:
                    port_ids = get_webcams()
                    if len(port_ids) == 0:
                        raise Exception('Cannot find any webcams')
                    if len(args)<= 1 and len(port_ids)> 1:
                        raise Exception("Multiple cameras found. Add the camera port ID as a second argument to use to this script")
                    videoCaptureDeviceId = int(port_ids[0])

                camera = cv2.VideoCapture(videoCaptureDeviceId)
                ret = camera.read()[0]
                if ret:
                    backendName = camera.getBackendName()
                    global width, height
                    w = camera.get(3)
                    h = camera.get(4)
                    width = 320
                    height = 320
                    print("Camera %s (%s x %s) in port %s selected." %(backendName,h,w, videoCaptureDeviceId))
                    camera.release()
                else:
                    raise Exception("Couldn't initialize selected camera.")

                next_frame = 0 # limit to ~10 fps here

                for res, img in runner.classifier(videoCaptureDeviceId):
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)

                    #print('classification runner response', res)

                    if "classification" in res["result"].keys():
                        print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                        for label in labels:
                            score = res['result']['classification'][label]
                            print('%s: %.2f\t' % (label, score), end='')
                        print('', flush=True)

                    elif "bounding_boxes" in res["result"].keys():
                        print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                        goToObject(res)
                        for bb in res["result"]["bounding_boxes"]:
                            # if bb['label'] == 'raspberry-pi':
                            #     count+=1
                            #     print('Spotted Raspberry Pi ' + str(count) + ' times!')
                            print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                            img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
                    if (show_camera):
                        cv2.imshow('edgeimpulse', img)
                        #print(cv2.getWindowImageRect('edgeimpulse'))
                        if cv2.waitKey(1) == ord('q'):
                            break

                    next_frame = now() + 100
            finally:
                if (runner):
                    runner.stop()

def goToObject(result):
    result = result['result']['bounding_boxes']
    for bb in result:
        if searching:
            if bb['label'] == 'cloth':
                minRange = (width/2)-40#(bb['width']/2)-150
                maxRange = (width/2)+40#(bb['width']/2)+25
                midPoint = bb['x']+(bb['width']/2)
                print('minRange',minRange,'x',bb['x'],'x+width',bb['x']+bb['width'],'maxRange',maxRange, 'x+width/2',bb['x']+(bb['width']/2))
                if midPoint >= minRange and midPoint <= maxRange:
                    print(getDistance(), 'cm')
                    print('in front of robot')
                    while getDistance() >= 9:
                        goForward()
                    stopMotor()
                    grab()
                else:
                    p.ChangeDutyCycle(25)
                    p2.ChangeDutyCycle(25)
                    while midPoint < minRange:
                        goLeft()
                    while midPoint > maxRange:
                        goRight()
                    stopMotor()
                    print('not in front of robot')
    #print(type(result))

def goForward():
    p.ChangeDutyCycle(57)
    p2.ChangeDutyCycle(75)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    # motor1.antiClockwise()
    # motor2.clockwise()

def goBack():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    # motor1.clockwise()
    # motor2.antiClockwise()

def goLeft():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    # motor1.clockwise()
    # motor2.clockwise()

def goRight():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    # motor1.antiClockwise()
    # motor2.antiClockwise()

def stopMotor():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    #motor.stop()

def grab():
    GPIO.output(in7,GPIO.HIGH)
    GPIO.output(in8,GPIO.LOW)
    time.sleep(1.75)
    GPIO.output(in7,GPIO.LOW)
    GPIO.output(in8,GPIO.LOW)
    GPIO.output(in5,GPIO.HIGH)
    GPIO.output(in6,GPIO.LOW)
    time.sleep(0.33)
    GPIO.output(in5,GPIO.LOW)
    GPIO.output(in6,GPIO.LOW)
    GPIO.output(in7,GPIO.LOW)
    GPIO.output(in8,GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(in7,GPIO.LOW)
    GPIO.output(in8,GPIO.LOW)

def drop():
    GPIO.output(in5,GPIO.LOW)
    GPIO.output(in6,GPIO.HIGH)
    time.sleep(0.3)
    GPIO.output(in5,GPIO.LOW)
    GPIO.output(in6,GPIO.LOW)

def getDistance():
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start = time.time()
    stop = time.time()

    while GPIO.input(echo) == 0:
        start = time.time()
        
    while GPIO.input(echo) == 1:
        stop = time.time()

    timeElapsed = stop - start
    distance = (timeElapsed * 34300) / 2

    return distance

if __name__ == "__main__":
    main(sys.argv[1:])
    GPIO.cleanup()