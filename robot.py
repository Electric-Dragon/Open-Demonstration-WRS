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
dropping = False
found = False
taskDone = False
doingTask = False
recheck = True
startTime = 0
startTime2 = 0
startTime3 = 0
x = 0
y = 0
z = 0

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
p.start(99.9)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
p2=GPIO.PWM(en2,1000)
p2.start(94)

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
p4.start(50)

ir = 36
ir2 = 37
ir3 = 38

GPIO.setup(ir,GPIO.IN)
GPIO.setup(ir2,GPIO.IN)
GPIO.setup(ir3,GPIO.IN)

d = 'l'

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
    global found, y, startTime2, z, startTime3
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

    if searching or dropping and not found:
        with ImageImpulseRunner(modelfile) as runner:
            try:
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

                next_frame = 0

                for res, img in runner.classifier(videoCaptureDeviceId):
                    if (next_frame > now()):
                        time.sleep((next_frame - now()) / 1000)

                    if "classification" in res["result"].keys():
                        print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                        for label in labels:
                            score = res['result']['classification'][label]
                            print('%s: %.2f\t' % (label, score), end='')
                        print('', flush=True)

                    elif "bounding_boxes" in res["result"].keys():
                        #if dropping:
                        #    d = 'r'
                        #else:
                        #    d='l'
                        if y == 0:
                            startTime2 = time.time()
                            y = 1
                        if z == 0:
                            startTime3 = time.time()
                            z = 1

                        if time.time() - startTime2 >=2.1:
                            if len(res['result']['bounding_boxes']) == 0:
                                rotate(d)
                                y = 0
                            else:
                                goToObject(res)
                        print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                        for bb in res["result"]["bounding_boxes"]:
                            print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                            img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
                    if (show_camera):
                        cv2.imshow('The Uchihas', img)
                        #print(cv2.getWindowImageRect('The Uchihas'))
                        if cv2.waitKey(1) == ord('q'):
                            break

                    next_frame = now() + 100
            finally:
                if (runner):
                    runner.stop()

def goToObject(result):
    global searching, dropping, doingTask, found, recheck, x, startTime, startTime2, y, startTime3, z
    if True: #time.time() - startTime2 >= 2.5:
        #y = 0
        result = result['result']['bounding_boxes']
        for bb in result:
            if searching:
                if bb['label'] == 'cloth':
                    minRange = (width/2)-40
                    maxRange = (width/2)+40
                    midPoint = bb['x']+(bb['width']/2)
                    print('minRange',minRange,'x',bb['x'],'x+width',bb['x']+bb['width'],'maxRange',maxRange, 'x+width/2',bb['x']+(bb['width']/2))
                    if midPoint >= minRange and midPoint <= maxRange:
                        found = True
                        doingTask = True
                        print('in front of robot')
                        if recheck:
                            if x == 0:
                                startTime = time.time()
                                x+=1
                            if time.time() - startTime < 3:
                                break
                            else:
                                recheck = False
                            #flag = True
                            #break
                        #if flag:
                        #    break
                        goRight()
                        time.sleep(0.3)
                        stopMotor()
                        time.sleep(0.05)
                        #goForward()
                        #time.sleep(2)
                        while not recheck:
                            #print(GPIO.input(ir))
                            #print(GPIO.input(ir2))
                            #print(GPIO.input(ir3))
                            if GPIO.input(ir) and GPIO.input(ir2) and GPIO.input(ir3):
                                #print('case 1')
                                goForward()
                            elif GPIO.input(ir) and GPIO.input(ir3) and not GPIO.input(ir2):
                                print('case 2')
                                while GPIO.input(ir):# and not GPIO.input(ir2):
                                    goLeft()
                                stopMotor()
                                grab()
                                break
                            elif GPIO.input(ir3) and not GPIO.input(ir) and not GPIO.input(ir2):
                                print('case 3')
                                while GPIO.input(ir):# and not GPIO.input(ir2):
                                    goLeft()
                                stopMotor()
                                grab()
                                break
                            elif GPIO.input(ir) and GPIO.input(ir2) and not GPIO.input(ir3):
                                print('case 4')
                                while GPIO.input(ir):# and not GPIO.input(ir3):
                                    goRight()
                                stopMotor()
                                grab()
                                break
                            elif GPIO.input(ir2) and not GPIO.input(ir) and not GPIO.input(ir3):
                                print('case 5')
                                while GPIO.input(ir):# and not GPIO.input(ir3):
                                    goRight()
                                stopMotor()
                                grab()
                                break
                    else:
                        if time.time() - startTime3 >= 2:
                            z = 0
                            alignRobot(midPoint, minRange, maxRange)
                            doingTask = False
                            print('not in front of robot')
                    break
                else:
                    rotate()
            if dropping:
                if bb['label'] == 'basket':
                    minRange = (width/2)-40
                    maxRange = (width/2)+40
                    midPoint = bb['x']+(bb['width']/2)
                    print('minRange',minRange,'x',bb['x'],'x+width',bb['x']+bb['width'],'maxRange',maxRange, 'x+width/2',bb['x']+(bb['width']/2))
                    if midPoint >= minRange and midPoint <= maxRange:
                        found = True
                        doingTask = True
                        print('in front of robot')
                        if recheck:
                            if x == 0:
                                startTime = time.time()
                                x+=1
                            if time.time() - startTime < 5:
                                break
                            else:
                                recheck = False
                        p4.ChangeDutyCycle(60)
                        GPIO.output(in7,GPIO.HIGH)
                        GPIO.output(in8,GPIO.LOW)
                        time.sleep(0.2)
                        GPIO.output(in7,GPIO.LOW)
                        GPIO.output(in8,GPIO.LOW)
                        p3.ChangeDutyCycle(50)
                        GPIO.output(in5,GPIO.LOW)
                        GPIO.output(in6,GPIO.HIGH)
                        time.sleep(0.4)
                        GPIO.output(in5,GPIO.LOW)
                        GPIO.output(in6,GPIO.LOW)
                        p3.ChangeDutyCycle(25)
                        goRight()
                        time.sleep(0.15)
                        stopMotor()
                        time.sleep(0.05)
                        while not recheck:
                            if GPIO.input(ir) and GPIO.input(ir2) and GPIO.input(ir3):
                                #print('case 1')
                                goForward()
                            elif GPIO.input(ir) and GPIO.input(ir3) and not GPIO.input(ir2):
                                print('case 2')
                                while GPIO.input(ir):# and not GPIO.input(ir2):
                                    goLeft()
                                stopMotor()
                                drop()
                                break
                            elif GPIO.input(ir3) and not GPIO.input(ir) and not GPIO.input(ir2):
                                print('case 3')
                                while GPIO.input(ir) and not GPIO.input(ir2):
                                    goLeft()
                                stopMotor()
                                drop()
                                break
                            elif GPIO.input(ir) and GPIO.input(ir2) and not GPIO.input(ir3):
                                print('case 4')
                                while GPIO.input(ir):# and not GPIO.input(ir3):
                                    goRight()
                                stopMotor()
                                drop()
                                break
                            elif GPIO.input(ir2) and not GPIO.input(ir) and not GPIO.input(ir3):
                                print('case 5')
                                while GPIO.input(ir) and not GPIO.input(ir3):
                                    goRight()
                                stopMotor()
                                drop()
                                break

                    else:
                        alignRobot(midPoint, minRange, maxRange)
                        doingTask = False
                        print('not in front of robot')
                    break
                else:
                    rotate()
        time.sleep(0.4)

def goForward():
    p.ChangeDutyCycle(99.9)
    p2.ChangeDutyCycle(94)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def goBack():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

def goRight():
    p.ChangeDutyCycle(80)
    p2.ChangeDutyCycle(80)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.HIGH)

def goLeft():
    p.ChangeDutyCycle(80)
    p2.ChangeDutyCycle(80)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

def stopMotor():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)

def grab():
    global found, doingTask, searching, dropping, x
    #goForward()
    #time.sleep(0.1)
    #stopMotor()
    p4.ChangeDutyCycle(60)
    GPIO.output(in5,GPIO.HIGH)
    GPIO.output(in6,GPIO.LOW)
    time.sleep(0.6)
    GPIO.output(in5,GPIO.LOW)
    GPIO.output(in6,GPIO.LOW)
    GPIO.output(in7,GPIO.HIGH)
    GPIO.output(in8,GPIO.LOW)
    time.sleep(2.3)
    GPIO.output(in7,GPIO.LOW)
    GPIO.output(in8,GPIO.LOW)
    p3.ChangeDutyCycle(50)
    GPIO.output(in5,GPIO.LOW)
    GPIO.output(in6,GPIO.HIGH)
    time.sleep(0.5)
    p3.ChangeDutyCycle(25)
    GPIO.output(in5,GPIO.LOW)
    GPIO.output(in6,GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(in7,GPIO.HIGH)
    GPIO.output(in8,GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(in7,GPIO.LOW)
    GPIO.output(in8,GPIO.LOW)
    found = False
    doingTask = False
    x = 0
    dropping = not dropping
    searching = not searching

def drop():
    global found, doingTask, searching, dropping, x
    p4.ChangeDutyCycle(28)
    GPIO.output(in7,GPIO.LOW)
    GPIO.output(in8,GPIO.HIGH)
    time.sleep(0.8)
    GPIO.output(in7,GPIO.LOW)
    GPIO.output(in8,GPIO.LOW)
    time.sleep(0.1)
    goBack()
    time.sleep(1.3)
    goLeft()
    time.sleep(0.6)
    stopMotor()
    GPIO.output(in5,GPIO.HIGH)
    GPIO.output(in6,GPIO.LOW)
    time.sleep(0.15)
    GPIO.output(in5,GPIO.LOW)
    GPIO.output(in6,GPIO.LOW)
    found = True
    #doingTask = False
    #x = 0
    dropping = False
    searching = False

def alignRobot(midPoint ,minRange, maxRange):
    print('aligning')
    global recheck, x
    # time.sleep(0.4)
    # p.ChangeDutyCycle(25)
    # p2.ChangeDutyCycle(25)
    if midPoint < minRange:
        goLeft()
        time.sleep(0.07)
        stopMotor()
    if midPoint > maxRange:
        goRight()
        time.sleep(0.07)
        stopMotor()
    recheck = True
    x = 0

def rotate(d = 'l'):
    print('rotating')
    # p.ChangeDutyCycle(25)
    # p2.ChangeDutyCycle(25)
    goLeft()
    time.sleep(0.1)
    stopMotor()


if __name__ == "__main__":
    main(sys.argv[1:])
    GPIO.cleanup()