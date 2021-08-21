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

runner = None
show_camera = True
searching = True
found = False
dropping = False
taskDone = False

    #    if 'cloth' in predictions and searching:
    #        clothes = predictions[predictions['label']=='cloth']
    #        sortedResults = sorted(clothes, key=lambda x: (clothes[x]['width']))
    #        print(sortedResults)

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
                    width = w
                    height = h
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
                minRange = (width/2)-(bb['width']/2)-10
                maxRange = (width/2)+(bb['width']/2)+10
                if bb['x'] >= minRange and bb['x']+bb['width'] <= maxRange:
                    goForward()
                    time.sleep(1.5)
                    stopMotor(motor1)
                    stopMotor(motor2)
                else:
                    print('not in front of robot')
    print(type(result))

def goForward():
    motor1.antiClockwise()
    motor2.clockwise()

def goBack():
    motor1.clockwise()
    motor2.antiClockwise()

def goLeft():
    motor1.clockwise()
    motor2.clockwise()

def goRight():
    motor1.antiClockwise()
    motor2.antiClockwise()

def stopMotor(motor: m):
    motor.stop()

if __name__ == "__main__":
    motor1 = m(16,18,22)
    motor2 = m(11,13,15)
    motor3 = m(19,21,23)
    motor4 = m(29,31,33)

    grabber = g(motor3, motor4)

    goForward()
    time.sleep(3)
    stopMotor(motor1)
    stopMotor(motor2)
    main(sys.argv[1:])
    GPIO.cleanup()