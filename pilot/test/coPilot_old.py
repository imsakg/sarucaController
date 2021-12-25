from pilot import mrPilot
from control import spiralScanning
import time
from threading import Thread
from vision.detector import detection
from vision import vision
from yolov5 import YOLOv5
from control.exposer import Exposer
import numpy as np
import cv2
import datetime
import time



expose = Exposer().expose

model_path = "yolov5/weights/circle-train-weights-v30-best.pt"

device = "cuda" # or "cpu"

Vision = vision.Vision

yolov5 = YOLOv5(model_path, device)
movements = spiralScanning.spiralMovements()

cap = cv2.VideoCapture(2)
#Codec tanımlama ve VideoWriter nesnesi(object) oluşturma
fourcc = cv2.VideoWriter_fourcc(*'XVID')
filename = str(datetime.datetime.now().strftime("ON-%Y%m%d-%H%M%S")) + ".avi"
out = cv2.VideoWriter(filename, fourcc, 20.0, (640,480))

capp = cv2.VideoCapture(0)
#Codec tanımlama ve VideoWriter nesnesi(object) oluşturma
fourccc = cv2.VideoWriter_fourcc(*'XVID')
filenamee = str(datetime.datetime.now().strftime("ALT-%Y%m%d-%H%M%S")) + ".avi"
outt = cv2.VideoWriter(filenamee, fourccc, 20.0, (640,480))




def parsePoses(box):
    try:
        x = box[0][0].item()
        y = box[0][1].item()
        w = box[0][2].item()
        h = box[0][3].item()
        return x,y,w,h
    except:
        return None

# if parsePoses(boxes) is not None:
#    pilot.changeMode("STABILIZE")

print(list(movements))

def predict(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = yolov5.predict(frame)
    predictions = results.pred[0]
    boxes = predictions[:, :4] 
    scores = predictions[:, 4]
    categories = predictions[:, 5]
    results.save(save_dir='results/')
    os.rename("results/image0.jpg", f"results/img{i}.jpg" )
    return results, boxes, scores, categories, time.time()

cap = cv2.VideoCapture(0)

pilot = mrPilot.Pilot()
pilot.launch()
pilot.changeMode("MANUAL")
pilot.armForce()
DEPTH = 0

def speed2pwm(speed):
    if speed > 0:
        pwm = 1500 + speed * 10
        return int(pwm)
    else:
        pwm = 1500 + speed * 10
        return int(pwm)


def goT(channel, pwm, duration):
    global DEPTH
    t1 = time.time()
    while time.time() - t1 < duration:
        pilot.channelOverRide(channel, pwm)
        setDepth(DEPTH)
        time.sleep(1)
    pilot.channelOverRide(channel, 1500)


def gotoTest(north, east, down):
    vehicle = pilot.goto_position_target_local_ned(north, east, down)

def rotate(value,relative=True):
    if relative:
        value = pilot.vehicle.heading + value
        pilot.set_target_attitude(0, 0, value)
    else: 
        pilot.set_target_attitude(0, 0, value)

def setDepth(depth):
    global DEPTH
    #pilot.changeMode("ALT_HOLD")
    pilot.set_target_depth(depth)
    DEPTH = depth

def dive(depth, tolorance=0.7, diveSpeed=-15):
    pilot.changeMode("ALT_HOLD")
    while pilot.vehicle.location.global_relative_frame.alt >= tolorance * depth:
        # print(pilot.vehicle.location.global_relative_frame.alt)
        pwm = speed2pwm(diveSpeed)
        pilot.channelOverRide(3, pwm)
    pilot.channelOverRide(3, 1500)


def spin(rotation="left", speed=30):
    roll_angle = pitch_angle = 0
    if rotation == "left":
        # spin the other way with 3x larger steps
        for yaw_angle in range(500, 0, -speed):
            pilot.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
            time.sleep(1)
    elif rotation == "right":
        for yaw_angle in range(0, 500, speed):
            pilot.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
            time.sleep(1)  # wait for a second
    else:
        raise Exception("Unknown rotation")


def roll(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=6, pwm=pwm, duration=duration)


def throotle(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def yaw(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=4, pwm=pwm, duration=duration)


def pitch(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=5, pwm=pwm, duration=duration)

def transform(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=2, pwm=pwm, duration=duration)


def forward(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=5, pwm=pwm, duration=duration)


def back(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def right(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def left(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def down(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def up(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def testDrone():
    pilot.changeMode("GUIDED")
    pilot.arm()
    pilot.takeoff(20)

    while True:
        for x, y in movements:
            currentLocation, targetLocation, targetDistance = pilot.goto(x, y)
            while pilot.vehicle.mode.name == "GUIDED":
                remainingDistance = pilot.get_distance_metres(
                    pilot.vehicle.location.global_relative_frame, targetLocation
                )
                print("Distance to target: ", remainingDistance, targetDistance * 0.2)
                if remainingDistance <= targetDistance * 0.2:
                    print("Reached target")
                    break
                time.sleep(1)

def startPath(firstRotate=pilot.vehicle.heading):
    time.sleep(3)
    firstRotate=pilot.vehicle.heading
    print(firstRotate)
    time.sleep(1)
    pilot.changeMode("STABILIZE")
    rotate(firstRotate,False)
    time.sleep(2)
    roll(-20,5) #sola 20h 5sn
    rotate(firstRotate,False)
    time.sleep(2)
    roll(20,10)
    time.sleep(2)
    setDepth(-0.7) 
    pilot.changeMode("ALT_HOLD")
    time.sleep(2)
    setDepth(-0.4)
    setDepth(-0.5)
    time.sleep(2)
    setDepth(-0.6)
    setDepth(-0.7)
    setDepth(-0.8)
    time.sleep(7)
    setDepth(0)
    pilot.changeMode("STABILIZE")
    pitch(10,6)
    time.sleep(2)
    rotate(-90)
    time.sleep(2)
    pitch(10,6)
    time.sleep(2)
    rotate(-90)
    time.sleep(2)
    pitch(10,6)
    for i in range(8):
        rotate(90)
        time.sleep(2)
    firstRotate-=180
    rotate(firstRotate,False)
    time.sleep(3)
    pitch(10,2)
    pilot.changeMode()

def startyfc(firstRotate=pilot.vehicle.heading):
    time.sleep(3)
    firstRotate=pilot.vehicle.heading
    print(firstRotate)
    time.sleep(1)
    pilot.changeMode("STABILIZE")
    rotate(firstRotate,False)
    time.sleep(2)
    #roll(-20,5)
    rotate(firstRotate,False)
    time.sleep(2)
    #roll(20,10)
    time.sleep(2)
    setDepth(-0.7)
    #pilot.changeMode("ALT_HOLD")
    #time.sleep(2)
    #setDepth(-0.4)
    #setDepth(-0.5)
    #time.sleep(2)
    #setDepth(-0.6)
    #setDepth(-0.7)
    #setDepth(-0.8)
    #time.sleep(7)
    #setDepth(0)
    #pilot.changeMode("STABILIZE")
    pitch(10,15)
    time.sleep(2)
    rotate(-90)
    time.sleep(2)
    pitch(10,6)
    time.sleep(2)
    rotate(-90)
    time.sleep(2)
    pitch(10,8)
    


def zigzagTest():
    for i in range(3):
        for x,y in movements:
            print(x,y)

def startVisionRec():
    while True:
        _,img = cap.read()
        if _:
            response=predict(img)
        cv2.waitKey(1)

def startRecordTest():
    return None
    return Thread(target = startVisionRec).start()

def startZigZag(itter = 4, pitchT = 4, rollT = 27, inDepth = True, depthness = -0.6):
    HEAD = pilot.vehicle.heading + 45

    if inDepth:
        pilot.changeMode("ALT_HOLD")
        setDepth(depthness)
    else:
        pilot.changeMode("STABILIZE")
    time.sleep(5)
    
    for i in range(itter):
        for itt in range(pitchT):
            pitch(10,1)
            setDepth(depthness)
        
        time.sleep(2)
        rotate(HEAD,False)
        time.sleep(2)
        for itt in range(rollT):
            roll(-10, 1)
            setDepth(depthness)
            _,img = cap.read()
            results, boxes, scores, categories = predict(img)
            if parsePoses(boxes) is not None:
                pilot.changeMode("STABILIZE")
        
        time.sleep(2)
        rotate(HEAD,False)
        time.sleep(2)
        for itt in range(pitchT):
            pitch(10,1)
            setDepth(depthness)
            _,img = cap.read()
            results, boxes, scores, categories = predict(img)
            if parsePoses(boxes) is not None:
                pilot.changeMode("STABILIZE")
        
        
        time.sleep(2)
        rotate(HEAD,False)
        time.sleep(2)
        for itt in range(rollT-3):
            roll(10, 1)
            setDepth(depthness)
            _,img = cap.read()
            results, boxes, scores, categories = predict(img)
            if parsePoses(boxes) is not None:
                pilot.changeMode("STABILIZE")
        
        time.sleep(2)
        rotate(HEAD,False)
        time.sleep(2)


def denemeFon():
    print('olmadı')

#Herşey yolunda gitti ise dükkanı kapatabiliriz :)
#cap.release()
#out.release()
#cv2.destroyAllWindows()
