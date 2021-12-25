from pilot import mrPilot
from control import spiralScanning
import time
from threading import Thread
from control.exposer import Exposer
import numpy as np
import cv2
import datetime
import time

expose = Exposer().expose

SIMM = False #! Update
movements = spiralScanning.spiralMovements()

def parsePoses(box):
    try:
        x = box[0][0].item()
        y = box[0][1].item()
        w = box[0][2].item()
        h = box[0][3].item()
        return x,y,w,h
    except:
        return None

def parseScore(score):
    try:
        score = float(score[0].item())*100
    except:
        score = None
    return score

def predict(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = yolov5.predict(frame)
    predictions = results.pred[0]
    boxes = predictions[:, :4] 
    scores = predictions[:, 4]
    categories = predictions[:, 5]
    return predictions, boxes, scores, categories, time.time()
"""
if SIMM:
    cap = cv2.VideoCapture(1) # update with 2
else:
    cap = cv2.VideoCapture(2) # update with 2
"""
pilot = mrPilot.Pilot()
pilot.launch()
pilot.changeMode("MANUAL")
pilot.armForce()

DEPTH = 0
hasCommand = False

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
def resetSticks(): 
    roll(0,0.1)
    pitch(0,0.1)
def stabilize():
    pilot.changeMode("STABILIZE")
def althold():
    pilot.changeMode("ALT_HOLD")
def poshold():
    pilot.changeMode("POSHOLD")
preded = False
lastPred = []
lastPredT = time.time() 
lastScore = 0.0
exposed = []
def startVision():
    global lastPredT, lastPred, preded, lastScore, exposed
    while True:
        _,img = cap.read()
        if _:
            presults, pboxes, pscores, pcategories, pT = predict(img)
            pBox = parsePoses(pboxes)
            pscore = parseScore(pscores)
            if (pBox is not None) and (pscore is not None):
                if (pscore > 70):
                    lastPred = pBox
                    lastPredT = time.time()
                    preded = True
                    lastScore = pscore
                    exposed = expose(lastPred[0], lastPred[1], lastPred[2], lastPred[3])
                    #print(lastPred[0], lastPred[1], lastPred[2], lastPred[3], "\n")
                    #print(f"preded = {preded} \t last score = {lastScore} \t last prediction time = {lastPredT} \n{exposed}")
            else:
                lastScore = 0.0
                preded = False
            cv2.waitKey(1)

def startVisionTH():
    th = Thread(target = startVision)
    th.start()
    return th
def poser():
    exposed

def interupt():
    resetSticks()
    time.sleep(2)
    
    althold()
    time.sleep(1)
    setDepth(-2)
    throttle(-25)
    time.sleep(25)
    stabilize()
def scanLinear(itter = 20, pitchT = 5, rollT = 50, rollS = 10, pitchS =15, inDepth = False, depthness = -2):
    HEAD = pilot.vehicle.heading

    if inDepth:
        pilot.changeMode("ALT_HOLD")
        setDepth(depthness)
    else:
        pilot.changeMode("STABILIZE")

    rotate(HEAD,False)
    time.sleep(5)
    pitch(20,15) 
    for itt in range(itter):
        for pitchitt in range(pitchT):
            pitch(-15,1)
            if preded:
                break
        time.sleep(2)
        rotate(hhead,False)
        time.sleep(2)
        if preded:
            break
        for itt in range(rollT-2):
            roll(-20, 1)
            if preded:
                break

        if preded:
            break
        
        time.sleep(2)
        rotate(hhead,False)
        time.sleep(2)
        
        for itt in range(pitchT):
            if preded:
                break
            pitch(-15,1)
        
        time.sleep(2)
        rotate(hhead,False)
        time.sleep(2)
        if preded:
            break
        
        for itt in range(rollT+2):
            if preded:
                break
            roll(20, 1)

        time.sleep(2)
        rotate(hhead,False)
        time.sleep(2)
        if preded:
            break
    interupt()

class mission:
    def __init__(self):
        self.TARGET_HEAD = 180
    
    def startScan():
        pass

    def scanInterup(self):
        pass

    def backWorker(self):
        pass
stabilize()
visionTH = startVisionTH()
def printer():
    while True:    
        print(f"preded = {preded} \t last score = {lastScore} \t last prediction time = {lastPredT} \n{exposed}")
        time.sleep(3)
prth = Thread(target=printer)
#prth.start()
rotate(45, False)
