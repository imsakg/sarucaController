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
import os
import torch
import numpy as np


t1 = time.time()
import sys
# set model params
#model_path = "yolov5/weights/up-downrect-batch49.pt" # it automatically downloads yolov5s model to given path
#device = "cuda" # or "cpu"
# init yolov5 model
#yolov5 = YOLOv5(model_path, device)

t2 = time.time()
print(t2-t1)
#cap = cv2.VideoCapture(0)
tb = time.time()

HEIGHT=640
WIDTH=480
flag=1

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
def son():
    pilot.changeMode("ALT_HOLD")
    setDepth(-0.7)
    time.sleep(30)
    pilot.changeMode("SURFACE")
    time.sleep(2)
    pilot.changeMode()

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
    firstRotate=pilot.vehicle.heading -65
    print(firstRotate)
    time.sleep(1)
    pilot.changeMode("ALT_HOLD")
    setDepth(-1.2)
    roll(10,5)
    tekrar=3
    i =0
    for i in range(3):
        i=i+1
        #rotate(firstRotate,False)
        time.sleep(2)
        pilot.changeMode("ALT_HOLD")
        time.sleep(2)
        setDepth(-1.2)
        time.sleep(2)
        #callYolo()
        time.sleep(2)
        roll(-10,10)
        #rotate(firstRotate,False)
        #time.sleep(2)
        #roll(20,10)
        time.sleep(2)
        roll(10,10)
        #setDepth(-0.7)
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
        #pitch(10,15)
        #time.sleep(2)
        #rotate(-90)
        #time.sleep(2)
        #pitch(10,6)
        #time.sleep(2)
        #rotate(-90)
        #time.sleep(2)
        #pitch(10,8)
        pilot.changeMode()


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
    HEAD = pilot.vehicle.heading

    if inDepth:
        pilot.changeMode("ALT_HOLD")
        setDepth(depthness)
    else:
        pilot.changeMode("STABILIZE")
    time.sleep(4)
    for i in range(itter):
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

def rollrun():
    print("roll calisti")
    roll(10,1)

def frameQuery(results):
    resArray=results.xyxy[0].detach().to("cpu").numpy()
    if resArray.size > 1:
        t3 = time.time()
        results.save(save_dir='results/')
        os.rename("results/image0.jpg", f"results/img{0}.jpg" )
        print(results.xyxy[0])
        resArray=results.xyxy[0].detach().to("cpu").numpy()
        print(resArray)
        print(type(resArray))
        if resArray.size != 0:
            newdata= np.zeros((resArray.shape[0],10),dtype=np.float32)
            for i in range(len(resArray)):
            
                newdata[i][0]=x1=resArray[i][0]  #x1
                newdata[i][1]=y1=resArray[i][1]   #y1
                newdata[i][2]=x2=resArray[i][2]   #x2
                newdata[i][3]=y2=resArray[i][3]   #y2
                newdata[i][4]=conf=resArray[i][4] #confidence
                newdata[i][5]=myclass =resArray[i][5]  #class


                t4=time.time()
            
                newdata[i][6]=t4-t3  #time

                newdata[i][7]= ((HEIGHT*WIDTH)/ ((x2-x1)*(y2-y1)))/100 #area

                newdata[i][8]= (x1+x2)/2  #MeanX

                newdata[i][9]= (y1+y2)/2 #MeanY


            np.set_printoptions(suppress=True)
            print(newdata)
            newDataUprects = np.where(newdata ==0 )
            if(len(newDataUprects[0])==1):
                index=newDataUprects[0][0]
                print('index uprect:',newDataUprects[0][0])
                meanX=newdata[index][8]
                print('meanX:',meanX)
                if(meanX<=280):
                    roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>320):
                    roll(10,1)
                    print("10 hizla 1 sn saga git")
                elif(meanX>=280 and meanX<=320 and myArea<0.8):
                    pitch(10,3)
                    print("10 hizla 1 sn ileri git")
                elif(meanX>=280 and meanX<=320 and myArea>=0.8):
                    pitch(10,5)
                    print("10 hizla 5 sn ileri git")
            elif(len(newDataUprects[0])>1):
                is1 = np.where(newdata ==1)[0]
                only0 = np.delete(newdata, (is1), axis=0)
                print('sadece 0lar:',only0)
                maxInRows = np.argmax(only0[:,4])
                print('uprectin conf maksi:',maxInRows)
                meanX=only0[maxInRows][8]
                print('meanX:',meanX)
                if(meanX<=280):
                    roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>320):
                    roll(10,1)
                    print("10 hizla 1 sn saga git")
                elif(meanX>=280 and meanX<=320 and myArea<0.8):
                    pitch(10,3)
                    print("10 hizla 1 sn ileri git")
                elif(meanX>=280 and meanX<=320 and myArea>=0.8):
                    pitch(10,5)
                    print("10 hizla 5 sn ileri git")

            elif(len(newDataUprects[0])==0):
                print("hic uprect yok - downrect i yazdiriyorum")
                downRects = np.where(newdata == np.amax(newdata[:,4]))
                index=downRects[0][0]
                print('index downrect:',downRects[0][0])
                meanX=newdata[index][8]
                myArea=newdata[index][7]
                print('meanX:',meanX)
                if(meanX<280):
                    roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>320):
                    roll(10,1)
                    print("10 hizla 1 sn saga git")
                elif(meanX>=280 and meanX<=320 and myArea<0.8):
                    pitch(10,3)
                    print("10 hizla 1 sn ileri git")
                elif(meanX>=280 and meanX<=320 and myArea>=0.8):
                    pitch(10,5)
                    print("10 hizla 5 sn ileri git")

            #np.save("newdata.npy", newdata)

        print(time.time()-tb)
        callYolo()
    elif results is None:
        flag=0

def callYolo():
    while 1:
        success,frame = cap.read()
        if(success and frame is not None):
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = yolov5.predict(frame)
            frameQuery(results)
            if flag==0:
                break
        else:
            print("empty frame")


def parsePoses(box):
    try:
        x = box[0][0].item()
        y = box[0][1].item()
        w = box[0][2].item()
        h = box[0][3].item()
        return x,y,w,h
    except:
        return None


def scanning(itter = 4, pitchT = 4, rollT = 27, inDepth = True, depthness = -0.6):
    HEAD = pilot.vehicle.heading
    pilot.changeMode("ALT_HOLD")
    rotate(HEAD,False)


#Herşey yolunda gitti ise dükkanı kapatabiliriz :)
#cap.release()
#out.release()
#cv2.destroyAllWindows()
