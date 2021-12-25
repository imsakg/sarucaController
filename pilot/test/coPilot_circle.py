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
model_path = "yolov5/weights/up-downrect-batch49.pt" # it automatically downloads yolov5s model to given path
device = "cuda" # or "cpu"
# init yolov5 model
yolov55 = YOLOv5(model_path, device)
cap=cv2.VideoCapture(2)
success,frame = cap.read()
a=yolov55.predict(frame)


t2 = time.time()
print(t2-t1)

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

def startPath(firstRotate=pilot.vehicle.heading):
    time.sleep(3)
    firstRotate=pilot.vehicle.heading
    print(firstRotate)
    time.sleep(1)
    pilot.changeMode("STABILIZE")
    rotate(firstRotate,False)
    time.sleep(2)
    pitch(10,5)
    time.sleep(1)
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
    setDepth(0)
    pitch(10,2)
    pilot.changeMode()

def derineDal():
    print("a")

def startyfc(firstRotate=pilot.vehicle.heading):
    time.sleep(3)
    firstRotate=pilot.vehicle.heading
    print(firstRotate)
    time.sleep(1)
    pilot.changeMode("STABILIZE")
    rotate(firstRotate,False)
    time.sleep(2)
    #roll(-10,5)
    pitch(10,3)
    time.sleep(2)
    rotate(-90)
    time.sleep(2)
    pitch(10,3)
    time.sleep(2)
    callYolo()
    
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

def frameQuery(results,width,height):
    if results is not None:
        t3 = time.time()

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
            widthLeft=((width/2)-40)
            widthRight=((width/2)+40)
            heightUp=((height/2)-40)
            heightDown=((height/2)+40)

            newDataUprects = np.where(newdata ==0 )
            if(len(newDataUprects[0])==1):
                index=newDataUprects[0][0]
                print('index uprect:',newDataUprects[0][0])
                meanX=newdata[index][8]
                meanY=newdata[index][9]
                print('meanX:',meanX)
                print('meanY:',meanY)
                if(meanX<widthLeft):
                    roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>widthRight):
                    roll(10,1)
                    print("10 hizla 1 sn saga git")
                if(meanY<heightDown):
                    pitch(-10,1)
                elif(meanY>heightUp):
                    pitch(10,1)
                elif(meanY<=heightUp and meanY>=heightDown and meanX<=widthRight and meanX >= widthLeft):
                    pilot.changeMode("ALT_HOLD")
                    time.sleep(5)
                    pilot.changeMode("SURFACE")



            elif(len(newDataUprects[0])>1):
                is1 = np.where(newdata ==1)[0]
                only0 = np.delete(newdata, (is1), axis=0)
                print('sadece 0lar:',only0)
                maxInRows = np.argmax(only0[:,4])
                print('uprectin conf maksi:',maxInRows)
                meanX=only0[maxInRows][8]
                meanY=only0[maxInRows][9]
                print('meanX:',meanX)
                if(meanX<=widthLeft): # merkez nokta solda o yuzden sola
                    roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>widthRight): # merkez nokta sagda o yuzden saga
                    roll(10,1)
                    print("10 hizla 1 sn saga git")
                if(meanY>heightUp): # merkez nokta yukarda o yuzden ileri
                    pitch(10,1)
                elif(meanY<heightDown): # merkez nokta altta o yuzden geri
                    pitch(-10,1)
                elif(meanY<=heightUp and meanY>=heightDown and meanX<=widthRight and meanX >= widthLeft):
                    pilot.changeMode("ALT_HOLD")
                    time.sleep(5)
                    pilot.changeMode("SURFACE")

            #np.save("newdata.npy", newdata)

        print(time.time()-tb)
    elif results is None:
        flag=0

def callYolo():
    i=0
    while 1:
        success,frame = cap.read()
        #print('frame sayisi:',len(frame))
        if(success and frame is not None):
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame=cv2.rotate(frame,cv2.ROTATE_90_CLOCKWISE)
            width=480  #.get(cv2.CAP_PROP_FRAME_WIDTH)
            height=640  #.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print('width:',width)
            print('height:',height)
            results = yolov55.predict(frame)
            results.save(save_dir='results/')
            os.rename("results/image0.jpg", f"results/img{i}.jpg" )
            frameQuery(results,width,height)
            if flag==0:
                break
        else:
            print("empty frame")
            break
        i=i+1



#Herşey yolunda gitti ise dükkanı kapatabiliriz :)
#cap.release()
#out.release()
#cv2.destroyAllWindows()
