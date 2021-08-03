import cv2
import datetime
import time


filename = str(datetime.datetime.now().strftime("%Y%m%d%H%M%S"))+".avi"
fourcc = cv2.VideoWriter_fourcc(*"XVID")
record =  cv2.VideoWriter(filename,fourcc,10,(640,480))

def newRecord():
    global record
    filename = str(datetime.datetime.now().strftime("%Y%m%d%H%M%S"))+".avi"
    record.release()
    record = cv2.VideoWriter(filename,fourcc,10,(640,480))

def writeImage2Record(image):
    global record
    record.write(image)