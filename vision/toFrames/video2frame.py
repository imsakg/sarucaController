import os 
import cv2
import time

videoTypes = ['mp4', 'avi', 'flv', 'mkv', 'mpeg', "AVI"]

input_path = 'inputs/'
output_path = 'outputs/'

videos = os.listdir(input_path)

def toDigits(num):
    return (str(num).zfill(6))

def toFrames(videoPath,outputPath):
    cap = cv2.VideoCapture(videoPath)
    ret, frame = cap.read()
    frame_count = 0
    while ret:
        filename = toDigits(frame_count) + '.jpg'
        cv2.imwrite(outputPath + "/" + filename, frame)
        ret, frame = cap.read()
        frame_count += 1
    
    cap.release()

for video in videos:
    if video.split('.')[-1] in videoTypes:
        folderName = str(video.split('.')[0])
        try:
            os.mkdir(output_path + folderName)
        except:
            if input(video + " is extracted and folder is exist, want to remove and try it again ? (y (Yes) / n (No))") == "y":
                # delete all files on folder
                for file in os.listdir(output_path + folderName):
                    os.remove(output_path + folderName + "/" + file)
                os.rmdir(output_path + folderName)
                os.mkdir(output_path + folderName)
            else:
                continue
        toFrames(input_path + video, output_path + folderName)
