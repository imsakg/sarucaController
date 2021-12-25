from yolov5 import YOLOv5
import cv2
import time
import os
import torch
#from pilot import coPilot
import numpy as np

t1 = time.time()
import sys
# set model params
model_path = "yolov5/weights/up-downrect-batch49.pt" # it automatically downloads yolov5s model to given path
device = "cuda" # or "cpu"
# init yolov5 model
yolov5 = YOLOv5(model_path, device)

t2 = time.time()
print(t2-t1)
cap = cv2.imread("orta6.jpg")
_,frame = cap.read()
tb = time.time()
i = 0
while _:

    if i<0:
        continue

    if i>5:
        break

    #_,frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    t3 = time.time()
    results = yolov5.predict(frame)

    if results != '':
        i+=1
        HEIGHT=640
        WIDTH=480
        

        results.save(save_dir='results/')
        os.rename("results/image0.jpg", f"results/img{i}.jpg" )
        print(time.time()-t3)
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

                newdata[i][7]= (HEIGHT*WIDTH)/ ((x2-x1)*(y2-y1)) #area

                newdata[i][8]= (x1+x2)/2  #MeanX

                newdata[i][9]= (y1+y2)/2 #MeanY


            np.set_printoptions(suppress=True)
            print(newdata)
            newDataUprects = np.where(newdata ==0 )
            if(len(newDataUprects[0])==1):
                index=newDataUprects[0][0]
                print(newDataUprects[0][0])
                meanX=newdata[index][8]
                print(meanX)
                if(meanX<=280):
                    #roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>320):
                    #roll(10,1)
                    print("10 hizla 1 sn saga git")
            elif(len(newDataUprects[0])>1):
                is1 = np.where(newdata ==1)[0]
                only0 = np.delete(newdata, (is1), axis=0)
                print(only0)
                maxInRows = np.argmax(only0[:,4])
                print(maxInRows)
                meanX=only0[maxInRows][8]
                print(meanX)
                if(meanX<=280):
                    #roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>320):
                    #roll(10,1)
                    print("10 hizla 1 sn saga git")

            elif(len(newDataUprects[0])==0):
                print("hiç uprect yok")
                downRects = np.where(newdata == np.amax(newdata[:,4]))
                index=downRects[0][0]
                print(downRects[0][0])
                meanX=newdata[index][8]
                print(meanX)
                if(meanX<=280):
                    #roll(-10,1)
                    print("10 hizla 1 sn sola git")
                elif(meanX>320):
                    #roll(10,1)
                    print("10 hizla 1 sn saga git")
            
            np.save("newdata.npy", newdata)

            


        # results = yolov5.predict(frame)
        # predictions = results.pred[0]
        # boxes = predictions[:, :4] 
        # scores = predictions[:, 4]
        # categories = predictions[:, 5]

        print(time.time()-tb)

        #results.show()
        break
    


# save results into "results/" folder
