import cv2
from vision.detector import detection
from queue import Queue
import time
import datetime
from utils.visionType import visionType
from utils.visionQueue import visionQueue
from control.exposer import Exposer

expose = Exposer().expose
verbose = True

q = visionQueue(maxSize=30)

COORDS = []

def Vision(img, tcords):
    global COORDS
    
    response, coords = detection(img)
    if coords is not None:
        q.put(visionType(coords, time.time()), True)
        coordsx = coords
        x, y, w, h = coordsx
        x, y, w, h = int(x), int(y), int(w), int(h)
        response = cv2.rectangle(response, (x, y), (x + w, y + h), (255, 198, 109), 2)
        print(x,y,w,h)
        cv2.putText(
            response, "Position", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (134, 76, 190)
        )

    if (q.qSize() > 1):
        # print(q.get(overdel=False))
        coordsx = q.getAccurs()
        x, y, w, h = coordsx
        x, y, w, h = int(x), int(y), int(w), int(h)
        response = cv2.rectangle(response, (x, y), (x + w, y + h), (0, 0, 255), 4)

        cv2.putText(
            response,
            "Filtered Position",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (254, 76, 190),
        )
        exps = str(expose(x,y,w,h))
        print(exps)
        cv2.putText(
                response,
                exps,
                (10,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0,255,190),
                )
    recorder.writeImage2Record(response)
    # record.write(response)

