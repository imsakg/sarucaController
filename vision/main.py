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

# filename = "data/"+str(datetime.datetime.now().strftime("%Y%m%d%H%M%S"))+".avi"

q = visionQueue(maxSize=30)
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("/dev/video1")

# fourcc = cv2.VideoWriter_fourcc(*"XVID")
# record =  cv2.VideoWriter(filename,fourcc,10,(640,480))

while 1:
    _, img = cap.read()

    response, coords = detection(img)
    if coords != None:
        q.put(visionType(coords, time.time()), True)
        coordsx = coords
        x, y, w, h = coordsx
        x, y, w, h = int(x), int(y), int(w), int(h)
        response = cv2.rectangle(response, (x, y), (x + w, y + h), (255, 198, 109), 2)

        cv2.putText(
            response, "Position", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (134, 76, 190)
        )

    if (q.qSize()) > 0:
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

        print(expose(x, y, w, h))

    cv2.imshow("re", response)
    # record.write(response)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
