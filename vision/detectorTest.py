import numpy as np
import cv2

hLo, hHi, sLo, sHi, vLo, vHi = 0, 52, 66, 250, 100, 168

cap = cv2.VideoCapture("/dev/video2")

def nothing(x):
    pass

cv2.namedWindow("track")
cv2.createTrackbar("hLo","track",hLo,180,nothing)
cv2.createTrackbar("hHi","track",hHi,180,nothing)
cv2.createTrackbar("sLo","track",sLo,255,nothing)
cv2.createTrackbar("sHi","track",sHi,255,nothing)
cv2.createTrackbar("vLo","track",vLo,255,nothing)
cv2.createTrackbar("vHi","track",vHi,255,nothing)


def detection(imageFrame):
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    
    hLo, hHi, sLo, sHi, vLo, vHi = (
            cv2.getTrackbarPos("hLo","track"),
            cv2.getTrackbarPos("hHi","track"),
            cv2.getTrackbarPos("sLo","track"),
            cv2.getTrackbarPos("sHi","track"),
            cv2.getTrackbarPos("vLo","track"),
            cv2.getTrackbarPos("vHi","track"))

    blue_lower = np.array([hLo, sLo, vLo], np.uint8)
    blue_upper = np.array([hHi, sHi, vHi], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    dilateKernel = np.ones((9, 9), "uint8")
    erodeKernel = np.ones((15, 15), "uint8")
    morphKernel = np.ones((15, 15), "uint8")

    floodFillTH, thresh = cv2.threshold(blue_mask, 220, 255, cv2.THRESH_BINARY)
    blur = cv2.medianBlur(thresh,ksize=3)
    erode = cv2.erode(blur, None, iterations=1)
    dilate = cv2.dilate(erode, (7,7), iterations=2)

    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=dilate)

    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )

    for pic, contour in enumerate(contours):
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        n = len(approx)
        area = cv2.contourArea(contour)
        if (area > 30) and (n > 10):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(
                imageFrame, (x, y), (x + w, y + h), (255, 198, 109), 2
            )
            cv2.putText(
                imageFrame,
                "Blue",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (134, 76, 190),
            )

            return (
                imageFrame,
                [x, y, w, h],
                res_blue,
                erode,
                dilate,
                blue_mask,
                cv2.drawContours(np.zeros(imageFrame.shape), contours, pic, 255, -1),
                thresh,
                blur,
            )
    return (
        imageFrame,
        None,
        res_blue,
        erode,
        dilate,
        blue_mask,
        np.zeros(imageFrame.shape),
        thresh,
        blur,
    )


while True:
    _, img = cap.read()
    (
        img,
        coords,
        res_blue,
        erode,
        dilate,
        blue_mask,
        contours,
        thresh,
        blur,
    ) = detection(img)
    img = cv2.resize(img, (200, 200))
    res_blue = cv2.resize(res_blue, (200, 200))
    erode = cv2.resize(erode, (200, 200))
    contours = cv2.resize(contours, (200, 200))
    dilate = cv2.resize(dilate, (200, 200))
    blue_mask = cv2.resize(blue_mask, (200, 200))
    thresh = cv2.resize(thresh, (200, 200))
    blur = cv2.resize(blur, (200, 200))
    cv2.imshow("blue_mask", blue_mask)
    cv2.imshow("contours", contours)
    cv2.imshow("dilate", dilate)
    cv2.imshow("contours", contours)
    cv2.imshow("erode", erode)
    cv2.imshow("res_blue", res_blue)
    cv2.imshow("frame", img)
    cv2.imshow("thresh", thresh)
    cv2.imshow("blur", blur)
    cv2.moveWindow("blue_mask", 0, 0)
    cv2.moveWindow("dilate", 0, 250)
    cv2.moveWindow("contours", 0, 500)
    cv2.moveWindow("erode", 400, 0)
    cv2.moveWindow("res_blue", 400, 250)
    cv2.moveWindow("frame", 400, 500)
    cv2.moveWindow("thresh", 800, 250)
    cv2.moveWindow("blur", 800, 500)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
