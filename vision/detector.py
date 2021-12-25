import numpy as np
import cv2

# rLo, rHi, gLo, gHi, bLo, bHi = 120,165,139,186,160,200
hLo, hHi, sLo, sHi, vLo, vHi = 0, 65, 69, 255, 100, 168


def detection(imageFrame):
    blur = cv2.GaussianBlur(imageFrame, (11, 11), 0)
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    blue_lower = np.array([hLo, sLo, vLo], np.uint8)
    blue_upper = np.array([hHi, sHi, vHi], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    dilateKernel = np.ones((9, 9), "uint8")
    erodeKernel = np.ones((15, 15), "uint8")
    morphKernel = np.ones((15, 15), "uint8")

    floodFillTH, thresh = cv2.threshold(blue_mask, 220, 255, cv2.THRESH_BINARY)
    blur = cv2.medianBlur(thresh, ksize=3)
    blur = cv2.medianBlur(blur, ksize=3)
    blur = cv2.medianBlur(blur, ksize=3)
    blur = cv2.medianBlur(blur, ksize=3)
    erode = cv2.erode(blur, None, iterations=1)
    dilate = cv2.dilate(erode, (7,7),iterations=5)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=dilate)

    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    reContour = None
    reArea = 0
    for pic, contour in enumerate(contours):
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        n = len(approx)
        area = cv2.contourArea(contour)
        if (area >= 4) and (n > 3):
            if area > reArea:
                reArea = area
                reContour = contour
    
    x, y, w, h = cv2.boundingRect(reContour)
    if not (w==0 and h==0):
        return imageFrame, [x, y, w, h]
    return imageFrame, None
