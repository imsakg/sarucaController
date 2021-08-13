import numpy as np
import cv2

rLo, rHi, gLo, gHi, bLo, bHi = 190, 255, 40, 153, 75, 110

cap = cv2.VideoCapture("/dev/video1")


def detection(imageFrame):
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    blue_lower = np.array([bLo, gLo, rLo], np.uint8)
    blue_upper = np.array([bHi, gHi, rHi], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    dilateKernel = np.ones((9, 9), "uint8")
    erodeKernel = np.ones((15, 15), "uint8")
    morphKernel = np.ones((15, 15), "uint8")

    floodFillTH, thresh = cv2.threshold(blue_mask, 220, 255, cv2.THRESH_BINARY)
    morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, morphKernel)
    blur = cv2.medianBlur(morph, ksize=5)
    erode = cv2.erode(blur, None, iterations=2)
    dilate = cv2.dilate(erode, None, iterations=2)

    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=dilate)

    contours, hierarchy = cv2.findContours(
        dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
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
                morph,
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
        morph,
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
        morph,
    ) = detection(img)
    img = cv2.resize(img, (200, 200))
    res_blue = cv2.resize(res_blue, (200, 200))
    erode = cv2.resize(erode, (200, 200))
    contours = cv2.resize(contours, (200, 200))
    dilate = cv2.resize(dilate, (200, 200))
    blue_mask = cv2.resize(blue_mask, (200, 200))
    thresh = cv2.resize(thresh, (200, 200))
    morph = cv2.resize(morph, (200, 200))
    cv2.imshow("blue_mask", blue_mask)
    cv2.imshow("contours", contours)
    cv2.imshow("dilate", dilate)
    cv2.imshow("contours", contours)
    cv2.imshow("erode", erode)
    cv2.imshow("res_blue", res_blue)
    cv2.imshow("frame", img)
    cv2.imshow("thresh", thresh)
    cv2.imshow("morph", morph)
    cv2.moveWindow("blue_mask", 0, 0)
    cv2.moveWindow("dilate", 0, 250)
    cv2.moveWindow("contours", 0, 500)
    cv2.moveWindow("erode", 400, 0)
    cv2.moveWindow("res_blue", 400, 250)
    cv2.moveWindow("frame", 400, 500)
    cv2.moveWindow("thresh", 800, 250)
    cv2.moveWindow("morph", 800, 500)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
