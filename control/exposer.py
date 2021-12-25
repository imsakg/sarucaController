class Exposer:
    def __init__(self):
        self.W = 640
        self.H = 480
        self.Tw = 100
        self.Th = 120

    def expose(self, x, y, w, h):
        xCentered = False
        yCentered = False

        if x+w/2 > self.W/2 - self.Tw and x+w/2 < self.W/2 + self.Tw:
            xCentered = True
        
        if y+h/2 > self.H/2 - self.Th and y+h/2 < self.H/2 + self.Th:
            yCentered = True

        if x + w / 2 > 640 / 2:
            if y + h / 2 > 480 / 2:

                return [
                    (x,y,w,h),
                    640 / 2 - 640 + x + w / 2,
                    480 / 2 - 480 + y + h / 2,
                    "LEFT",
                    "FRONT",
                    xCentered,
                    yCentered,
                ]
            else:
                return [
                    (x,y,w,h),
                    640 / 2 - 640 + x + w / 2,
                    abs(-480 / 2 + y + h / 2),
                    "LEFT",
                    "BACK",
                    xCentered,
                    yCentered,
                ]
        else:
            if y + h / 2 > 480 / 2:
                return [
                    (x,y,w,h),
                    abs(-640 / 2 + x + w / 2),
                    480 / 2 - 480 + y + h / 2,
                    "RIGHT",
                    "FRONT",
                    xCentered,
                    yCentered,
                ]
            else:
                return [
                    (x,y,w,h),
                    abs(-640 / 2 + x + w / 2),
                    abs(-480 / 2 + y + h / 2),
                    "RIGHT",
                    "BACK",
                    xCentered,
                    yCentered,
                ]
