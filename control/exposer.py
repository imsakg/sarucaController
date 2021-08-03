class Exposer:
    def __init__(self):
        return None

    def expose(self,x,y,w,h):
        if x+w/2 > 640/2:
            if y+h/2 > 480/2:
                
                return [640/2 - 640 + x+w/2, 480/2 - 480 + y+h/2, "RIGHT", "DOWN"]
            else:
                return [640/2 - 640 + x+w/2, abs(- 480/2 + y+h/2), "RIGHT", "UP"]
        else:
            if y+h/2 > 480/2:
                return [abs(-640/2 + x+w/2),  480/2 - 480 + y+h/2, "LEFT", "DOWN"]
            else:
                return [abs(-640/2 + x+w/2), abs(- 480/2 + y+h/2), "LEFT", "UP"]