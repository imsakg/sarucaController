import time

class visionType(object):
    def __init__(self,coordinates : list = [], time : float = time.time()):
        self.coordinates = coordinates
        self.time = time

    def __repr__(self):
        return f"Coordinates: {self.coordinates}\nTime : {self.time}"

    def __setitem__(self, index, data):
        if index == 0:
            self.coordinates = data
        elif index == 1:
            self.time = data
        else:
            raise Exception(f"Out Of Index {index}")

    def __iter__(self):
        yield self.coordinates
        yield self.time

    def __getitem__(self,index):
        if index == 0:
            return self.coordinates
        elif index == 1:
            return self.time
        else:
            raise Exception(f"Out Of Index {index}")
    
    @staticmethod
    def x(self):
        return self.coordinates[0]

    @staticmethod
    def y(self):
        return self.coordinates[1]

    @staticmethod
    def width(self):
        return self.coordinates[2]

    @staticmethod
    def heigh(self):
        return self.coordinates[3]

    def getTime(self):
        return self.time

    def getCoordinates(self):
        return self.coordinates