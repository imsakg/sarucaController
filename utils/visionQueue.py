from threading import Thread
from utils.visionType import visionType
import time

class visionQueue:
    def __init__(self,vision : list = [] ,maxSize : int = 30, timeout : float = 5.0):
        self.Q = vision
        self.maxSize = maxSize
        self.timeout = timeout

        self.gbTh=Thread(target=self.garbageCollector,daemon=True)
        self.gbTh.start()

    def __iter__(self):
        n = len(self.Q)
        while n>=0:
            n-=1
            yield self.Q[n]

    def __sizeof__(self):
        return len(self.Q)

    def qSize(self):
        return len(self.Q)

    def garbageCollector(self):
        for vis in self.Q:
            if time.time()-vis.getTime()>self.timeout:
                self.Q.remove(vis)
        time.sleep(1)
        self.garbageCollector()
    
    def lastHeartbeat(self):
        return time.time() - self.Q[-1].getTime()

    def put(self, item, overwrite : bool = False):
        if len(self.Q)<self.maxSize:
            self.Q.append(item)
            return True
        else:
            if overwrite:
                self.Q.pop(0)
                self.Q.append(item)
                return True
            else:
                raise Exception("Queue is full!")

    def clearQueue(self):
        self.Q.clear()
        return True
    
    def get(self,index = -1, overdel : bool = True):
        if len(self.Q)>0:
            item = self.Q[index]
            if overdel: self.Q.pop(index)
            return item
        else:
            raise Exception("Queue is empty!")
    
    def getAccurs(self) -> list:
        accurs = [0,0,0,0]
        n = len(self.Q)
        GAIN = 2
        for i,vis in enumerate(self.Q):
            vis = vis.getCoordinates()
            i+=1
            accurs[0] += vis[0]
            accurs[1] += vis[1]
            accurs[2] += vis[2]
            accurs[3] += vis[3]
        accurs[0] /= n
        accurs[1] /= n
        accurs[2] /= n
        accurs[3] /= n

        return accurs
    
    def getAccuracy(self):
        accuracy = 1
        accurs = self.getAccurs()
        n = len(accurs)

        for i,vis in enumerate(self.Q):
            vis = vis.getCoordinates()
            accuracy *= (accurs[0] / (vis[0]+1) + accurs[1] / (vis[1]+1) + accurs[2] / (vis[2]+1) + accurs[3] / (vis[3]+1))/ 4
        accuracy = accuracy/n
        
        return accuracy
