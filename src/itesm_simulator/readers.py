from enums import *
from utils.pathPlanner import genRandomCoords
from utils.visualModule import decodeFileFromPath,droneCameraQR,webCameraQR
import json

class Reader():

    def __init__(self):
        self.WAIT_TIME = 0
        self.config = FlightConfig.ASYNC

    def _defineWaitTime(self,option : int):
        if option == 0:
            self.WAIT_TIME = 1
        elif option == 1:
            self.WAIT_TIME = 2
        else:
            self.WAIT_TIME = 2
            # self.config = FlightConfig.LOOPING

    def _separateInstructions(self, l : list) -> list:
        instructions = []
        for x in l:
            s = x.strip().split(",")
            temp = []
            for y in s:
                tempFactors = y.split(";")
                if(len(tempFactors) > 1):
                    temp.append(tuple([float(z) for z in tempFactors]))
            if(len(temp) > 0):
                instructions.append(tuple(temp))
        return instructions

class ImgFileReader(Reader):

    def __init__(self):
        super().__init__()

    def readInstructions(self):
        d = json.loads(decodeFileFromPath("QR.png")[0].data.decode("UTF-8"))
        l = d["instructions"]
        super()._defineWaitTime(d["coordination"])
        return super()._separateInstructions(l)

class WebCamReader(Reader):

    def __init__(self):
        super().__init__()

    def readInstructions(self):
        d = json.loads(webCameraQR())
        l = d["instructions"]
        super()._defineWaitTime(d["coordination"])
        return super()._separateInstructions(l)

class DronCamReader(Reader):

    def __init__(self):
        super().__init__()

    def readInstructions(self):
        d = json.loads(droneCameraQR())
        l = d["instructions"]
        super()._defineWaitTime(d["coordination"])
        return super()._separateInstructions(l)

class CsvFileReader(Reader):

    def __init__(self):
        super().__init__()

    def readInstructions(self):
        l = []
        super()._defineWaitTime(2)
        with open("visualize.csv","r") as f:
            l = f.readlines()
            f.close()
        return super()._separateInstructions(l)

class RandomPositionReader(Reader):

    def __init__(self):
        super().__init__()

    def readInstructions(self):
        l = []
        super()._defineWaitTime(2)
        coordLen = 5
        replicable = input("Replicable number? (y) ") == 'y'
        return [tuple([coord]) for coord in genRandomCoords(coordLen, replicable)]