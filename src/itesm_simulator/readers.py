"""
Este programa sirve como la interfaz de comunicacion entre los diferentes tipos de archivo que 
se pueden leer como son .json, .csv, .png, etc.

En caso de que se desee agregar una nueva interfaz de lectura, se debera agregar una nueva sublcase de Reader.

Escrito por Jesús Omar Cuenca Espino
"""

from utils.pathPlanner import genRandomCoords
from utils.visualModule import decodeFileFromPath,droneCameraQR,webCameraQR
from utils.flightConfig import FlightConfig
import json

class Reader():

    def __init__(self, enum = FlightConfig.ASYNC, inst = []):
        self.WAIT_TIME = 0
        self.rawInstructions = inst
        self._defineWaitTime(enum)

    def _defineWaitTime(self, enum):
        self.config = enum
        if enum == FlightConfig.ASYNC:
            self.WAIT_TIME = 1
        else:
            self.WAIT_TIME = 2

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

    def readInstructions(self):
        return self._separateInstructions(self.rawInstructions)

class ImgFileReader(Reader):

    def __init__(self):
        d = json.loads(decodeFileFromPath("QR.png")[0].data.decode("UTF-8"))
        super().__init__(d["coordination"], d["instructions"])

class WebCamReader(Reader):

    def __init__(self):
        d = json.loads(webCameraQR())
        super().__init__(d["coordination"], d["instructions"])

class DronCamReader(Reader):

    def __init__(self):
        d = json.loads(droneCameraQR())
        super().__init__(d["coordination"], d["instructions"])

class CsvFileReader(Reader):

    def __init__(self):
        l = []
        with open("visualize.csv","r") as f:
            l = f.readlines()
            f.close()
        super().__init__(FlightConfig.ASYNC, l)

class SpecificCsvFileReader(Reader):

    def __init__(self, name):
        l = []
        with open(name,"r") as f:
            l = f.readlines()
            f.close()
        super().__init__(FlightConfig.ASYNC, l)

class JSONReader(Reader):

    def __init__(self):
        d = {}
        with open("special.json","r") as f:
            temp = f.read().strip()
            d = json.loads(temp)
            f.close()
        
        l = SpecificCsvFileReader(d["fileName"]).readInstructions()

        super().__init__(FlightConfig.ASYNC, l)
    
    def readInstructions(self):
        return self.rawInstructions

class RandomPositionReader(Reader):

    def __init__(self):
        coordLen = 9
        replicable = input("Replicable number? (y) ") == 'y'
        super().__init__(FlightConfig.ASYNC, [tuple([coord]) for coord in genRandomCoords(coordLen)])

    def readInstructions(self):
        return self.rawInstructions