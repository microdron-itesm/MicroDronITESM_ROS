import argparse
import math
import numpy as np
from collections import deque

def roundTo0(f : float) -> float:
    return 0 if abs(f) < .0001 else f

def heartEquation(x : float, scale : int,neg = False) -> float:
    result = math.sqrt(scale - math.pow(x,2))
    if(neg):
        result = -result
    temp = abs(x) ** (2/3)
    return result + temp


def makeHeart(size : int, precision : int) -> list:
    limit = int(math.sqrt(size))
    size = limit ** 2
    xAxis = []
    yAxis = []
    steps = 1/precision
    i = -limit
    while(i<limit):
        xAxis.append(i)
        yAxis.append(heartEquation(i,size))
        i += steps
    i = limit
    while(i>-limit):
        xAxis.append(i)
        yAxis.append(heartEquation(i,size,True))
        i -= steps
    return [
        xAxis,
        yAxis
    ]

def makeSquare(size : int,stepSize : int) -> list:
    result = [
        [], # for the x axis
        []  # for the y axis
    ]
    half = size//2
    y = half
    for x in range(-half,half,stepSize):
        result[0].append(x)
        result[1].append(y)
    x = half
    for y in range(-half,half,stepSize):
        result[0].append(x)
        result[1].append(-y)
    y = -half
    for x in range(-half,half,stepSize):
        result[0].append(-x)
        result[1].append(y)
    x = -half
    for y in range(-half,half,stepSize):
        result[0].append(x)
        result[1].append(y)
    return result

def makeCircle(radius : float,stepSize : int) -> list:
    result = [
        [], # values for the x axis
        []  # values for the Y axis
    ]
    s = (math.pi/2)/stepSize
    i = 0
    while i < math.pi*2:
        ##some code
        result[0].append(roundTo0(math.sin(i)*radius)) # values for the x axis
        result[1].append(roundTo0(math.cos(i)*radius)) # values for the Y axis
        i += s
    return result

def makeTriangle(size : int, stepSize : int) -> list:
    result = [
        [], # values for the x axis
        []  # values for the Y axis
    ]
    tan = math.tan(math.pi/3)
    halfSize = size//2
    for x in range(0,halfSize,stepSize):
        result[0].append(x)
        result[1].append(x*tan)
    result[0].append(halfSize)
    result[1].append(halfSize*tan)
    maxHeight = result[1][-1]
    for x in range(stepSize,halfSize,stepSize):
        result[0].append(halfSize + x)
        result[1].append(maxHeight - x*tan)
    for x in range(size,0,-stepSize):
        result[0].append(x)
        result[1].append(0)
    return [
        list(np.array(result[0]) - halfSize),
        list(np.array(result[1]) - halfSize)
    ]

def merge(l : list) -> list:
    boolean = len(l[0]) != len(l[1])
    if(boolean):
        raise IndexError("The arrays are wrong size")
    return [f"{l[0][x]};{l[1][x]}" for x in range(len(l[0]))]

def makeSpecialHeart(size,step) -> list:
    coords = [
        [], # x
        []  # y
    ]
    

def stringFormat(l : list,drones : int) -> str:
    s = ""
    partsLength = len(l)//drones
    newL = deque(l)
    for padding in range(drones):
        for x in newL:
            s += x + ","
        if(padding < drones-1):
            s += "\n"
        newL.rotate(partsLength)
        # print(newL[0])
    return s
    
parser = argparse.ArgumentParser(description='Print the points for the drone/drones to follow')
parser.add_argument('size', metavar='s', type=float,
                    help="The size of the given figure to create")
parser.add_argument('stepSize', metavar='step', type=int,
                    help='The number of divisions to give the figure')
parser.add_argument('--drones', metavar='drones', type=int,
                    default=1, help='The number of sets to divide the points in due to the number of drones to handle, defaults to 1')
parser.add_argument('--figure', dest='figure', type=str,
                    help='options are >> cirlce square triangle heart')

if __name__ == "__main__":
    args = parser.parse_args()
    resultStr = ""
    if(args.figure == "circle"):
        radius = args.size
        stepSize = args.stepSize
        resultStr = stringFormat(merge(makeCircle(radius,stepSize)),args.drones)
    elif(args.figure == "square"):
        size = int(args.size)
        stepSize = args.stepSize
        resultStr = stringFormat(merge(makeSquare(size,stepSize)),args.drones)
    elif(args.figure == "triangle"):
        size = int(args.size)
        stepSize = args.stepSize
        resultStr = stringFormat(merge(makeTriangle(size,stepSize)),args.drones)
    elif(args.figure == "heart"):
        size = int(args.size)
        stepSize = args.stepSize
        resultStr = stringFormat(merge(makeHeart(size,stepSize)),args.drones)
    elif(args.figure == "special"):
        size = 9
        stepSize = 8
        resultStr = makeSpecialHeart(size,stepSize)
    print(resultStr)