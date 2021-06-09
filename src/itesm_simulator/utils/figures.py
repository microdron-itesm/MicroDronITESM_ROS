"""
Programa de apoyo para crear figuras 3D que los drones puedan navegar
"""
import argparse
import math
import numpy as np
from collections import deque
from utils import *

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
        yAxis.append(round2f(heartEquation(i,size)))
        i += steps
    i = limit
    while(i>-limit):
        xAxis.append(i)
        yAxis.append(round2f(heartEquation(i,size,True)))
        i -= steps
    return [
        xAxis,
        yAxis
    ]

def makeSquare(size : int,stepSize : int) -> list:
    result = [
        [], # for the x axis
        [],  # for the y axis
        []
    ]
    half = size//2
    y = half
    for x in range(-half,half,stepSize):
        result[0].append(x)
        result[1].append(y)
        result[2].append(2)
    x = half
    for y in range(-half,half,stepSize):
        result[0].append(x)
        result[1].append(-y)
        result[2].append(2)
    y = -half
    for x in range(-half,half,stepSize):
        result[0].append(-x)
        result[1].append(y)
        result[2].append(2)
    x = -half
    for y in range(-half,half,stepSize):
        result[0].append(x)
        result[1].append(y)
        result[2].append(2)
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
        result[0].append(round2f(roundTo0(math.sin(i)*radius))) # values for the x axis
        result[1].append(round2f(roundTo0(math.cos(i)*radius))) # values for the Y axis
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
    d3 = len(l) == 3
    if not d3:
        boolean = len(l[0]) != len(l[1])
        if(boolean):
            raise IndexError("The arrays are wrong size")
        return [f"{l[0][x]};{l[1][x]}" for x in range(len(l[0]))]
    else:
        bA = len(l[0]) == len(l[1])
        bB =  len(l[2]) == len(l[1])
        if(not (bA and bB)):
            raise IndexError("The arrays are wrong size")
        return [f"{l[0][x]};{l[1][x]};{l[2][x]}" for x in range(len(l[0]))]

def halfHeart(size, precision, left = True):
    coords = [
        [], # x
        []  # y
    ]
    limit = int(math.sqrt(size))
    size = limit ** 2
    steps = 1/precision
    i = 0
    while(abs(i) < limit):
        coords[0].append(i)
        coords[1].append(heartEquation(i,size))
        i += steps if not left else -steps
    while(abs(i) > 0):
        coords[0].append(i)
        coords[1].append(heartEquation(i,size,True))
        i -= steps if not left else -steps
    return coords

def makeSpecialHeart(size,precision) -> str:
    coords = halfHeart(size,precision)
    t = [f"{coords[0][x]};{coords[1][x]}" for x in range(len(coords[0]))]
    s = ""
    for x in t:
        s += x + ','
    s += '\n'
    coords = halfHeart(size,precision,False)
    t = [f"{coords[0][x]};{coords[1][x]}" for x in range(len(coords[0]))]
    for x in t:
        s += x + ','
    return s
    
def makeCube(size) -> list:
    xAxis = [0,1,1,0,0,1,1,0,0,0,1,1,1,1,0,0]
    yAxis = [0,0,1,1,1,1,0,0,0,1,1,1,0,0,1,0]
    zAxis = [0,0,0,0,1,1,1,1,0,0,0,1,0,1,1,1]
    return [
        list((np.array(xAxis))      * size),
        list((np.array(yAxis))      * size),
        list((np.array(zAxis)+.5)   * size)
    ]

def makeSecret() -> list:
    xAxis = [0,1,1.5,2,2.5,3,3,3.5,4,4,4.5,4.5,5.5,0]
    yAxis = [0,0,-1,1,-1,-1,1,0,1,-1,-1,-1.5,3,0]
    return [
        xAxis,
        yAxis
    ]

def cylinderHelper(arr : list, deltaVal : float, deltaRate : float, minVal : float, maxVal : float):
    arr1 = []
    arr2 = []

    delta           = deltaRate
    currentZAxis    = deltaVal

    length = len(arr[0])

    for point in range(length):
        arr1.append([ arr[0][point] , arr[1][point] ,round2f(currentZAxis)])
        arr2.append([-arr[0][point] ,-arr[1][point] ,round2f(currentZAxis)])
        if(currentZAxis >= maxVal or currentZAxis <= minVal):
            delta = -delta
        currentZAxis += delta

    return arr1, arr2

def toString(arr : list) -> str:
    result = ""
    first = True

    for point in arr:
        if first:
            first = False
        else:
            result += ","
        result += f"{point[0]};{point[1]};{point[2]}"

    return result

def makeHeartWitHeight(size : int, precision : int, height : int) -> list:
    limit = int(math.sqrt(size))
    size = limit ** 2
    xAxis = []
    yAxis = []
    steps = 1/precision
    i = -limit
    while(i<limit):
        xAxis.append(i)
        yAxis.append(round2f(heartEquation(i,size)))
        i += steps
    i = limit
    while(i>-limit):
        xAxis.append(i)
        yAxis.append(round2f(heartEquation(i,size,True)))
        i -= steps
    return [
        xAxis,
        yAxis,
        [height] * len(xAxis)
    ]

def makeSpecialCylinder() -> str:
    PRECISION = 4
    points = [
        [],
        [],
        [], # center
        [],
        []
    ]

    Z_AXIS_MIN = 1
    Z_AXIS_MAX = 3
    delta = .5
    currentZAxis = Z_AXIS_MIN + delta

    # Circle
    RADIUS = .8
    innerFigure = makeCircle(RADIUS,PRECISION)
    points[1], points[3] = cylinderHelper(innerFigure, Z_AXIS_MIN + delta, delta ,Z_AXIS_MIN ,Z_AXIS_MAX)
    # Rectangle 
    innerFigure = makeSquare(round(RADIUS) + 2, 5)
    temp1, temp2 = cylinderHelper(innerFigure, points[1][-1][-1], 0 ,Z_AXIS_MIN ,Z_AXIS_MAX)
    points[1].extend(temp1)
    points[3].extend(temp2)

    delta = -0.5
    
    # Circle
    RADIUS = 1.6
    outerFigure = makeCircle(RADIUS,PRECISION)
    points[0], points[4] = cylinderHelper(outerFigure, Z_AXIS_MAX + delta, delta ,Z_AXIS_MIN ,Z_AXIS_MAX)
    # Rectangle
    outerFigure = makeSquare(round(RADIUS) + 2, 5)
    temp1, temp2 = cylinderHelper(outerFigure, points[0][-1][-1], 0 ,Z_AXIS_MIN ,Z_AXIS_MAX)
    points[0].extend(temp1)
    points[4].extend(temp2)

    # Rectangles but exchanging heights
    temp1, temp2 = cylinderHelper(innerFigure, points[0][-1][-1], 0 ,Z_AXIS_MIN ,Z_AXIS_MAX)
    points[1].extend(temp1)
    points[3].extend(temp2)

    temp1, temp2 = cylinderHelper(outerFigure, points[1][-1][-1], 0 ,Z_AXIS_MIN ,Z_AXIS_MAX)
    points[0].extend(temp1)
    points[4].extend(temp2)

    # Circle
    # RADIUS = .8
    # innerFigure = makeSquare(round(RADIUS)+2, 5)
    # points[1], points[3] = cylinderHelper(innerFigure, Z_AXIS_MIN + delta, 0 ,Z_AXIS_MIN ,Z_AXIS_MAX)

    # delta = -0.5
    
    # # Circle
    # RADIUS = 1.6
    # outerFigure = makeSquare(round(RADIUS)+2, 5)
    # points[0], points[4] = cylinderHelper(outerFigure, Z_AXIS_MAX + delta, 0 ,Z_AXIS_MIN ,Z_AXIS_MAX)

    points[2] = [[0,0,zValue[2]] for zValue in points[0]]

    # Bottomless Triangle

    points[0].append([-2,-2,points[0][-1][-1]])
    points[1].append([-1,-1,points[1][-1][-1] + 1])
    points[2].append([0,0,points[2][-1][-1] + 2])
    points[3].append([1,1,points[3][-1][-1] + 1])
    points[4].append([2,2,points[4][-1][-1]])

    pointsStrings = [toString(droneInstructions) for droneInstructions in points]

    result = ""
    first = True

    for s in pointsStrings:
        if first :
            first = False
        else:
            result += "\n"
        result += s
    
    return result

def stringFormat(l : list,drones : int) -> str:
    s = ""
    partsLength = len(l)//drones
    newL = deque(l)
    for padding in range(drones):
        for x in newL:
            s += x + ","
        # if(padding < drones-1):
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
                    help='options are >> cirlce square triangle heart specialCilinder')
parser.add_argument('--height', dest='height', type=int,
                    default=1, help='options are >> cirlce square triangle heart specialCilinder')

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
    elif(args.figure == "heartH"):
        size = int(args.size)
        stepSize = args.stepSize
        resultStr = stringFormat(merge(makeHeartWitHeight(size, stepSize, args.height)),args.drones)
    elif(args.figure == "special"):
        size = 9
        stepSize = 4
        resultStr = makeSpecialHeart(size,stepSize)
    elif(args.figure == "all"):
        # print("Cirlce")
        resultStr += stringFormat(merge(makeCircle(2,8)),2)
        # print("Square")
        resultStr += stringFormat(merge(makeSquare(4,1)),2)
        # print("Triangle")
        resultStr += stringFormat(merge(makeTriangle(7,1)),2)
        # print("Corazon")
        resultStr += stringFormat(merge(makeHeart(9,4)),2)
        # print("Special Heart")
        resultStr += makeSpecialHeart(9,4)
        temp = resultStr.rstrip().split('\n')
        resultStr = ""
        for x in range(0,len(temp),2):
            resultStr += temp[x]
        resultStr += '\n'
        for x in range(1,len(temp),2):
            resultStr += temp[x]
        resultStr += '\n'
    elif(args.figure == "cube"):
        size = int(args.size)
        resultStr = stringFormat(merge(makeCube(size)),args.drones)
    elif(args.figure == "specialCilinder"):
        resultStr = makeSpecialCylinder()

    print(resultStr)