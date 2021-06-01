from random import randint, seed, shuffle
from .utils import round2f, euclidianDistance

def genRandomCoords(coordLen, replicable = False):
    if replicable :
        seed(91939)
    floatFactor = .1
    return [[
        -round2f(randint(0,25)*floatFactor) if randint(0,100) % 2 == 0 else round2f(randint(0,25)*floatFactor),
        -round2f(randint(0,25)*floatFactor) if randint(0,100) % 2 == 0 else round2f(randint(0,25)*floatFactor),
        round2f(randint(0,25)*floatFactor) + 1
    ] for x in range(coordLen)]

def calculate_distances(position, otherPositions):
    return [round2f(euclidianDistance(
        position[0], nextPosition[0],
        position[1], nextPosition[1],
        position[2], nextPosition[2]
    )) for nextPosition in otherPositions]

def convertSequenceAsString(seq:list) -> str:
    result = ""
    for val in seq:
        result += str(val)+','
    return result[:-1]

def calculatePathDistance(seq : list, distances: list) -> float:
    return sum([distances[point][seq[point]] for point in range(len(seq))])

def aStar(curr : list, distances : list, memory : dict):
    temporalKey = convertSequenceAsString(curr)
    if len(curr) == len(distances):
        if(temporalKey not in memory):
            memory[temporalKey] = calculatePathDistance(curr,distances)
        return memory[temporalKey]
    if(temporalKey not in memory):
        memory[temporalKey] = calculatePathDistance(curr,distances)
    currentDistance = memory[temporalKey]
    allDistances = []
    for position in range(len(distances)):
        if position not in curr:
            temp = list(curr)
            temp.append(position)
            allDistances.append(aStar(temp, distances, memory))
    return min(allDistances)

def to_int_arr(arr) -> list:
    return [int(x) for x in arr]

def path_plan(current_positions, next_positions, verbose = False):
    distances = [calculate_distances(currP,next_positions) for currP in current_positions]
    if verbose :
        print("\n========================= MAPPING DISTANCES =========================\n")
        for position in range(len(current_positions)):
            print(f"For position {current_positions[position]}\t distances are\t {distances[position]}")
        print("\n========================= MAPPING RESULTS =========================\n")
    hashMemory = {}
    result = aStar([], distances, hashMemory)
    totalPathCombinations = {}
    for key in hashMemory.keys():
        partsOfKey = key.split(',')
        if(len(partsOfKey) == len(distances)):
            if verbose:
                print(f"Key {key}\tequals a distance of\t{hashMemory[key]}")
            totalPathCombinations[hashMemory[key]] = partsOfKey

    return to_int_arr(totalPathCombinations[result]), result

def factorial(n):
    if n < 2:
        return 1
    return n*factorial(n-1)

if __name__ == "__main__":
    from utils import measureTime
    from sys import argv
    nextPositions = [
        [0.0 ,  1.6,    2.5],
        [0.0 ,  0.8,    1.5],
        [0   ,  0,      2.5],
        [-0.0,  -0.8,   1.5],
        [-0.0,  -1.6,   2.5],
        # [0.0 ,  1.6,    2.0],
        # [0.0 ,  0.8,    1.0],
        # [0   ,  0,      2.0]
    ]
    meassured = measureTime(path_plan)
    v = argv[-1] == "-v"
    combination, minDistance = meassured(genRandomCoords(len(nextPositions), True), nextPositions, v)
    print(f"The best combination being {combination}\ttraveling a distance of {minDistance}")
    # print(factorial(100))