"""
Este programa contiene operaciones utiles que ayudan con operaciones a lo largo de todo el programa
"""
from math import sqrt
from time import time

def roundTo0(f : float) -> float:
    return 0 if abs(f) < .0001 else f

def round2f(f : float) -> float:
    return float("{:.2f}".format(f))

def euclidianDistance(x1,x2,y1,y2,z1,z2) -> float:
    return sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

def measureTime(fun):
    def measureExTime(*args):
        init = time()
        result = fun(*args)
        finit = time()
        print(f"Execution Time = {finit - init} seconds")
        return result
    return measureExTime