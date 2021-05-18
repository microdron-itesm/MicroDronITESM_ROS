"""
Discrete fourier transform.
Commented code uses complex numbers explicitly, whereas current implementation is
using simple approach by sorting out by real and imaginaries.
"""

import math
# from cmath import phase
import numpy as np

class DFT():
    # Parse a signal named x
    # Variable are named after the nomenclature on the wikipedia article
    # https://en.wikipedia.org/wiki/Discrete_Fourier_transform
    def __init__(self, x):
        # Array that holds the final transform
        self.X = []
        # We don't have infinity and the loops must end somewhere
        # Maybe we should end it where the length of the signal ends
        self.N = len(x)
        for k in range(self.N):
            self.re = 0
            self.im = 0
            # self.partial = 0
            for n in range(self.N):
                # basically find the coefficients of the complex numbers that holds the 
                # amplitude for any specific frequencies
                argument = (2*math.pi*k*n)/self.N
                # self.partial += x[n]*complex(math.cos(argument), -math.sin(argument))
                self.re += x[n]*math.cos(argument)
                self.im -= x[n]*math.sin(argument)
            # It is used to attenuate amplitudes by the overall length of the series
            # self.partial = self.partial/self.N
            self.re = self.re/self.N
            self.im = self.im/self.N
            # d = {
            #     "amp": abs(self.partial),
            #     "phase": phase(self.partial),
            #     "freq": k,
            #     "re": self.partial.real,
            #     "im": self.partial.imag
            # }
            d = {
                "amp": math.sqrt(self.re**2 + self.im**2),
                "phase": math.atan2(self.im, self.re),
                "freq": k,
                "re": self.re,
                "im": self.im
            }
            self.X.append(d)
            

if __name__ == '__main__':
    signal = []
    for i in range(100):
        if i >= 50:
            signal.append(10)
        else:
            signal.append(-10)
    DFT(signal)