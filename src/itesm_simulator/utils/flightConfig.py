"""
Enumerador que contiene los tipos de configuraciones de vuelo con nombre, 
en lugar de utilizar codigos
"""
from enum import Enum

class FlightConfig(Enum):
    ASYNC     = 1
    LOOPING   = 2
    HOME_INITIAL = 3