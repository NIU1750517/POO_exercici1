import random
import numpy as np
import pygame

class NBodySimulator():
    pass

class Universe():
    def __init__(self, bodies):
        self.bodies = bodies

    @classmethod
    def random(cls, num_bodies):
        pass

    @classmethod
    def from_file(cls, fname):
        pass

class Body():
    G=6.67408e-11
    def __init__(self, mass, position, velocity):
        self._mass = mass
        self._position = position
        self._velocity = velocity
    
    
#------------------------------------------------------MAIN CODE---------------------------------------------------------------------
if __name__ == '__main__':
    pass
