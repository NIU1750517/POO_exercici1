import random
import numpy as np
import pygame
class NBodySimulator():
    def __init__(self, windowSize):
        self._windowSize = windowSize
    def nBodySimulator(self, universe):
        pass
    def draw_bodies(color, size):
        pass
class Universe():
    def __init__(self, radius, name, num_bodies):
        self._radius = radius
        self._name = name
        self._num_bodies = num_bodies
        self._bodies = []
    def getBodyPosition(self,idxBody):
        pass
    def update(self,timeStep):
        pass
    def __computeForces(self):
        pass
    def random(cls, num_bodies):
        return cls([Body.random() for i in range(num_bodies)])
    def from_file(cls, fname):
        bodies = []
        with open(fname,'r') as f:
            num_bodies = int(f.readline())
            radius = float(f.readline())
            for _ in range(num_bodies):
                linia = f.readline()
                if linia:
                    m, px, py, vx, vy = [float(z) for z in linia.split()]
                    bodies.append(Body([px, py], [vx, vy], m))
                else:
                    raise ValueError("Empty line encountered in the file")
                
class Body():
    G = 6.674e-11
    def __init__(self, position_i, velocity_i, mass_i):
        self._position = np.array(position_i)
        self._velocity = np.array(velocity_i)
        self._mass = mass_i

#------------------------------------------------------MAIN CODE---------------------------------------------------------------------
if __name__ == '__main__':
    universe = Universe.from_file('2body.txt')
    simulator = NBodySimulator(700, universe)
