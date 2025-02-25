import random
import numpy as np
import pygame

class NBodySimulator():
    def __init__(self, windowSize, universe):
        self._windowSize = windowSize
        self.universe = universe
        self.space_radius = self.universe.radius
        self.factor = self._windowSize / 2 / self.space_radius

    def _draw(self, position_space, color, size=5.):
        position_pixels = self.factor * position_space + self._windowSize / 2.
        pygame.draw.circle(self.screen, color, position_pixels, size)
    
    def animate(self, time_step, trace=False):
        pygame.init()
        self.screen = pygame.display.set_mode([self._windowSize, self._windowSize])
        pygame.display.set_caption(f'N-Body Simulation  |  timestep = {time_step}  |  {len(self.universe.bodies)} Body' )
        running = True
        color_background, color_body, color_trace = (128, 128, 128), (0, 0, 0), (192, 192, 192)
        self.screen.fill(color_background)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            if trace:
                for body in self.universe.bodies:
                    self._draw(body._position, color_trace)
                self._update(time_step)
                for body in self.universe.bodies:
                    self._draw(body._position, color_body)
                pygame.display.flip()
            else:
                self.screen.fill(color_background)
                self._update(time_step)
                for body in self.universe.bodies:
                    self._draw(body._position, color_body)
                pygame.display.flip()
        pygame.quit()

    def _update(self, time_step):
        for body in self.universe.bodies:
            total_force = body.total_force(self.universe.bodies)
            body.update(total_force, time_step)

class Universe():
    def __init__(self, bodies, radius=1e12):
        self.bodies = bodies
        self.radius = radius
        
    @classmethod
    def random(cls, num_bodies):
        bodies=[Body([0,0],[0,0],1e32)]
        for i in range(num_bodies-1):
            bodies.append(Body.random())
        return cls(bodies)
    
    @classmethod
    def from_file(cls, fname):
        bodies = []
        with open(fname,'r') as f:
            num_bodies = int(f.readline())
            radius = float(f.readline())
            for _ in range(num_bodies):
                linia = f.readline()
                px, py, vx, vy, m = [float(z) for z in linia.strip().split() if z]
                bodies.append(Body([px, py], [vx, vy], m))
        print(f"Universe imported successfully {len(bodies)} Bodies !!!")
        return cls(bodies, radius)
    @classmethod
    def configured_interactive(cls):
        bodies = []
        num_bodies = int(input("¿Cuántos cuerpos querés agregar? "))
        for i in range(num_bodies):
            print(f"\nCuerpo {i+1}:")
            x = float(input("Posición X (m): "))
            y = float(input("Posición Y (m): "))
            vx = float(input("Velocidad X (m/s): "))
            vy = float(input("Velocidad Y (m/s): "))
            mass = float(input("Masa (kg): "))

            body = Body(position=[x, y], velocity=[vx, vy], mass=mass)
            bodies.append(body)

        return cls(bodies)


class Body():
    G = 6.67408e-11
    def __init__(self, position, velocity, mass):
        self._mass = mass
        self._position = np.array(position, dtype=np.float64)  
        self._velocity = np.array(velocity, dtype=np.float64)   


    @property
    def mass(self):
        return self._mass
    @property
    def position(self):
        return self._position
    @property
    def velocity(self):
        return self._velocity

    def _force(self, another_body):
        # Calcula la força gravitatòria exercida per un altre cos.
        distance12 = self._distance_to(another_body)
        magnitude = self._mass * another_body._mass * Body.G / distance12**2
        direction = (another_body._position - self._position) / distance12
        return magnitude * direction
    
    def _distance(self, p, q):
        return np.sqrt(np.square(p - q).sum())
    
    def _distance_to(self, another_body):
        return self._distance(self._position, another_body._position)
    
    def update(self, force, dt):
        if self._mass == 0:
            raise ValueError("La Massa NO pot ser zero")
        acceleration = force / self._mass
        self._velocity += acceleration * dt
        self._position += self._velocity * dt

    def total_force(self, other_bodies):
        force = np.zeros(2)
        for body in other_bodies:
            if body is not self:
                force += self._force(body)
        return force

    def move(self, other_bodies, dt): 
        # Actualitza la velocitat i posició del cos en funció de la força i el pas de temps.
        total_force = self.total_force(other_bodies)
        self.update(total_force, dt)
    
    @staticmethod
    def random_vector(a, b, dim=2):
        return np.random.uniform(a, b, dim)
    
    @classmethod
    def random(cls, universe_radius=1e12): 
        mass = np.random.uniform(1e23, 1e29)
        position = Body.random_vector(-universe_radius, universe_radius, 2)
        velocity = Body.random_vector(-1e5, 1e5, 2)
        
        return cls(position, velocity, mass)
    

#------------------------------------------------------MAIN CODE---------------------------------------------------------------------
if __name__ == '__main__':
    #universe = Universe.random(10)
    #universe = Universe.from_file('2body.txt')
    universe = Universe.configured_interactive()
    for body in universe.bodies:
        print(f"Body: {body._position} x, {body._velocity} v, {body._mass} m")
    simulator = NBodySimulator(800, universe)
    time_step = 5000
    simulator.animate(time_step, trace=True)