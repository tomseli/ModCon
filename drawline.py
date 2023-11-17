import pygame
import numpy as np

class Line():
    def __init__(self, screen : pygame.Surface) -> None:
        self.screen = screen
        return

    def polard(self, origin : (int, int), angle : float, 
              length : float, color : (int, int, int)) -> None:
        # adjust for the weird coordinates
        rad = np.deg2rad((angle + 270) % 360)

        # find a and b lengths
        a = np.cos(rad) * length
        b = np.sin(rad) * length

        pygame.draw.line(
            self.screen, 
            color, 
            origin, 
            (origin[0] + a, origin[1] + b), 
            3)
        return
    
    # Bugged, angles do not match the correct direction
    def polarr(self, origin : (int, int), angle : float, 
        length : float, color : (int, int, int)) -> None:
        self.polard(origin, np.rad2deg(angle+np.pi), length, color)
        return