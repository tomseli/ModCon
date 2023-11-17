import pygame
import drawline
import numpy as np

pygame.init()

HEIGHT = 500
WIDTH = 1000
MIDDLE = (WIDTH/2, HEIGHT/2)

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

screen = pygame.display.set_mode([WIDTH, HEIGHT])
clock = pygame.time.Clock()
line = drawline.Line(screen)

## Sim variables ##
theta = 3.14
theta_d = 0
theta_dd = 0

theta_prev = theta
theta_d_prev = 0
theta_dd_prev = 0

dt = 0
g = 9.81
l = 0.2

offset = 0
coordinates = list(MIDDLE)

rect_w = 50
rect_h = 25

x = 0
x_d = 0
x_dd = 0
x_prev = 0
x_d_prev = 0
x_dd_prev = 0
f = 0
m = 1
kw = 2

running = True
while running:
    ## Inputs ##
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                f = -25
            if event.key == pygame.K_RIGHT:
                f = 25
    
    ## Simulation ##
    # Rectangle
    x_dd = f - kw * x_d_prev
    x_d = x_d_prev + x_dd_prev * dt
    x = x_prev + x_d_prev * dt

    x_dd_prev = x_dd
    x_d_prev = x_d
    x_prev = x

    print(x)

    # Pendulum
    term1 = -x_dd * np.cos(theta_prev)
    term2 = x_d * np.sin(theta_prev)
    term3 = -x_d * theta_d_prev * np.sin(theta_prev)
    term4 = g * np.sin(theta_prev)
    term5 = 0.5 * theta_d_prev
    numerator = term1 + term2 + term3 + term4 + term5
    denominator = -l

    theta_dd = numerator/denominator
    theta_d = theta_d_prev + theta_dd * dt
    theta = theta_prev + theta_d * dt

    theta_dd_prev = theta_dd
    theta_d_prev = theta_d
    theta_prev = theta

    f = 0
    ## Rendering ## 
    screen.fill(WHITE)

    rect_x = coordinates[0]-rect_w/2+(x*1000)
    rect_y = coordinates[1]-rect_h/2
    rect = pygame.Rect((rect_x, rect_y), 
                       (rect_w, rect_h))
    pygame.draw.rect(screen, BLUE, rect)
    line.polarr((rect.center), theta, l*1000, RED)

    # Simular to update, but faster?
    pygame.display.flip()

    dt = clock.tick(60)/1000

pygame.quit()