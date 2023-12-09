# Libraries
import pygame
import numpy as np
import math

# Modules
from modules import drawline
from modules import physics
from modules import controller

## Defines ##
# Simulation defines
CART_SPACE = 0.35

# Rendering defines
HEIGHT = 500
WIDTH = 1000
MIDDLE = (WIDTH/2, HEIGHT/2)
SCALE = 500

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

## Game Objects ##
pygame.init()
screen = pygame.display.set_mode([WIDTH, HEIGHT])
clock = pygame.time.Clock()
line = drawline.Line(screen)
line2 = drawline.Line(screen)

font = pygame.font.Font('freesansbold.ttf', 32)
text_enabled = font.render('Control Enabled', True, GREEN)
text_disabled = font.render('Control Disabled', True, RED)

## Physics ##
pendulum = physics.PhysicsDoublePendulumCart()
control = controller.ControlLQR(pendulum)

## Sim variables ##
# I/O
enable_control = True
disturb = 0

# Time
dt = 0

# Properties
pendulum.l1 = 0.2
pendulum.m1 = 0.1
# pendulum.cf = pendulum.m1/20.1428571428571428571428571428571


pendulum.l2  = 0.2
pendulum.m2  = 0.1
# pendulum.cf2 = pendulum.m2/20.1428571428571428571428571428571
pendulum.M = 2

f = 0

pendulum.set_limit(CART_SPACE)
pendulum.init()

# Space
coordinates = list(MIDDLE)

# Control
control.M = pendulum.M
control.m1 = pendulum.m1
control.m2 = pendulum.m2
control.l1 = pendulum.l1
control.l2 = pendulum.l2
control.init()

## Rendering variables ##
rect_w = 50
rect_h = 25

running = True
while running:
    ## Inputs ##
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                disturb = -10
            if event.key == pygame.K_RIGHT:
                disturb = 10
            if event.key == pygame.K_DOWN:
                enable_control = False
    
    ## Simulation ##
    # Controller
    if(enable_control):
# =============================================================================
#         if(theta > 3.14 + 0.5 or theta < 3.14 - 0.5):
#             running = False
# =============================================================================
        # f = control.control() + disturb
        f = 0 + disturb
    else:
        f = 0 + disturb

    U = np.array(
        [[0+disturb]]
    )
    disturb = 0

    Y = pendulum.step(U, dt)
    x = Y[0][0]
    theta1 = Y[1][0]
    theta2 = Y[2][0]

    ## Rendering ## 
    # Background
    screen.fill(WHITE)

    # Additional visuals
    pygame.draw.circle(screen, (0, 0, 0), 
                       (WIDTH/2+(CART_SPACE*SCALE)+rect_h, HEIGHT/2), 5)
    pygame.draw.circle(screen, (0, 0, 0), 
                       (WIDTH/2-(CART_SPACE*SCALE)-rect_h, HEIGHT/2), 5)
    pygame.draw.line(
        screen, 
        (10, 10, 10), 
        (WIDTH/2+(CART_SPACE*SCALE)+rect_h, HEIGHT/2), 
        (WIDTH/2-(CART_SPACE*SCALE)-rect_h, HEIGHT/2)
        )
    if(enable_control):
        screen.blit(text_enabled, (20, 20))
    else:
        screen.blit(text_disabled, (20, 20))

    # Cart + Pendulum
    rect_x = coordinates[0]-rect_w/2+(x*SCALE)
    rect_y = coordinates[1]-rect_h/2
    rect = pygame.Rect((rect_x, rect_y), 
                       (rect_w, rect_h))
    pygame.draw.rect(screen, BLUE, rect)
    line.polarr((rect.center), theta1, pendulum.l1*SCALE, RED)
    line2.polarr(
        (rect.center[0] + pendulum.l1*SCALE*np.sin(theta1), 
         rect.center[1] - pendulum.l1*SCALE*np.cos(theta1)), 
         theta2, pendulum.l2*SCALE, BLUE)

    # Refresh
    pygame.display.flip()

    # Time keeping
    dt = clock.tick(480)/1000

pygame.quit()