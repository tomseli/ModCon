# libraries
import pygame
import numpy as np

# modules
import drawline
import physics
import controller

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

## Physics ##
cart = physics.PhysicsCart()
pendulum = physics.PhysicsCartPendulum(cart)
ctrl_angle = controller.Control()
ctrl_pos = controller.Control()

## Sim variables ##
# I/O
pendulum.theta = 3.05
theta = pendulum.theta
cart.x = 0.1
f = 0

# Time
dt = 0

# Properties
pendulum.l = 0.2
pendulum.cf = 1
cart.m = 0.5
cart.cf = 5

cart.set_limit(0.35)

# Space
coordinates = list(MIDDLE)

# PID
ctrl_angle.kp = 150
ctrl_angle.ki = 4
ctrl_angle.kd = 2

ctrl_pos.kp = 20
ctrl_pos.ki = 0
ctrl_pos.kd = 0

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
                f = -50
            if event.key == pygame.K_RIGHT:
                f = 50
    
    ## Simulation ##
    f = -ctrl_angle.control(theta, 3.14, dt) + ctrl_pos.control(cart.x, 0, dt)
    # ctrl_pos.control(cart.x, 0, dt) -
    cart.f = f
    x = cart.step(dt)
    theta = pendulum.step(dt)
    f = 0

    ctrl_angle.print()
    ## Rendering ## 
    screen.fill(WHITE)

    rect_x = coordinates[0]-rect_w/2+(x*1000)
    rect_y = coordinates[1]-rect_h/2
    rect = pygame.Rect((rect_x, rect_y), 
                       (rect_w, rect_h))
    pygame.draw.rect(screen, BLUE, rect)
    line.polarr((rect.center), theta, pendulum.l*1000, RED)

    # Simular to update, but faster?
    pygame.display.flip()

    dt = clock.tick(120)/1000

pygame.quit()