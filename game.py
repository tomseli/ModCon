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
pendulum.theta = 3.14
theta = pendulum.theta
cart.x = 0.0
f = 0

# Time
dt = 0

# Properties
pendulum.l = 0.1
pendulum.cf = 1
cart.m = 0.5
cart.cf = 10

cart.set_limit(0.35)

# Space
coordinates = list(MIDDLE)

# PID
ctrl_angle.kp = 2.6
ctrl_angle.ki = 10
ctrl_angle.kd = 0.01

ctrl_pos.kp = 90
ctrl_pos.ki = 0
ctrl_pos.kd = 0

ctrl_pos.enableLogging()
ctrl_angle.enableLogging()

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
    if(theta > 3.14 + 0.5 or theta < 3.14 - 0.5):
        running = False
        pass
    
    ## Simulation ##
    if((theta < 3.14 + 0.01) and (theta > 3.14 - 0.01)):
    # if(0):
        f = ctrl_pos.control(cart.x, 0, dt) - 100*ctrl_angle.control(theta, 3.14, dt)
        print("State 1")
    else:
        f = 100*-ctrl_angle.control(theta, 3.14, dt)
        ctrl_pos.control(cart.x, 0, dt)
        print("State 2")

    # ctrl_pos.control(cart.x, 0, dt) -
    cart.f = f
    x = cart.step(dt)
    theta = pendulum.step(dt)
    f = 0

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

ctrl_angle.plotLogs("Control angle", False)
ctrl_pos.plotLogs("Control pos", True)

pygame.quit()