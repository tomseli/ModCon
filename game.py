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
cart.addPendulum(pendulum)
controller1 = controller.Control()
controller2 = controller.Control()

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
pendulum.m = 0.01
cart.m = 1
cart.cf = 5

cart.set_limit(0.35)

# Space
coordinates = list(MIDDLE)

# PID
controller1.kp = 0.5
controller1.ki = 0.1
controller1.kd = 0

controller2.kp = 15
controller2.ki = 0
controller2.kd = 1.5

controller1.enableLogging()
controller2.enableLogging()

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
    
    ## Simulation ##
    sp_theta = controller1.control(cart.x, 0, dt)
    f = -controller2.control(pendulum.theta, sp_theta+3.14, dt)
    
    cart.f = f
    x = cart.step(dt)
    theta = pendulum.step(dt)


    ## Rendering ## 
    screen.fill(WHITE)

    rect_x = coordinates[0]-rect_w/2+(x*1000)
    rect_y = coordinates[1]-rect_h/2
    rect = pygame.Rect((rect_x, rect_y), 
                       (rect_w, rect_h))
    pygame.draw.rect(screen, BLUE, rect)
    line.polarr((rect.center), theta, pendulum.l*1000, RED)

    pygame.display.flip()

    # Time keeping
    dt = clock.tick(120)/1000

controller2.plotLogs("Control 2", False)
controller1.plotLogs("Control 1", True)

pygame.quit()