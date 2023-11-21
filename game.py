# Libraries
import pygame
import numpy as np

# Modules
import drawline
import physics
import controller
import lowpass

## Defines ##
# Simulation defines
CART_SPACE = 0.35

# Rendering defines
HEIGHT = 500
WIDTH = 1000
MIDDLE = (WIDTH/2, HEIGHT/2)

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

pygame.init()
screen = pygame.display.set_mode([WIDTH, HEIGHT])
clock = pygame.time.Clock()
line = drawline.Line(screen)
font = pygame.font.Font('freesansbold.ttf', 32)
text_enabled = font.render('Control Enabled', True, GREEN)
text_disabled = font.render('Control Disabled', True, RED)

## Physics ##
cart = physics.PhysicsCart()
pendulum = physics.PhysicsCartPendulum(cart)
cart.addPendulum(pendulum)
controller1 = controller.Control()
controller2 = controller.Control()

## Sim variables ##
# I/O
enable_control = True
disturb = 0

pendulum.theta = 3.14
theta = pendulum.theta

# Time
dt = 0

# Properties
pendulum.l = 0.1
pendulum.cf = 0.5
pendulum.m = 0.2
cart.m = 1
cart.cf = 5
cart.x = 0.0

f = 0

cart.set_limit(CART_SPACE)

# Space
coordinates = list(MIDDLE)

# PID
controller1.kp = 1.0
controller1.ki = 0.2
controller1.kd = 0.6

controller2.kp = 20
controller2.ki = 0.5
controller2.kd = 1.5

controller2.addLPFilter(0.01)
controller2.addLimit(5)

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
                disturb = -10
            if event.key == pygame.K_RIGHT:
                disturb = 10
            if event.key == pygame.K_DOWN:
                enable_control = False
    
    ## Simulation ##
    # Controller
    if(enable_control):
        if(theta > 3.14 + 0.5 or theta < 3.14 - 0.5):
            running = False
    
        sp_theta = controller1.control(cart.x, 0, dt)
        f = -controller2.control(pendulum.theta, sp_theta+3.14, dt) + disturb
    else:
        f = 0
    disturb = 0

    cart.f = f
    x = cart.step(dt)
    theta = pendulum.step(dt)

    ## Rendering ## 
    # Background
    screen.fill(WHITE)

    # Additional visuals
    pygame.draw.circle(screen, (0, 0, 0), 
                       (WIDTH/2+(CART_SPACE*1000)+rect_h, HEIGHT/2), 5)
    pygame.draw.circle(screen, (0, 0, 0), 
                       (WIDTH/2-(CART_SPACE*1000)-rect_h, HEIGHT/2), 5)
    pygame.draw.line(
        screen, 
        (10, 10, 10), 
        (WIDTH/2+(CART_SPACE*1000)+rect_h, HEIGHT/2), 
        (WIDTH/2-(CART_SPACE*1000)-rect_h, HEIGHT/2)
        )
    if(enable_control):
        screen.blit(text_enabled, (20, 20))
    else:
        screen.blit(text_disabled, (20, 20))

    # Cart + Pendulum
    rect_x = coordinates[0]-rect_w/2+(x*1000)
    rect_y = coordinates[1]-rect_h/2
    rect = pygame.Rect((rect_x, rect_y), 
                       (rect_w, rect_h))
    pygame.draw.rect(screen, BLUE, rect)
    line.polarr((rect.center), theta, pendulum.l*1000, RED)

    # Refresh
    pygame.display.flip()

    # Time keeping
    dt = clock.tick(120)/1000

controller2.plotLogs("Controller 2", False)
controller1.plotLogs("Controller 1", True)

pygame.quit()