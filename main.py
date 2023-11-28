# Libraries
import pygame
import numpy as np

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

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

## Rendering Objects ##
pygame.init()
screen = pygame.display.set_mode([WIDTH, HEIGHT])
clock = pygame.time.Clock()
line = drawline.Line(screen)
font = pygame.font.Font('freesansbold.ttf', 32)
text_enabled_PID = font.render('Control Enabled PID', True, GREEN)
text_enabled_LQR = font.render('Control Enabled LQR', True, GREEN)
text_disabled = font.render('Control Disabled', True, RED)

## Physics ##
cart = physics.PhysicsCart()
pendulum = physics.PhysicsCartPendulum(cart)
cart.addPendulum(pendulum)

## Sim variables ##
# I/O
enable_control = 1
disturb = 0

pendulum.theta = 0.01
theta = pendulum.theta

# Time
dt = 0

# Properties
pendulum.l = 0.10
pendulum.cf = 0.5
pendulum.m = 0.2
cart.m = 1
cart.cf = 5
cart.x = 0.0

f = 0

cart.set_limit(CART_SPACE)

# Space
coordinates = list(MIDDLE)

## Controllers ## 
ctrlLQR = controller.ControlLQR(pendulum)
ctrlPID1 = controller.ControlPID()
ctrlPID2 = controller.ControlPID()

ctrlLQR.init()
# ctrlLQR.toggleNoise(0.01)

ctrlPID1.kp = 1.0
ctrlPID1.ki = 0.2
ctrlPID1.kd = 0.6

ctrlPID2.kp = 20
ctrlPID2.ki = 0.5
ctrlPID2.kd = 1.5

# ctrlPID2.addLPFilter(0.006)
# ctrlPID2.addLimit(5)

ctrlPID1.enableLogging()
ctrlPID2.enableLogging()

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
            if event.key == pygame.K_UP:
                if(enable_control == 1):
                    enable_control = 2
                else:
                    enable_control = 1
    
    ## Simulation ##
    # Controller
    if(enable_control != 0):
        if(theta > 0 + 0.5 or theta < 0 - 0.5):
            running = False

    if(enable_control == 1):
        f = ctrlLQR.control() + disturb
    elif(enable_control == 2):
        sp_theta = -ctrlPID1.control(cart.x, 0, dt)
        f = ctrlPID2.control(pendulum.theta, sp_theta, dt) + disturb
    else:
        f = 0
    disturb = 0

    # Physics
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
    if(enable_control == 1):
        screen.blit(text_enabled_LQR, (20, 20))
    elif(enable_control == 2):
        screen.blit(text_enabled_PID, (20, 20))
    else:
        screen.blit(text_disabled, (20, 20))

    # Cart + Pendulum
    rect_x = int(coordinates[0]-rect_w/2+(x*1000))
    rect_y = int(coordinates[1]-rect_h/2)
    rect = pygame.Rect((rect_x, rect_y), (rect_w, rect_h))
    pygame.draw.rect(screen, BLUE, rect)
    line.polarr((rect.center), theta, pendulum.l*1000, RED)

    # Refresh
    pygame.display.flip()

    # Time keeping
    dt = clock.tick(120)/1000

pygame.quit()

ctrlPID2.plotLogs("Controller 2 PID", False)
ctrlPID1.plotLogs("Controller 1 PID", True)