# Libraries
import pygame
import numpy as np
import math
import matplotlib.pyplot as plt

# Modules
from modules import drawline
from modules import physics
from modules import controller

## Defines ##
# Simulation defines
CART_SPACE = 2

# Rendering defines
HEIGHT = 500
WIDTH = 1000
MIDDLE = (WIDTH/2, HEIGHT/2)
SCALE = 200

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
cart = physics.PhysicsCart()
pendulum = physics.PhysicsDoublePendulum(cart)
cart.addPendulum(pendulum)
control = controller.ControlLQR(pendulum)

## Sim variables ##
# I/O
enable_control = True
disturb = 0

# Time
dt = 0
dt_cart = 0

# Properties
pendulum.l1 = 0.4
pendulum.m1 = 0.1
pendulum.cf = 0#pendulum.m1/20.1428571428571428571428571428571


pendulum.l2  = 0.4
pendulum.m2  = 0.1
pendulum.cf2 = 0#pendulum.m2/20.1428571428571428571428571428571
cart.m = 1
pendulum.M = cart.m

f = 0
cart.set_limit(CART_SPACE)
# pendulum.set_limit(CART_SPACE)
# pendulum.init()

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


cartAccarray = []
time = []
timerVar = 0
teller = 0
recordFlag = False

running = True
while running:
    ## Inputs ##
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                disturb = -500
                recordFlag = True
            if event.key == pygame.K_RIGHT:
                disturb = 500
                recordFlag = True
            if event.key == pygame.K_DOWN:
                enable_control = False
    
    ## Simulation ##
    # Controller
    if(enable_control):
# =============================================================================
#         if(theta > 3.14 + 0.5 or theta < 3.14 - 0.5):
#             running = False
# =============================================================================
        f = control.control()
        if(f>1000):
            f=1000
        elif(f<-1000):
            f=-1000
        f += disturb       
        # print(f, disturb, f-disturb)
        
    else:
        f = 0 + disturb

    cart.f = f
    theta1, theta2, x = pendulum.step(dt)
    if((theta1 > math.pi/2) or (theta1 < -math.pi/2)):
        enable_control = False
    f = 0

    if(recordFlag == True):
        if(timerVar < 1):
            # cartAccarray.append(cart.x_d) 
            # time.append(timerVar)
            timerVar += dt
        # if(timerVar > 1):
            # plt.plot(time, cartAccarray, label='Cart Acceleration')
            # plt.xlabel('Time')
            # plt.ylabel('Cart Acceleration')
            # plt.title('Time vs Cart Acceleration')
            # plt.legend()
            # plt.show()
            # break
        if(timerVar > 0.05):
            disturb = 0
            timeVar = 0


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
    dt = clock.tick(480)/5000

pygame.quit()