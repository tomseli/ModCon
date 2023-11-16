import pygame
import drawline

pygame.init()

HEIGHT = 500
WIDTH = 1000
MIDDLE = (WIDTH/2, HEIGHT/2)

WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

screen = pygame.display.set_mode([WIDTH, HEIGHT])
line = drawline.Line(screen)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    screen.fill(WHITE)

    line.polard(MIDDLE, 270, 200, RED)

    # Simular to update, but faster?
    pygame.display.flip()

pygame.quit()