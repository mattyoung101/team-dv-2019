import pygame
import numpy as np
import math
import random

# pylint: disable=no-member
pygame.init()
display = pygame.display.set_mode((1280, 720), 0, 32)
pygame.display.set_caption("Light Sensor Visualisation")
clock = pygame.time.Clock()
CIRCLE_SIZE = 300
RED = (255, 0, 0)
BLACK = (0, 0, 0)

running = True
while running:
    clock.tick(60)

    for i in pygame.event.get():
        if i.type == pygame.QUIT or (i.type == pygame.KEYUP and i.key == pygame.K_ESCAPE):
            running = False
            break
    display.fill((255, 255, 255))

    points = np.linspace(0, 360, 48)
    for angle in points:
        # cos theta is the x coord, sin theta is the y coord
        x = int(CIRCLE_SIZE * math.cos(math.radians(angle)) + (1280 / 2))
        y = int(CIRCLE_SIZE * math.sin(math.radians(angle)) + (720 / 2))
        pygame.draw.circle(display, RED, (x, y), 8)

    pygame.display.update()
pygame.quit()