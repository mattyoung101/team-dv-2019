import serial
import pygame
import os
from numpy import interp

# can't believe we have to do this
os.environ["SDL_VIDEO_CENTERED"] = "1"

# ser = serial.Serial("COM5", 115200)
# just pylint being a dumbass as usual, please ignore
# pylint: disable=no-member
pygame.init()
display = pygame.display.set_mode((1180, 750), 0, 32)
pygame.display.set_caption("TSOP Debug")
clock = pygame.time.Clock()

running = True
while running:
    clock.tick(60)

    for i in pygame.event.get():
        if i.type == pygame.QUIT or (i.type == pygame.KEYUP and i.key == pygame.K_ESCAPE):
            running = False
            break
    display.fill((255, 0, 0))

    # line = ser.readline().decode("utf-8")

    # if line.startswith("TSOP_DEBUG_BEGIN"):
        # actually need to split this into vectors or something
        # values = line.split()[1::]
        # pass

    pygame.display.update()

pygame.quit()

# while True:
