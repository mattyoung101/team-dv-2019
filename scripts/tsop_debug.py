import serial
import pygame
from numpy import interp

ser = serial.Serial("COM8", 115200)
# just pylint being a dumbass as usual, please ignore
# pylint: disable=no-member
pygame.init()
display = pygame.display.set_mode((1280, 128), 0, 32)
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

    line = ser.readline().decode("utf-8")

    if line.startswith("BEGIN_TSOP_DEBUG"):
        values = [int(interp(float(x), [0.0, 1.0], [0, 255])) for x in line.split()[1::]]
        rect_width = 1280 / 24
        for sensor in range(24):
            colour = values[sensor]
            pygame.draw.rect(display, (colour, colour, colour), (sensor * rect_width, 0, rect_width + 1, 128))

    pygame.display.update()

pygame.quit()
