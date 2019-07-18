import sensor, image, time, utime, pyb
from pyb import UART
import ucollections

# OpenMV object tracking, by Matt Young
# Serial out format:
# [0xB, bfound, bx, by, yfound, yx, yy, 0xE] (6 bytes not including 0xB and 0xE)

thresholds = [(100, 100, 100, 100, 100, 100), # yellow
             (40, 45, -15, 6, -43, -5), # blue
             (54, 77, 28, 80, 1, 127)] # orange

# Normal
# Blue (31, 50, -13, 24, -70, -21)
# Yellow (50, 89, -9, 34, 26, 74)

# Field 6
# Blue (30, 43, -13, 14, -46, -10)

# Field 10
# Blue (27, 45, -8, 17, -61, -12)
# Yellow (60, 94, -3, 43, 38, 97)

# Field 12
# Blue (28, 38, -17, 10, -48, -8)

# this comes from the output of blob.code()
# you're meant to compare them using binary (see docs) but... yeah nah
YELLOW = 1
BLUE = 2
ORANGE = 3

pyb.LED(1).on()

debug = True
light = False
out = []
clock = time.clock()
uart = UART(3)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)

# sensor setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) #Resolution, QVGA = 42FPS,QQVGA = 85FPS

sensor.skip_frames(time=500)

sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)
# Need to let the above settings get in...
sensor.skip_frames(time=500)
sensor.set_windowing((36, 5, 112, 112)) # Robot 0
#sensor.set_windowing((38, 8, 112, 112)) # Robot 1

# === GAIN ===
curr_gain = sensor.get_gain_db()
sensor.set_auto_gain(False, gain_db=curr_gain)

# === EXPOSURE ===
curr_exposure = sensor.get_exposure_us()
sensor.set_auto_exposure(False, exposure_us = int(curr_exposure))

# === WHITE BAL ===
sensor.set_auto_whitebal(False,
rgb_gain_db=(-6.02073, -4.19219, -0.5613652))

# Standard
sensor.set_brightness(0)
sensor.set_contrast(3)
sensor.set_saturation(3)

sensor.skip_frames(time=500)

# Blink LEDs
pyb.LED(1).off()
for i in range(3):
    pyb.LED(2).on()
    pyb.delay(50)
    pyb.LED(2).off()
    pyb.delay(50)

if light:
    pyb.LED(1).on()
    pyb.LED(2).on()
    pyb.LED(3).on()

# Find biggest blob of specific colour
def scanBlobs(blobs, colour):
    biggestBlob = None
    bbArea = 0

    for blob in blobs:
        if blob.code() != colour:
            continue

        if biggestBlob == None:
            biggestBlob = blob
            bbArea = blob.area()
        else:
            blobArea = blob.area()

            if blobArea > bbArea:
                biggestBlob = blob
                bbArea = blobArea

    return biggestBlob

# Main loop
while True:
    begin = utime.time()
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(thresholds, x_stride=3, y_stride=3, pixels_threshold=15,
            area_threshold=15, merge=True, margin=2)
    biggestYellow = scanBlobs(blobs, YELLOW)
    biggestBlue = scanBlobs(blobs, BLUE)

    print(thresholds[-1])
    orangeBlobs = img.find_blobs([thresholds[-1]], x_stride=3, y_stride=3, pixels_threshold=15,
                    area_threshold=15, merge=True, margin=2)
    biggestOrange = sorted(orangeBlobs, key=lambda l: l.area(), reverse=True)[0]

    # Debug drawing
    if biggestYellow != None and debug:
        img.draw_rectangle(biggestYellow.rect(), color=(255, 255, 0))
        img.draw_cross(biggestYellow.cx(), biggestYellow.cy())
        img.draw_string(biggestYellow.cx(), biggestYellow.cy(), "Goal_Y",
                        color=(255, 0, 0))

    if biggestBlue != None and debug:
        img.draw_rectangle(biggestBlue.rect(), color=(0, 0, 255))
        img.draw_cross(biggestBlue.cx(), biggestBlue.cy())
        img.draw_string(biggestBlue.cx(), biggestBlue.cy(), "Goal_B",
                        color=(255, 0, 0))

    if biggestOrange != None and debug:
        img.draw_rectangle(biggestOrange.rect(), color=(254, 95, 27))
        img.draw_cross(biggestOrange.cx(), biggestOrange.cy())
        img.draw_string(biggestOrange.cx(), biggestOrange.cy(), "Ball",
                        color=(255, 0, 0))

    # Serial out preparation
    out.clear()
    out += [0xB]

    if biggestBlue == None:
        out += [False, 0, 0]
    else:
        out += [True, int(biggestBlue.cx()), int(biggestBlue.cy())]

    if biggestYellow == None:
        out += [False, 0, 0]
    else:
        out += [True, int(biggestYellow.cx()), int(biggestYellow.cy())]

    if biggestOrange == None:
        out += [False, 0, 0]
    else:
        out += [True, int(biggestOrange.cx()), int(biggestOrange.cy())]

    out += [0xE]

    #pyb.LED(2).on()
    for byte in out:
        uart.writechar(byte)
    #pyb.LED(2).off()

    if debug:
        img.draw_string(5, 5, "" + "\n".join(str(x) for x in out))

    print(clock.fps())
