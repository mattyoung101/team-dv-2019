import sensor, image, time
from pyb import UART, LED

cam_debug = 1 # Whether to draw camera data
fps_debug = 0  # Whether to print out the FPS

robot = 1 # Robot number

# LAB thresholds for ball and goals
if robot == 1:
    cam_thresholds = [((52, 81, 18, 74, 18, 66),),  # Ball
                    ((0, 7, -19, 18, -33, 13),),  # Blue Goal
                    ((47, 100, 8, 33, 25, 70),)]   # Yellow Goal
else:
    cam_thresholds = [((52, 69, 44, 76, -7, 56),), # Ball
                    ((38, 64, 12, 48, -91, -48),),  # Blue Goal
                    ((48, 100, -7, 37, 26, 73),)]   # Yellow Goal

thresholds = [(0, 0, 0, 0, 0, 0), # yellow
              (0, 0, 0, 0, 0, 0), # blue
              (52, 81, 18, 74, 18, 66)] # orange

# yellow - (43, 74, -10, 53, 15, 64)

def largest_blob(list_blobs):
    # Returns the largest blob from a set of blobs
    if not list_blobs:
        return None
    return sorted(list_blobs, key=lambda blob: -blob.area())[0]


class Sender:
    """Sends data to robot"""

    def __init__(self):
        # Initialise serial
        self.uart = UART(3, 9600, timeout_char=1000)

    def send_data(self, send_buffer):
        # Sends the position of the ball and goals
        self.uart.writechar(255)
        for b in send_buffer:
            self.uart.writechar(b)

class Reader:
    """Reads camera data"""

    def __init__(self, thresholds, debug_cam=False):
        # Initialises camera
        self.setup_camera()

        self.thresholds = thresholds
        self.debug = debug_cam

    def update(self):
        # Updates the camera
        img = sensor.snapshot()
        #img.draw_circle(img.width()//2, img.height()//2, 55, (255, 255, 255), fill=True)

        # Get the x and y coordinates of the ball and goals
        ball_blobby = largest_blob(img.find_blobs(self.thresholds[0], x_stride=2, y_stride=2, merge=True, margin=10))
        ball_blob = self.get_xy(ball_blobby)
        blue_blob = self.get_xy(largest_blob(img.find_blobs(self.thresholds[1], x_stride=15, y_stride=8, merge=True, margin=34, pixels_threshold=30)))
        yellow_blobby = largest_blob(img.find_blobs(self.thresholds[2], x_stride=15, y_stride=8, merge=True, margin=34, pixels_threshold=50))
        yellow_blob = self.get_xy(yellow_blobby)
        # Draws lines to all blobs seen from centre of screen
        # Also draws cross at the centre
        centre_x = img.width() // 2
        centre_y = img.height() // 2
        #print(centre_x,centre_y)
        img.draw_cross(centre_x, centre_y)
        if self.debug:

            if ball_blob != (254, 254):
                img.draw_line((centre_x, centre_y, ball_blob[0], ball_blob[1]), color=(255, 92, 0), thickness=2)
                img.draw_rectangle(ball_blobby.x(), ball_blobby.y(), ball_blobby.w(), ball_blobby.h())
                #print(ball_blob[0]-centre_x, centre_y-ball_blob[1])
                #ball_distance = ((ball_blob[0] - centre_x)**2 + (ball_blob[1] - centre_y)**2)**0.5
                #print("Ball Distance:", ball_distance)
            if blue_blob != (254, 254):
                img.draw_line((centre_x, centre_y, blue_blob[0], blue_blob[1]), color=(0, 75, 255), thickness=2)
            if yellow_blob != (254, 254):
                img.draw_line((centre_x, centre_y, yellow_blob[0], yellow_blob[1]), color=(255, 255, 0), thickness=2)
                img.draw_rectangle(yellow_blobby.x(), yellow_blobby.y(), yellow_blobby.w(), yellow_blobby.h())
                #goal_dis = ((yellow_blob[0] - centre_x)**2 + (yellow_blob[1] -  centre_y)**2)**0.5
                #print(goal_dis)
        self.data = (ball_blob[0], ball_blob[1], blue_blob[0], blue_blob[1], yellow_blob[0], yellow_blob[1])

    def get_xy(self, blob):
        # Returns the centre x and y coords of the blob,
        # otherwise some mock values which will never be seen
        return (blob.cx(), blob.cy()) if blob else (254, 254)

    def get_data(self):
        # Returns coordinates of all blobs
        return self.data

    def setup_camera(self):
        # Sets up the camera with the following settings
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=1000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False,value=(-6.02073, -6.02073, -0.707413))
        #sensor.set_auto_exposure(False, exposure_us=15000)
        sensor.set_windowing((250,240))
        sensor.set_saturation(3)
        sensor.set_brightness(0)
        sensor.set_contrast(3)

        LED(1).on()
        time.sleep(100)
        LED(1).off()


send = Sender()
cam = Reader(cam_thresholds, debug_cam=cam_debug)

if fps_debug:
    clock = time.clock()

while True:
    # Updates camera and sends data
    cam.update()
    send.send_data(cam.get_data())

    # Prints FPS
    if fps_debug:
        print(clock.fps())
        clock.tick()
