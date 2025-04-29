import cv2 as cv
import logging
import time
import struct
from path_planning.models import *

from scripts.arg_parser import generate_args
args = generate_args()
logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING)

try:
    from modules.camera.camera import Camera
    camera = Camera(args)
except:
    from modules.camera.camera_cv2 import CameraCV2
    camera = CameraCV2(args)
from modules.uart import UART
from modules.lidar import Lidar

uart = UART(port='/dev/ttyUSB1', input_format='ffff??')
lidar = Lidar(args)

camera.start()
uart.start()
lidar.start()

field = Field()
robot = Robot(Point(0, 0))
ball = Ball(Point(0, 40))

try:
    while True:
        camera.read()
        
        if lidar.new_data():
            lidar_data = lidar.output_queue.get()
            
            angle = float(lidar_data[0])
            dist = float(lidar_data[1])
            print(angle, dist)

        camera.preview()
        key = cv.waitKey(40)
        if key == ord('s'):
            camera.save()
        if key == ord('q') or key == 27:
            break

except KeyboardInterrupt:
    print("Keyboard interrupt caught. Terminating child process.")
finally:
    lidar.stop()
    print("Main loop ended. Python script finished.")
