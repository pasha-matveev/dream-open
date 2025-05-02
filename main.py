import cv2 as cv
import logging
import time
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

field = Field()
robot = Robot(Point(0, -40))
ball = Ball(Point(0, 0))
obstacles = []

uart = UART()
lidar = Lidar(args)

# camera.start()
#uart.start()
#lidar.start()

lst = time.time()
try:
    loop_fps = 50
    while True:
        start_time = time.time()

        if uart.new_data:
            uart.read()
            robot.update_state(uart.data)

        if lidar.new_data:
            lidar.compute()

            robot.update_pos(*lidar.robot_data)

            # update obstacles
        else:
            # predict robot position and angle based on its velocity
            pass
        
        if camera.new_data:
            camera.compute()
            camera.preview()
            
            ball.update_pos(robot, camera.ball)
            
            # update open goal space

        # strategy
        
        # print(1/loop_fps - (time.time() - start_time))
        time.sleep(max(0, 1/loop_fps - (time.time() - start_time)))
        # time.sleep(max(0, 1/loop_fps))
        print(1 / (time.time() - start_time))


except KeyboardInterrupt:
    print("Keyboard interrupt caught. Terminating child process.")
finally:
    camera.stop()
    uart.stop()
    lidar.stop()
    print("Main loop ended. Python script finished.")
