import cv2 as cv
import logging
import time

from scripts.arg_parser import generate_args
try:
    from modules.camera.camera import Camera
except:
    from modules.camera.camera_cv2 import CameraCV2
from modules.uart import UART
from modules.lidar import Lidar

args = generate_args()
logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING)
# camera = CameraCV2(args)
uart = UART()
lidar = Lidar(args)

lidar.start()

try:
    while True:
        if lidar.new_data():
            print(lidar.output_queue.get())

        # camera.read()
        # uart.write('fffii', 0, 30, 90, 0, 0)
        # print(uart.data)

        time.sleep(0.03)

        # camera.preview()
        # key = cv.waitKey(30)
        # if key == ord('s'):
        #     camera.save()
        # if key == ord('q') or key == 27:
        #     break
except KeyboardInterrupt:
    print("Keyboard interrupt caught. Terminating child process.")
    lidar.stop()
    print("Main loop ended. Python script finished.")