import cv2 as cv
import logging

from scripts.arg_parser import generate_args
from modules.camera.camera_cv2 import CameraCV2
from modules.uart import UART

args = generate_args()
logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING)
camera = CameraCV2(args)
uart = UART()

while True:
    camera.read()
    print(f'[Arduino]: {uart.data}')
    # logging.info(f'Arduino data: {uart.data}')
    uart.write('fff', camera.ball.angle, 10, camera.ball.angle)

    camera.preview()
    key = cv.waitKey(30)
    if key == ord('s'):
        camera.save()
    if key == ord('q') or key == 27:
        break
