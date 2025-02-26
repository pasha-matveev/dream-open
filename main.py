import cv2 as cv
from scripts.arg_parser import generate_args
from modules.camera.camera_cv2 import CameraCV2

args = generate_args()
camera = CameraCV2(args)

while True:
    camera.read()
    camera.preview()
    key = cv.waitKey(30)
    if key == ord('s'):
        camera.save()
    if key == ord('q') or key == 27:
        break
