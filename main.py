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
from path_planning.visualization import Visualization

field = Field()
robot = Robot(Point(0, 0))
ball = Ball(Point(0, 0))
obstacles = []

vis = Visualization(field, ball, robot, fps=10)
vis.start()

uart = UART()
lidar = Lidar(args)

#camera.start()
uart.start()
lidar.start()

time.sleep(7)

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
            
            if math.cos(lidar.field.rotation - robot.gyro) < 0:
                lidar.field.rotation += math.pi
            
            if abs(1 - lidar.field.width/242) < 0.2 and abs(1 - lidar.field.height/122) < 0.2:
                robot.update_pos(lidar.field)
                print(robot.pos, '\t', round(lidar.field.rotation, 2), '\t', round(robot.gyro, 2))
            else:
                print('rotation')

            # update obstacles
        else:
            robot.predict_pos()
        
        if camera.new_data:
            camera.compute()
            camera.preview()
            
            ball.update_pos(robot, camera.ball)
            
            # update open goal space
        
        robot.vel = Vector.from_points(robot.pos, ball.pos)
        robot.vel.length *= 2
        # ball.attract(robot, 0, 200 / loop_fps)

        nearest = field.nearest_border(robot.pos)

        if field.is_inside(robot.pos):
            nearest.constrain(robot, 1)
            for border in field.borders:
                if border.is_inside(robot.pos) and border != nearest:
                    border.constrain(robot, 1)
        else:
            border_vec = Vector.from_points(robot.pos, nearest.nearest_point(robot.pos))
            normal = robot.vel.normal(border_vec.angle)
            tangent = robot.vel.tangent(border_vec.angle)
            normal.angle = border_vec.angle
            normal.length = max(border_vec.length * 2, normal.length)
            robot.vel = normal + tangent
        
        ball.constrain(robot, 2)
        robot.limit_speed()
            
        uart.write('fffii', robot.vel.angle, robot.vel.length, 0, 0, 0)
        #uart.write('fffii', 0, 0, robot.gyro, 0, 0)
        
        if vis.update_tm:
            vis.step()
        
        time.sleep(max(0, 1/loop_fps - (time.time() - start_time)))
        # print(1 / (time.time() - start_time))


except KeyboardInterrupt:
    print("Keyboard interrupt caught. Terminating child process.")
finally:
    camera.stop()
    lidar.stop()
    print("Main loop ended. Python script finished.")
