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
from path_planning.algorithms import dubins_path

field = Field()
robot = Robot(Point(40, 40), True)
ball = Ball(Point(0, 0))
obstacles = []

vis = Visualization(field, ball, robot, obstacles, fps=30)
vis.start()

uart = UART()
lidar = Lidar(args)

#camera.start()
#uart.start()
#lidar.start()

gyro_correction = 0

lst = time.time()
try:
    loop_fps = 30
    while True:
        start_time = time.time()
        vis._clear()
        vis._draw()

        if uart.new_data:
            uart.read()
            robot.update_state(uart.data)
            

        if lidar.new_data:
            lidar.compute()
            
            if math.cos(lidar.field.rotation - robot.gyro) < 0:
                lidar.field.rotation += math.pi
                        
            if abs(1 - lidar.field.width/250) < 0.2 and abs(1 - lidar.field.height/186) < 0.2:
                robot.update_pos(lidar.field)
                gyro_correction = robot.gyro - lidar.field.rotation

            # update obstacles
        else:
            robot.predict_pos()
        
        if camera.new_data:
            camera.compute()
            camera.preview()
            
            ball.update_pos(robot, camera.ball)
            
            # update open goal space
        
        robot.kick = 0
        if robot.is_attacker:
            angle = math.pi/2
            dubins_point = ball.pos.move(Vector.from_angle(angle + math.pi, 7))
            vector = dubins_path(vis.view, robot, dubins_point, angle, 20)
            vector.length *= 2
            robot.vel = vector
            robot.rot_limit = 30
            # if robot.emitter:
            #     robot.vel = Vector(0, 0)
            #     robot.dribbling = 90
            #     if time.time() - robot.first_tm > 1:
            #         robot.rotation = camera.yellow_goal.angle - robot.gyro
            #         robot.rot_limit = 10
            #     # if math.cos(camera.yellow_goal.angle - robot.gyro) > 0.99:
            #         # robot.kick = 20
            # else:
            #     robot.vel = Vector.from_points(robot.pos, ball.pos)
            #     robot.vel.length = robot.vel.length * 1 + 20
            #     robot.dribbling = 70
            #     robot.rotation = robot.pos.angle(ball.pos)
            #     robot.rot_limit = 30
        else:
            if time.time() - ball.visible_tm < 1:
                robot.vel = Vector.from_points(robot.pos, ball.pos)
                robot.vel.length *= 2

        # limit robot speed based on field borders
        nearest = field.nearest_border(robot.pos)
        if field.is_inside(robot.pos):
            nearest.constrain(robot, 2)
            for border in field.borders:
                if border.is_inside(robot.pos) and border != nearest:
                    border.constrain(robot, 2)
        else:
            border_vec = Vector.from_points(robot.pos, nearest.nearest_point(robot.pos))
            normal = robot.vel.normal(border_vec.angle)
            tangent = robot.vel.tangent(border_vec.angle)
            normal.angle = border_vec.angle
            normal.length = max(border_vec.length * 2, normal.length)
            robot.vel = normal + tangent
        robot.limit_speed(50)
        
                
        # send data to arduino
        if uart.writable:
            uart.write('ffffii', -robot.vel.angle + math.pi / 2 + gyro_correction, robot.vel.length, -robot.rotation + math.pi / 2, robot.rot_limit, robot.dribbling, robot.kick)
            #uart.write('fffii', 0, 0, robot.gyro, 70, 0)
        
        # update visualization
        # if vis.update_tm:
        #     vis.step()
        
        vis._update()
        # maintain loop rate
        time.sleep(max(0, 1/loop_fps - (time.time() - start_time)))
        #print(1 / (time.time() - start_time))


except KeyboardInterrupt:
    print("Keyboard interrupt caught. Terminating child process.")
finally:
    camera.stop()
    lidar.stop()
    print("Main loop ended. Python script finished.")
