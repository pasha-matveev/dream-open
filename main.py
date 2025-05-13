from path_planning.algorithms import dubins_path
from path_planning.visualization import Visualization
from modules.lidar import Lidar
from modules.uart import UART
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

gyro_correction = 0
active_deff = False
straight_path = False
front_color = 'yellow'

robot = Robot(Point(40, 40), False)
field = Field([(-78, -108.5), (-55.4, -108.5), (-29, -82.1), (29, -82.1), (55.4, -108.5),
               (78, -108.5), (78, 108.5), (55.4, 108.5), (29, 82.1), (-29, 82.1),
               (-55.4, 108.5), (-78, 108.5)])
attacker_field = Field([(78, -40), (78, 108.5), (55.4, 108.5), (29, 82.1), (-29, 82.1),
                        (-55.4, 108.5), (-78, 108.5), (-78, -40)])
defender_field = Field([(-78, -108.5), (-55.4, -108.5), (-29, -82.1), (29, -82.1), (55.4, -108.5),
                        (78, -108.5), (78, -40), (-78, -40)])
ball = Ball(Point(0, 0))
obstacles = []
front_goal = Goal(112, camera.yellow_goal if front_color ==
                  'yellow' else camera.blue_goal, front_color)
back_goal = Goal(-112, camera.blue_goal if front_color ==
                 'yellow' else camera.yellow_goal, 'blue' if front_color == 'yellow' else 'yellow')

vis = Visualization(defender_field, ball, robot, obstacles,
                    front_goal, back_goal, fps=1)
uart = UART()
lidar = Lidar(args)

#vis.start()
camera.start()
uart.start()
lidar.start()


lst = time.time()
try:
    loop_fps = 50
    while True:
        start_time = time.time()
        robot.kick = 0

        if uart.new_data:  # interpret data from uart
            uart.read()
            robot.update_state(uart.data)

        if lidar.new_data:  # interpret data from lidar
            lidar.compute()

            if math.cos(lidar.field.rotation - robot.gyro) < 0:
                lidar.field.rotation += math.pi

            if abs(1 - lidar.field.width/250) < 0.2 and abs(1 - lidar.field.height/186) < 0.2:
                robot.update_pos(lidar.field)
                gyro_correction = robot.gyro - lidar.field.rotation
            else:
                robot.predict_pos()
            
            obstacles = []
            for obstacle in lidar.obstacles_data:
                obstacles.append(Obstacle.from_lidar(robot, obstacle))
                
            for obstacle in obstacles:
                print(obstacle.pos, end='\t')
            print()
        else:
            robot.predict_pos()

        if camera.new_data:  # interpret data from camera
            camera.compute()
            camera.preview()

            if camera.ball.visible:
                ball.update_pos(robot, camera.ball)

            if front_goal.camera_object.visible:
                front_goal.update_free_space(robot)

            if back_goal.camera_object.visible:
                back_goal.update_free_space(robot)

        if robot.is_attacker:  # attacker ------------------------------------------------------------------------

            if not attacker_field.is_inside(ball.pos) and robot.pos.dist(ball.pos) < 30:
                straight_path = True

            if attacker_field.is_inside(ball.pos) and robot.pos.dist(ball.pos) > 40:
                straight_path = False

            if straight_path:  # straight path stratagy
                if robot.emitter:
                    angle = robot.pos.angle(front_goal.free_space)
                    robot.dribbling = 100
                    robot.rotation = angle
                    robot.rot_limit = 10
                    robot.vel = Vector.from_angle(-robot.gyro + math.pi / 2, 0)
                    if math.cos(angle - (-robot.gyro + math.pi / 2)) > 0.99:
                        robot.kick = 20
                else:
                    robot.vel = Vector.from_points(robot.pos, ball.pos)
                    robot.vel.length *= 2
                    robot.dribbling = 70
                    robot.rotation = robot.pos.angle(ball.pos)
                    robot.rot_limit = 30
                    #print('straight movement')

            else:  # dubnis path
                angle = ball.pos.angle(front_goal.free_space)
                robot.rotation = angle
                robot.rot_limit = 30
                if math.cos(angle - robot.pos.angle(ball.pos)) > 0.98:
                    robot.vel = Vector.from_points(robot.pos, ball.pos)
                    #print('dubins straight')
                else:
                    dubins_point = ball.pos.move(
                        Vector.from_angle(angle + math.pi, 7))
                    dubins_path(robot, dubins_point, angle, 10, attacker_field)
                    #print('dubins curve')

                robot.dribbling = 70
                if robot.emitter:
                    robot.kick = 20

            robot.limit_speed(attacker_field.border_dist(robot.pos) * 5 + 50)
            #robot.limit_speed(50)
            # robot.limit_acc(100)
            attacker_field.applay_constraints(robot)

        else:  # defender ----------------------------------------------------------------------------------------

            if (time.time() - ball.visible_tm) < 1:

                if ball.pos.y < -40:
                    active_deff = True
                if ball.pos.y > -30:
                    active_deff = False

                if active_deff:
                    if robot.emitter:
                        angle = robot.pos.angle(front_goal.center)
                        robot.dribbling = 100
                        robot.rotation = angle
                        robot.rot_limit = 10
                        robot.vel = Vector.from_angle(-robot.gyro + math.pi / 2, 10)
                        if math.cos(angle - (-robot.gyro + math.pi / 2)) > 0.95:
                            robot.kick = 20
                        #print('emitter')
                    else:
                        robot.vel = Vector.from_points(robot.pos, ball.pos)
                        robot.vel.length *= 2
                        robot.dribbling = 70
                        robot.rotation = robot.pos.angle(ball.pos)
                        robot.rot_limit = 30
                        #print('active_deff')
                else:
                    target = Point(
                        ball.pos.x / (ball.pos.y + 100) * (-80 + 100), -80)
                    robot.vel = Vector.from_points(robot.pos, target)
                    robot.vel.length *= 5
                    robot.dribbling = 0
                    robot.rot_limit = 30
                    #print('passive_deff')

            else:
                close_obstacles = [obstacle for obstacle in obstacles if defender_field.is_inside(obstacle.pos)]
                if len(close_obstacles):
                    big_obstacle = max(close_obstacles, key=lambda x: x.radius)
                    robot.vel = Vector.from_points(robot.pos, Point(big_obstacle.pos.x, -80))
                    robot.rotation = math.pi / 2
                    robot.dribbling = 0
                    robot.rot_limit = 30
                else:
                    robot.vel = Vector.from_points(robot.pos, Point(0, -80))
                    robot.rotation = math.pi / 2
                    robot.dribbling = 0
                    robot.rot_limit = 30
                    #print('no ball')


            robot.limit_speed()
            defender_field.applay_constraints(robot)



        # send data to arduino
        if uart.writable:
            uart.write('ffffii', -robot.vel.angle + math.pi / 2 + gyro_correction, robot.vel.length, -
                       robot.rotation + math.pi / 2, robot.rot_limit, robot.dribbling, robot.kick)
            # uart.write('fffii', 0, 0, robot.gyro, 70, 0)

        # update visualization
        #if vis.update_tm:
            #vis.step()

        # maintain loop rate
        time.sleep(max(0, 1/loop_fps - (time.time() - start_time)))
        # print(1 / (time.time() - start_time))


except KeyboardInterrupt:
    print("Keyboard interrupt caught. Terminating child process.")
finally:
    camera.stop()
    lidar.stop()
    print("Main loop ended. Python script finished.")
