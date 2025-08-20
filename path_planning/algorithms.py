from path_planning.models import Vector, Circle
import numpy as np


def dubins_path(robot, point, angle, radius, field):
    result = Vector(0, 0)
    circles = []
    circles.append(Circle.from_angle(point, angle + np.pi / 2, radius))
    circles.append(Circle.from_angle(point, angle - np.pi / 2, radius))
    best_circle = min(circles, key=lambda x: x.dist(robot.pos))

    if best_circle.dist(robot.pos) >= 0:
        tagent_segments = best_circle.tangent_segments(robot.pos)
        segment = max(tagent_segments, key=lambda x: np.cos(x.p2.angle(point) - angle))
        # segment.draw(plot, color='r', width=2)
        result = Vector.from_points(robot.pos, segment.p2)
        result.length = robot.pos.dist(point)
        result.length *= 3
        robot.vel = result
    else:
        result = Vector.from_points(robot.pos, best_circle.center)
        result.angle += np.pi / 2 * np.sign(np.sin(robot.pos.angle(point) - angle))
        result.length *= 3
        robot.vel = result
        best_circle.center.attract(robot, 0, best_circle.dist(robot.pos) * 2)
        # result = Vector.from_points(robot.pos, point)

    # circles[0].draw(plot, color='r', width=2)
    # circles[1].draw(plot, color='r', width=2)
    # best_circle.draw(plot, color='g', width=2)

    return result
