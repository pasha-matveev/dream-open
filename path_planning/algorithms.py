from path_planning.models import *

def draw_arc(plot, center, radius, theta1, theta2, pen=None, samples=60):
    """Quick replacement for Matplotlib's Arc patch."""
    ang = np.radians(np.linspace(theta1, theta2, samples))
    xs  = center.x + radius * np.cos(ang)
    ys  = center.y + radius * np.sin(ang)
    plot.addItem(pg.PlotDataItem(xs, ys, pen=pen or pg.mkPen('r', width=2)))

def dubins_path(plot, robot, point, angle, radius):
    result = Vector(0, 0)
    circles = []
    circles.append(Circle.from_angle(point, angle + np.pi / 2, radius))
    circles.append(Circle.from_angle(point, angle - np.pi / 2, radius))
    nearest_circle = min(circles, key=lambda x: x.dist(robot.pos))

    if nearest_circle.dist(robot.pos) >= 0:
        tagent_segments = nearest_circle.tangent_segments(robot.pos)
        segment = max(tagent_segments, key=lambda x: np.cos(x.p2.angle(point) - angle))
        result = Vector.from_points(robot.pos, segment.p2)
        result.length = robot.pos.dist(point)
        segment.draw(plot, color='r', width=2)
    else:
        result = Vector.from_points(robot.pos, point)
        result.angle += np.pi / 2 * np.sign(np.sin(robot.pos.angle(point) - angle))
        # result = Vector.from_points(robot.pos, point)
    print(np.sin(robot.pos.angle(point) - angle))
    
    circles[0].draw(plot, color='r', width=2)
    circles[1].draw(plot, color='r', width=2)
    nearest_circle.draw(plot, color='g', width=2)

    return result