# OLD
# import matplotlib.pyplot as plt
# from matplotlib.patches import Arc
from __future__ import annotations

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import numpy as np
import time

def draw_arc(plot, center, radius, theta1, theta2, pen=None, samples=60):
    """Quick replacement for Matplotlib's Arc patch."""
    ang = np.radians(np.linspace(theta1, theta2, samples))
    xs  = center.x + radius * np.cos(ang)
    ys  = center.y + radius * np.sin(ang)
    plot.addItem(pg.PlotDataItem(xs, ys, pen=pen or pg.mkPen('r', width=2)))


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    @property
    def list(self):
        return [self.x, self.y]

    def dist(self, other: Point):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def angle(self, other: Point):
        return np.arctan2(other.y - self.y, other.x - self.x) % (2 * np.pi)
    
    def move(self, vector: Vector):
        return Point(self.x + vector.x, self.y + vector.y)
        
    def attract(self, robot: Robot, min_dist: float = 0, max_dist: float = 1000, min_speed: float = 0, max_speed: float = 1000):
        speed = 0
        if self.dist(robot.pos) < min_dist:
            speed = min_speed
        elif self.dist(robot.pos) > max_dist:
            speed = max_speed
        else:
            speed = (self.dist(robot.pos) - min_dist) / (max_dist - min_dist) * (max_speed - min_speed) + min_speed
        robot.vel += Vector.from_angle(robot.pos.angle(self), speed)
    
    def constrain(self, robot: Robot, min_dist: float = 0, max_dist: float = 1000, min_speed: float = 0, max_speed: float = 1000):
        vector = Vector.from_points(robot.pos, self)
        normal = robot.vel.normal(vector.angle)
        tangent = robot.vel.tangent(vector.angle)
        if normal.collinear(vector) == 1:
            if normal.length < min_dist:
                normal.length = min_dist
            elif normal.length > max_dist:
                normal.length = max_dist
            else:
                normal.length = (normal.length - min_dist) / (max_dist - min_dist) * (max_speed - min_speed) + min_speed
            robot.vel = normal + tangent

    def draw(self, plot: pg.PlotItem, symbol='o', size=5, color='k', **kw):
        item = pg.ScatterPlotItem([self.x], [self.y],
                                  symbol=symbol, size=size,
                                  brush=pg.mkBrush(color), pen=pg.mkPen(None))
        plot.addItem(item)
    
    def __repr__(self):
        return f'Point({round(self.x, 2)}, {round(self.y, 2)})'

class Vector:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.origin = None

    @classmethod
    def from_points(cls, p1: Point, p2: Point):
        return cls(p2.x - p1.x, p2.y - p1.y)

    @classmethod
    def from_angle(cls, angle: float, length: float):
        return cls(length * np.cos(angle), length * np.sin(angle))

    @property
    def length(self):
        return np.sqrt(self.x**2 + self.y**2)

    @length.setter
    def length(self, value: float):
        angle = self.angle
        self.x = value * np.cos(angle)
        self.y = value * np.sin(angle)

    @property
    def angle(self):
        return np.arctan2(self.y, self.x) % (2 * np.pi)

    @angle.setter
    def angle(self, value: float):
        angle = value % (2 * np.pi)
        length = self.length
        self.x = length * np.cos(angle)
        self.y = length * np.sin(angle)

    def __add__(self, other: Vector):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other: float):
        return Vector(self.x * other, self.y * other)

    def __truediv__(self, other: float):
        return Vector(self.x / other, self.y / other)

    def dot(self, other: Vector):
        return self.x * other.x + self.y * other.y

    def cross(self, other: Vector):
        return self.x * other.y - self.y * other.x

    def normal(self, angle: float):
        return Vector.from_angle(angle, np.cos(self.angle - angle) * self.length)

    def tangent(self, angle: float):
        return Vector.from_angle(angle + np.pi / 2, np.sin(self.angle - angle) * self.length)
    
    def collinear(self, other: Vector):
        if abs(self.angle - other.angle) % (2 * np.pi) < 0.001:
            return 1
        if abs(self.angle - other.angle + np.pi) % (2 * np.pi) < 0.001:
            return -1
        return 0

    def __repr__(self):
        return f'Vector({round(self.x, 2)}, {round(self.y, 2)})'
    
    def draw(self, plot: pg.PlotItem, origin: Point,
             color='k', width=4, head_len=12, length_scale=1, **kw):
        end_x, end_y = origin.x + self.x * length_scale, origin.y + self.y * length_scale
        pen  = pg.mkPen(color, width=width)
        plot.addItem(pg.PlotDataItem([origin.x, end_x],
                                     [origin.y, end_y], pen=pen))

        # arrow = pg.ArrowItem(angle=np.degrees(self.angle),
        #                      tipAngle=30, headLen=head_len,
        #                      brush=pg.mkBrush(color))
        arrow = pg.ArrowItem(angle=180 - np.degrees(self.angle),
                             tipAngle=30, headLen=head_len,
                             brush=pg.mkBrush(color))
        arrow.setPos(end_x, end_y)
        plot.addItem(arrow)


class Segment:
    def __init__(self, p1: Point, p2: Point):
        self.p1 = p1
        self.p2 = p2

    @classmethod
    def from_list(cls, p1: list, p2: list):
        return cls(Point(p1[0], p1[1]), Point(p2[0], p2[1]))

    @property
    def length(self):
        return self.p1.dist(self.p2)

    @property
    def angle(self):
        return np.arctan2(self.p2.y - self.p1.y, self.p2.x - self.p1.x) % (2 * np.pi)
    
    @property
    def vector(self):
        return Vector.from_points(self.p1, self.p2)
    
    def __eq__(self, other: Segment):
        return self.p1 == other.p1 and self.p2 == other.p2
    
    def is_inside(self, point):
        if Vector.from_points(point, self.p1).dot(Vector.from_points(self.p2, self.p1)) > 0:
            if Vector.from_points(point, self.p2).dot(Vector.from_points(self.p1, self.p2)) > 0:
                return True
        return False

    def dist(self, point):
        if self.is_inside(point):
            return abs(Vector.from_points(point, self.p1).cross(Vector.from_points(self.p2, self.p1))) / self.length
        return min(point.dist(self.p1), point.dist(self.p2))

    def nearest_point(self, point):
        if self.is_inside(point):
            if self.p1.x == self.p2.x:
                return Point(self.p1.x, point.y)
            if self.p1.y == self.p2.y:
                return Point(point.x, self.p1.y)
            k = (self.p2.y - self.p1.y) / (self.p2.x - self.p1.x)
            x1 = (k**2 * self.p1.x - k * self.p1.y +
                    k * point.y + point.x) / (k**2 + 1)
            y1 = k * (x1 - self.p1.x) + self.p1.y
            return Point(x1, y1)
        return self.p1 if point.dist(self.p1) < point.dist(self.p2) else self.p2

    def attract(self, robot: Robot, min_dist: float = 0, max_dist: float = 1000, min_speed: float = 0, max_speed: float = 1000):
        point = self.nearest_point(robot.pos)
        speed = 0
        if point.dist(robot.pos) < min_dist:
            speed = min_speed
        elif point.dist(robot.pos) > max_dist:
            speed = max_speed
        else:
            speed = (point.dist(robot.pos) - min_dist) / (max_dist - min_dist) * (max_speed - min_speed) + min_speed
        robot.vel += Vector.from_angle(robot.pos.angle(point), speed)
    
    def constrain(self, robot: Robot, min_dist: float = 0, max_dist: float = 1000, min_speed: float = 0, max_speed: float = 1000):
        point = self.nearest_point(robot.pos)

    def draw(self, plot: pg.PlotItem, color='k', width=3, **kw):
        pen = pg.mkPen(color, width=width)
        plot.addItem(pg.PlotDataItem([self.p1.x, self.p2.x],
                                     [self.p1.y, self.p2.y],
                                     pen=pen))


class Circle:
    def __init__(self, center: Point, radius: float):
        self.center = center
        self.radius = radius
    
    @classmethod
    def from_angle(cls, point: Point, angle: float, radius: float):
        return cls(Point(point.x + radius * np.cos(angle), point.y + radius * np.sin(angle)), radius)

    def dist(self, point):
        return self.center.dist(point) - self.radius

    def angle(self, point):
        return self.center.angle(point)

    def nearest_point(self, point):
        dist = self.dist(point)
        angle = self.angle(point)
        return Point(dist * np.cos(angle) + point.x, dist * np.sin(angle) + point.y)
    
    def tangent_segments(self, point: Point):
        a1 = point.angle(self.center)
        a2 = np.asin(self.radius / point.dist(self.center))
        l = np.sqrt(point.dist(self.center)**2 - self.radius**2)
        s1 = Segment.from_list([point.x, point.y], [point.x + np.cos(a1 + a2) * l, point.y + np.sin(a1 + a2) * l])
        s2 = Segment.from_list([point.x, point.y], [point.x + np.cos(a1 - a2) * l, point.y + np.sin(a1 - a2) * l])
        return [s1, s2]

    def draw(self, plot: pg.PlotItem, fill=False,
             color='k', width=3, **kw):
        pen   = pg.mkPen(color, width=width)
        brush = pg.mkBrush(color) if fill else pg.mkBrush(None)

        item = QtWidgets.QGraphicsEllipseItem(-self.radius, -self.radius,
                                          2*self.radius, 2*self.radius)
        item.setPen(pen); item.setBrush(brush)
        item.setPos(self.center.x, self.center.y)
        plot.addItem(item)


class Field:
    def __init__(self):
        self.borders = [
            Segment.from_list((-78, -108.5), (-55.4, -108.5)),
            Segment.from_list((-55.4, -108.5), (-29, -82.1)),
            Segment.from_list((-29, -82.1), (29, -82.1)),
            Segment.from_list((29, -82.1), (55.4, -108.5)),
            Segment.from_list((55.4, -108.5), (78, -108.5)),
            Segment.from_list((78, -108.5), (78, 108.5)),
            Segment.from_list((78, 108.5), (55.4, 108.5)),
            Segment.from_list((55.4, 108.5), (29, 82.1)),
            Segment.from_list((29, 82.1), (-29, 82.1)),
            Segment.from_list((-29, 82.1), (-55.4, 108.5)),
            Segment.from_list((-55.4, 108.5), (-78, 108.5)),
            Segment.from_list((-78, 108.5), (-78, -108.5)),
        ]

        self.dots = [
            Point(-39, -64.5),
            Point(39, -64.5),
            Point(-39, 64.5),
            Point(39, 64.5),
            Point(0, 0),
        ]

        self.center_circle = Circle(Point(0, 0), 30)

        self.obstacles = [
            Circle(Point(0, 64.5), 9),
        ]

    def is_inside(self, point: Point):
        cnt = 0
        for border in self.borders:
            if (point.y < border.p1.y) != (point.y < border.p2.y):
                if point.x < border.p1.x + (point.y - border.p1.y) / (border.p2.y - border.p1.y) * (border.p2.x - border.p1.x):
                    cnt += 1
        return cnt % 2 == 1

    def nearest_border(self, point: Point):
        return min(self.borders, key=lambda x: x.dist(point))

    def border_dist(self, point: Point):
        return self.nearest_border(point).dist(point)

    def nearest_border_point(self, point: Point):
        return self.nearest_border(point).nearest_point(point)

    def draw(self, plot: pg.PlotItem):
        for border in self.borders:
            border.draw(plot)
        for dot in self.dots:
            dot.draw(plot)
        self.center_circle.draw(plot)

        # for obstacle in self.obstacles:
        #     obstacle.draw(plot, color='red', fill=True)

    def draw_heatmap(self, plot: pg.PlotItem):
        grid = np.zeros((61, 46))
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                point = Point((j - grid.shape[1] / 2 + 0.5) * (
                    182 / grid.shape[1]), (i - grid.shape[0] / 2 + 0.5) * (243 / grid.shape[0]))
                if self.is_inside(point):
                    dist = self.border_dist(point)
                    dist = dist if dist < 20 else 20
                else:
                    dist = self.border_dist(point) * -2
                    dist = dist if dist > -20 else -20
                grid[i, j] = dist
        img = pg.ImageItem(grid.T)                                  # transpose = correct orientation
        lut = pg.colormap.get('RdYlGn', source='matplotlib').getLookupTable(0.0,1.0,256)
        img.setLookupTable(lut); img.setOpacity(0.75)
        img.setRect(pg.QtCore.QRectF(-91, -121.5, 182, 243))
        plot.addItem(img)


class Robot:
    def __init__(self, pos: Point, number: int = 1):
        self.pos = pos
        self.number = number
        self.radius = 9
        self.vel = Vector(0, 0)
        self.ang_vel = 0
        self.visible = False
        self.update_tm = 0

        self.gyro = 0
        self.emitter = False
        self.kicker = False

    @property
    def circle(self):
        return Circle(self.pos, self.radius)
    
    def dubins(self, plot: pg.PlotItem, point: Point, angle: float, radius: float):
        circles = []
        circles.append(Circle.from_angle(point, angle + np.pi / 2, radius))
        circles.append(Circle.from_angle(point, angle - np.pi / 2, radius))
        nearest_circle = min(circles, key=lambda x: x.dist(self.pos))
        tagent_segments = nearest_circle.tangent_segments(self.pos)
        segment = max(tagent_segments, key=lambda x: np.cos(x.p2.angle(point) - angle))

        if np.sin(segment.p2.angle(point) - angle) < 0:
            t1 = np.degrees(nearest_circle.center.angle(segment.p2))
            t2 = np.degrees(nearest_circle.center.angle(point))
        else:
            t2 = np.degrees(nearest_circle.center.angle(segment.p2))
            t1 = np.degrees(nearest_circle.center.angle(point))

        print(np.degrees(angle), t1, t2)
        print(np.sin(segment.p2.angle(point) - angle))

        arc_pen = pg.mkPen('r', width=2, style=QtCore.Qt.DashLine)
        draw_arc(plot, nearest_circle.center, radius, t1, t2, pen=arc_pen)

        return segment
    
    def update_state(self, uart_data):
        self.gyro = uart_data[3]
        self.emitter = uart_data[4]
        self.kicker = uart_data[5]

    def update_pos(self, angle: float, dist: float):
        self.pos.x = dist * np.cos(self.get_radians(angle + self.gyro) + np.pi)
        self.pos.y = dist * np.sin(self.get_radians(angle + self.gyro) + np.pi)
        self.update_tm = time.time()
    
    def predict_pos(self):
        self.pos.x = self.pos.x + self.vel.x * (time.time() - self.update_tm)
        self.pos.y = self.pos.y + self.vel.y * (time.time() - self.update_tm)
        self.update_tm = time.time()
    
    def get_radians(self, angle: float):
        res = -angle / 180 * np.pi + np.pi / 2
        while res < 0:
            res += 2 * np.pi
        return res
    
    def get_degrees(self, angle: float):
        res = angle / np.pi * 180
        return angle / np.pi * 180

    def draw(self, plot: pg.PlotItem):
        self.circle.draw(plot, color='black', fill=True)


class Ball:
    def __init__(self, pos: Point):
        self.pos = pos
        self.radius = 2
        self.vel = Vector(0, 0)
        # self.visible = False
        self.update_tm = 0

    @property
    def circle(self):
        return Circle(self.pos, self.radius)
    
    @property
    def visible(self):
        return (time.time() - self.update_tm) < 0.5
    
    def update_pos(self, robot, camera_ball):
        if camera_ball.visible:
            self.pos.x = camera_ball.dist * np.cos(self.get_radians(camera_ball.angle + robot.gyro)) + robot.pos.x
            self.pos.y = camera_ball.dist * np.sin(self.get_radians(camera_ball.angle + robot.gyro)) + robot.pos.y
            self.update_tm = time.time()
    
    def predict_pos(self):
        self.pos.x = self.pox.x + self.vel.x * (time.time() - self.update_tm)
        self.pos.y = self.pox.y + self.vel.y * (time.time() - self.update_tm)
        self.update_tm = time.time()
    
    def get_radians(self, angle: float):
        res = -angle / 180 * np.pi + np.pi / 2
        while res < 0:
            res += 2 * np.pi
        return res
    
    def get_degrees(self, angle: float):
        res = angle / np.pi * 180
        return angle / np.pi * 180
    
    def draw(self, plot: pg.PlotItem):
        self.circle.draw(plot, color='orange', fill=True)


class Obstacle:
    def __init__(self, pos: Point, radius: float):
        self.pos = pos
        self.radius = radius

    def draw(self, plot: pg.PlotItem):
        Circle(self.pos, self.radius).draw(plot, color='red', fill=True)


if __name__ == "__main__":
    import sys, numpy as np, time
    pg.setConfigOption('background', 'w')          # white canvas
    pg.setConfigOption('foreground', 'k')          # black axes / text
    pg.setConfigOption('antialias', True)          # prettier curves
    app = pg.mkQApp("Robot visualiser")            # cross-binding helper

    # 2. Window / plot item ----------------------------------------------------------------
    win  = pg.GraphicsLayoutWidget(title="Robot visualiser"); win.show()
    plot: pg.PlotItem = win.addPlot()
    plot.setAspectLocked(True)                     # keep x & y in the same scale
    plot.setXRange(-91, 91)                        # matches your figure
    plot.setYRange(-121.5, 121.5)

    plot.invertY(False)                            # positive Y up (default)
    plot.setMouseEnabled(x=False, y=False)         # lock panning / zoom if desired

    # Turn off the automatic margins so lines hug the edge like MPL’s tight-layout
    plot.vb.disableAutoRange()

    # Show axes ticks & numbers (default), but kill  the top/right axes
    plot.hideAxis('right'); plot.hideAxis('top')

    pen_field = pg.mkPen(width=5, color='k')

    # 3.1 outer border (rectangle)
    outer = np.array([[-91,-121.5], [-91,121.5], [91,121.5], [91,-121.5], [-91,-121.5]])
    plot.addItem(pg.PlotDataItem(outer[:,0], outer[:,1], pen=pen_field))

    # 4. Live mouse coordinate read-out (bottom-right) --------------------------
    label = pg.LabelItem(justify='right')
    win.addItem(label, row=1, col=0)

    def _update_mouse(evt):
        vb = plot.vb
        if vb.sceneBoundingRect().contains(evt):
            mouse_point = vb.mapSceneToView(evt)
            label.setText(f"(x, y) = ({mouse_point.x():.1f}, {mouse_point.y():.1f})")

    plot.scene().sigMouseMoved.connect(_update_mouse)


    field = Field()
    robot = Robot(Point(65, -60))
    ball = Ball(Point(10, 10))

    robot.vel = Vector(20, 20)

    inner_k = 1
    outer_k = 2

    # Draw Field
    # field.draw_heatmap(plot)
    field.draw(plot)
    robot.draw(plot)

    # Avoid Field Borders
    robot.vel.draw(plot, robot.pos, color='black')
    nearest_border = field.nearest_border(robot.pos)
    border_point = nearest_border.nearest_point(robot.pos)
    border_vector = Vector.from_points(border_point, robot.pos)
    normal = robot.vel.normal(border_vector.angle)
    tangent = robot.vel.tangent(border_vector.angle)

    if field.is_inside(robot.pos):
        if border_vector.collinear(normal) == -1:
            if normal.length > border_vector.length * inner_k:
                normal.length = border_vector.length * inner_k
        
        robot.vel = normal + tangent

        for border in field.borders:
            if border.is_inside(robot.pos) and border != nearest_border:
                border_point = border.nearest_point(robot.pos)
                border_vector = Vector.from_points(border_point, robot.pos)
                normal = robot.vel.normal(border_vector.angle)
                tangent = robot.vel.tangent(border_vector.angle)
                if border_vector.collinear(normal) == -1:
                    if normal.length > border_vector.length * inner_k:
                        normal.length = border_vector.length * inner_k
                        # border.draw(ax, color='yellow', linewidth=2)
                        robot.vel = normal + tangent
    else:

        normal.angle = border_vector.angle + np.pi
        normal.length = border_vector.length * outer_k
        # nearest_border.draw(ax, color='red', linewidth=2)

        robot.vel = normal + tangent

    ball.draw(plot)
    robot.vel.draw(plot, robot.pos, color='green')
    # dubins.draw(ax, color='red', linestyle='--')

    # 60 Hz GUI repaints
    timer = QtCore.QTimer(); timer.start(16)
    timer.timeout.connect(lambda: None)   # pure tick; scene items already hold state

    if sys.flags.interactive != 1:
        QtWidgets.QApplication.instance().exec()
