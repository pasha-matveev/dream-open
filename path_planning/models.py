"""Geometry, kinematics and soccer‑field model – *no graphics dependencies*.

Replaces the original PyQtGraph‑based draw() helpers with pure data logic so it
can plug into **any** renderer (Pi3D, Arcade, …). All numeric behaviour is
unchanged.
"""
from __future__ import annotations

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import math
import time
from typing import List

import numpy as np

# ----------------------------------------------------------------------------
# Basic 2‑D primitives --------------------------------------------------------
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # ---------- numerics ----------------------------------------------------
    def dist(self, other: "Point") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def angle(self, other: "Point") -> float:
        """Absolute angle *in radians* from this point to *other*."""
        return math.atan2(other.y - self.y, other.x - self.x) % (2 * math.pi)

    def move(self, vec: "Vector") -> "Point":
        return Point(self.x + vec.x, self.y + vec.y)

    # ---------- convenience -------------------------------------------------
    @property
    def as_tuple(self):
        return (self.x, self.y)

    def __iter__(self):
        yield self.x; yield self.y

    # string‑helpers ----------------------------------------------------------
    def __repr__(self):
        return f"Point({self.x:.2f}, {self.y:.2f})"
    
    # ---------- potential field methods --------------------------------------
    def attract(self, robot: Robot, k: float = 1, const: float = 0):
        vector = Vector.from_points(robot.pos, self)
        vector.length = vector.length * k + const
        robot.vel += vector
    
    def constrain(self, robot: Robot, k: float = 1, const: float = 0):
        vector = Vector.from_points(robot.pos, self)
        normal = robot.vel.normal(vector.angle)
        tangent = robot.vel.tangent(vector.angle)
        if normal.collinear(vector) == 1:
            normal.length = min(normal.length, k * vector.length + const)
            robot.vel = normal + tangent

    def draw(self, plot: pg.PlotItem, symbol='o', size=5, color='k', **kw):
        item = pg.ScatterPlotItem([self.x], [self.y],
                                  symbol=symbol, size=size,
                                  brush=pg.mkBrush(color), pen=pg.mkPen(None))
        plot.addItem(item)


class Vector:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    # ------------ factory helpers ------------------------------------------
    @classmethod
    def from_points(cls, p1: Point, p2: Point):
        return cls(p2.x - p1.x, p2.y - p1.y)

    @classmethod
    def from_angle(cls, ang: float, length: float):
        return cls(length * math.cos(ang), length * math.sin(ang))

    # ------------ polar getters/setters ------------------------------------
    @property
    def length(self) -> float:
        return math.hypot(self.x, self.y)

    @length.setter
    def length(self, value: float):
        ang = self.angle
        self.x = value * math.cos(ang)
        self.y = value * math.sin(ang)

    @property
    def angle(self) -> float:
        return math.atan2(self.y, self.x) % (2 * math.pi)

    @angle.setter
    def angle(self, ang: float):
        l = self.length
        self.x = l * math.cos(ang)
        self.y = l * math.sin(ang)

    # ------------ vector algebra -------------------------------------------
    def __add__(self, o: "Vector") -> "Vector":
        return Vector(self.x + o.x, self.y + o.y)

    def __sub__(self, o: "Vector") -> "Vector":
        return Vector(self.x - o.x, self.y - o.y)

    def __mul__(self, k: float):
        return Vector(self.x * k, self.y * k)

    __rmul__ = __mul__

    # dot / cross -------------------------------------------------------------
    def dot(self, o: "Vector") -> float:
        return self.x * o.x + self.y * o.y

    def cross(self, o: "Vector") -> float:
        return self.x * o.y - self.y * o.x

    # normal & tangent relative to an axis -----------------------------------
    def normal(self, axis_ang: float) -> "Vector":
        """Component along *axis_ang* (projection)."""
        return Vector.from_angle(axis_ang, math.cos(self.angle - axis_ang) * self.length)

    def tangent(self, axis_ang: float) -> "Vector":
        return Vector.from_angle(axis_ang + math.pi / 2, math.sin(self.angle - axis_ang) * self.length)

    # collinearity test -------------------------------------------------------
    def collinear(self, other: "Vector") -> int:
        d_ang = abs(self.angle - other.angle) % (2 * math.pi)
        if d_ang < 1e-3:
            return 1
        if abs(d_ang - math.pi) < 1e-3:
            return -1
        return 0

    # ------------------------------------------------------------------------
    def __repr__(self):
        return f"Vector({self.x:.2f}, {self.y:.2f})"
    

    def draw(self, plot: pg.PlotItem, origin: Point,
             color='k', width=4, head_len=12, length_scale=1, **kw):
        end_x, end_y = origin.x + self.x * length_scale, origin.y + self.y * length_scale
        pen  = pg.mkPen(color, width=width)
        plot.addItem(pg.PlotDataItem([origin.x, end_x],
                                     [origin.y, end_y], pen=pen))

        arrow = pg.ArrowItem(angle=180 - np.degrees(self.angle),
                             tipAngle=30, headLen=head_len,
                             brush=pg.mkBrush(color))
        arrow.setPos(end_x, end_y)
        plot.addItem(arrow)


# ----------------------------------------------------------------------------
# Higher‑level primitives -----------------------------------------------------
class Segment:
    def __init__(self, p1: Point, p2: Point):
        self.p1, self.p2 = p1, p2

    @classmethod
    def from_tuple(cls, p1, p2):
        return cls(Point(*p1), Point(*p2))

    # geometry helpers -------------------------------------------------------
    @property
    def length(self):
        return self.p1.dist(self.p2)

    @property
    def angle(self):
        return self.p1.angle(self.p2)

    def is_inside(self, point: Point) -> bool:
        if Vector.from_points(point, self.p1).dot(Vector.from_points(self.p2, self.p1)) > 0:
            if Vector.from_points(point, self.p2).dot(Vector.from_points(self.p1, self.p2)) > 0:
                return True
        return False

    def nearest_point(self, point: Point) -> Point:
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
    
    def attract(self, robot: Robot, f: float = 1, const: float = 0):
        point = self.nearest_point(robot.pos)
        point.attract(robot, f, const)
    
    def constrain(self, robot: Robot, f: float = 1, const: float = 0):
        point = self.nearest_point(robot.pos)
        point.constrain(robot, f, const)

    # distance ---------------------------------------------------------------
    def dist(self, p: Point):
        return p.dist(self.nearest_point(p))
    
    def draw(self, plot: pg.PlotItem, color='k', width=3, **kw):
        pen = pg.mkPen(color, width=width)
        plot.addItem(pg.PlotDataItem([self.p1.x, self.p2.x],
                                     [self.p1.y, self.p2.y],
                                     pen=pen))


class Circle:
    def __init__(self, center: Point, radius: float):
        self.center, self.radius = center, radius

    def dist(self, p: Point):
        return self.center.dist(p) - self.radius
    
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


# ----------------------------------------------------------------------------
# World description (soccer field) -------------------------------------------
class Field:
    """Official RoboCup Junior Soccer field (2024 rules). Units: *centimetres*."""
    def __init__(self):
        # 12‑sided border (cut corners)
        b = [(-73, -103.5), (-55.4, -103.5), (-34, -82.1), (34, -82.1), (55.4, -103.5),
             (73, -103.5), (73, 103.5), (55.4, 103.5), (34, 82.1), (-34, 82.1),
             (-55.4, 103.5), (-73, 103.5)]
        self.borders: List[Segment] = [Segment.from_tuple(b[i], b[(i+1) % len(b)]) for i in range(len(b))]

        self.dots   = [Point(-39, -64.5), Point(39, -64.5), Point(-39, 64.5), Point(39, 64.5), Point(0, 0)]
        self.center_circle = Circle(Point(0, 0), 30)
        self.obstacles = [Circle(Point(0, 64.5), 9)]      # e.g. goal area posts
        self.size = [182, 243]
        outer = [(-91, -121.5), (-91, 121.5), (91, 121.5), (91, -121.5)]
        self.outer: List[Segment] = [Segment.from_tuple(outer[i], outer[(i+1) % len(outer)]) for i in range(len(outer))]

    # ---------------- point‑in‑polygon via ray casting ----------------------
    def is_inside(self, p: Point) -> bool:
        crossings = 0
        for s in self.borders:
            cond = (p.y < s.p1.y) != (p.y < s.p2.y)
            if cond:
                x_int = s.p1.x + (p.y - s.p1.y) / (s.p2.y - s.p1.y) * (s.p2.x - s.p1.x)
                if p.x < x_int:
                    crossings += 1
        return crossings % 2 == 1

    # ---------------- nearest border utilities -----------------------------
    def nearest_border(self, p: Point) -> Segment:
        return min(self.borders, key=lambda s: s.dist(p))

    def border_dist(self, p: Point) -> float:
        return self.nearest_border(p).dist(p)
    
    def nearest_border_point(self, point: Point):
        return self.nearest_border(point).nearest_point(point)
    
    def draw(self, plot: pg.PlotItem):
        for border in self.borders:
            border.draw(plot)
        for outer in self.outer:
            outer.draw(plot)
        for dot in self.dots:
            dot.draw(plot)
        self.center_circle.draw(plot)

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


# ----------------------------------------------------------------------------
# Field Objects --------------------------------------------------------------
class FieldObject:
    def __init__(self, pos: Point, radius: float):
        self.pos = pos
        self.radius = radius
        self.vel = Vector(0, 0)
        self.lst_vel = Vector(0, 0)
        self.update_tm = 0
        self.visible_tm = 0
    
    @property
    def circle(self):
        return Circle(self.pos, self.radius)
    
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
        res = -angle / np.pi * 180 + 90
        while res < 0:
            res += 360
        return res
    
    def attract(self, robot: Robot, k: float = 1, const: float = 0):
        self.pos.attract(robot, k, const)
    
    def constrain(self, robot: Robot, k: float = 1, const: float = 0):
        self.pos.constrain(robot, k, const)
    

class Robot(FieldObject):
    def __init__(self, pos: Point):
        super().__init__(pos, 9)
        self.gyro = 0
        self.emitter = False
        self.kicker = False

        self.max_speed = 200  # [cm / s]
        self.max_acc = 200 # [cm / s^2]

    def update_state(self, uart_data):
        self.gyro = uart_data[3]
        self.emitter = uart_data[4]
        self.kicker = uart_data[5]

    def update_pos(self, angle: float, dist: float):
        self.pos.x = dist * np.cos(self.get_radians(angle + self.gyro) + np.pi)
        self.pos.y = dist * np.sin(self.get_radians(angle + self.gyro) + np.pi)
        self.update_tm = time.time()
        self.visible_tm = time.time()
    
    def limit_speed(self, max_speed: float|None = None):
        if max_speed is None:
            max_speed = self.max_speed
        self.vel.length = min(self.vel.length, max_speed)
        
    def limit_acc(self, fps:int, max_acc: float|None = None):
        if max_acc is None:
            max_acc = self.max_acc
        diff = self.vel - self.lst_vel
        diff.length = min(max_acc * 1/fps, diff.length)
        self.vel = self.lst_vel + diff
        self.lst_vel = self.vel

    def draw(self, plot: pg.PlotItem):
        self.circle.draw(plot, color='black', fill=True)


class Ball(FieldObject):
    def __init__(self, pos: Point):
        super().__init__(pos, 2)

    def update_pos(self, robot, camera_ball):
        if camera_ball.visible:
            self.pos.x = camera_ball.dist * np.cos(self.get_radians(camera_ball.angle + robot.gyro)) + robot.pos.x
            self.pos.y = camera_ball.dist * np.sin(self.get_radians(camera_ball.angle + robot.gyro)) + robot.pos.y
            self.update_tm = time.time()
            self.visible_tm = time.time()

    def draw(self, plot: pg.PlotItem):
        self.circle.draw(plot, color='orange', fill=True)

if __name__ == "__main__":
    import sys, numpy as np, time
    pg.setConfigOption('background', 'w')          # white canvas
    pg.setConfigOption('foreground', 'k')          # black axes / text
    pg.setConfigOption('antialias', False)          # prettier curves
    app = pg.mkQApp("Robot visualiser")            # cross-binding helper

    # 2. Window / plot item ----------------------------------------------------------------
    win  = pg.GraphicsLayoutWidget(title="Robot visualiser"); win.show()
    plot: pg.PlotItem = win.addPlot()
    plot.setAspectLocked(True)                     # keep x & y in the same scale
    plot.setXRange(-91, 91)                        # matches your figure
    plot.setYRange(-121.5, 121.5)

    field = Field()
    robot = Robot(Point(85, -60))
    ball = Ball(Point(10, 10))

    # robot.vel = Vector(20, 20)

    field.draw(plot)
    robot.draw(plot)

    robot.vel.draw(plot, robot.pos, color='black')
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

    ball.draw(plot)
    robot.vel.draw(plot, robot.pos, color='green')

    if sys.flags.interactive != 1:
        QtWidgets.QApplication.instance().exec()
