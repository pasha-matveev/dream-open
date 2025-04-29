import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import numpy as np


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    @property
    def list(self):
        return [self.x, self.y]

    def dist(self, other: "Point"):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def angle(self, other: "Point"):
        return np.arctan2(other.y - self.y, other.x - self.x) % (2 * np.pi)
    
    def move(self, vector: "Vector"):
        return Point(self.x + vector.x, self.y + vector.y)

    def draw(self, ax: plt.Axes, marker='o', markersize=2, color='black', **kwargs):
        ax.plot(self.x, self.y, marker=marker,
                markersize=markersize, color=color, **kwargs)


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

    def __add__(self, other: "Vector"):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector"):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other: float):
        return Vector(self.x * other, self.y * other)

    def __truediv__(self, other: float):
        return Vector(self.x / other, self.y / other)

    def dot(self, other: "Vector"):
        return self.x * other.x + self.y * other.y

    def cross(self, other: "Vector"):
        return self.x * other.y - self.y * other.x

    def normal(self, angle: float):
        return Vector.from_angle(angle, np.cos(self.angle - angle) * self.length)

    def tangent(self, angle: float):
        return Vector.from_angle(angle + np.pi / 2, np.sin(self.angle - angle) * self.length)
    
    def collinear(self, other: "Vector"):
        if abs(self.angle - other.angle) % (2 * np.pi) < 0.001:
            return 1
        if abs(self.angle - other.angle + np.pi) % (2 * np.pi) < 0.001:
            return -1
        return 0

    def draw(self, ax: plt.Axes, origin: Point, color='black', linewidth=1, **kwargs):
        ax.arrow(origin.x, origin.y, self.x, self.y,
                 color=color, width=linewidth, **kwargs)

    def __repr__(self):
        return f'Vector({round(self.x, 2)}, {round(self.y, 2)})'


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
    
    def __eq__(self, other: "Segment"):
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

    def draw(self, ax: plt.Axes, color='black', linewidth=2, **kwargs):
        ax.plot([self.p1.x, self.p2.x], [self.p1.y, self.p2.y],
                color=color, linewidth=linewidth, **kwargs)


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

    def draw(self, ax: plt.Axes, fill=False, color='black', linewidth=2, **kwargs):
        circle = plt.Circle((self.center.x, self.center.y), self.radius, fill=fill,
                            color=color, linewidth=linewidth, **kwargs)
        ax.add_patch(circle)


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

    def draw(self, ax: plt.Axes):
        for border in self.borders:
            border.draw(ax)
        for dot in self.dots:
            dot.draw(ax)
        self.center_circle.draw(ax)

        for obstacle in self.obstacles:
            obstacle.draw(ax, color='red', fill=True)

    def draw_heatmap(self, ax: plt.Axes):
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
        ax.imshow(grid, extent=(-91, 91, -121.5, 121.5),
                  cmap='RdYlGn', alpha=0.75, interpolation='gaussian')


class Robot:
    def __init__(self, pos: Point, number: int = 1):
        self.pos = pos
        self.number = number
        self.radius = 9
        self.vel = Vector(0, 0)
        self.acc = Vector(0, 0)

    @property
    def circle(self):
        return Circle(self.pos, self.radius)
    
    def dubins(self, ax: plt.Axes, point: Point, angle: float, radius: float):
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

        arc = Arc(nearest_circle.center.list, radius*2, radius*2, theta1=t1, theta2=t2, color='red', linewidth=2, linestyle='--')
        ax.add_patch(arc)

        return segment

    def update_pos(self, angle: float, dist: float):
        self.pos.x = dist * np.cos(angle)
        self.pos.y = dist * np.sin(angle)
    
    def get_radians(self, angle: float):
        res = -angle / 180 * np.pi - np.pi / 2
        while res < 0:
            res += 2 * np.pi
        return res
    
    def get_degrees(self, angle: float):
        res = angle / np.pi * 180
        return angle / np.pi * 180

    def draw(self, ax: plt.Axes):
        self.circle.draw(ax, color='black', fill=True)
        ax.text(self.pos.x, self.pos.y, str(self.number), color='white', fontsize=10,
                horizontalalignment='center', verticalalignment='center', fontweight='bold')
        self.vel.draw(ax, self.pos)


class Ball:
    def __init__(self, pos: Point):
        self.pos = pos
        self.radius = 2
        self.vel = Vector(0, 0)
        self.acc = Vector(0, 0)

    @property
    def circle(self):
        return Circle(self.pos, self.radius)
    
    def draw(self, ax: plt.Axes):
        self.circle.draw(ax, color='orange', fill=True)


if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(5, 6))
    field = Field()
    robot = Robot(Point(5, 40))
    ball = Ball(Point(-50, -50))

    field.draw(ax)
    robot.draw(ax)
    ball.draw(ax)

    dubins = robot.dubins(ax, ball.pos, np.pi/2, 20)
    dubins.draw(ax, color='red', linestyle='--')

    robot.vel = Vector.from_angle(dubins.angle, 10)

    ax.set_xlim(-91, 91)
    ax.set_ylim(-121.5, 121.5)
    ax.set_aspect('equal')

    plt.show()
# fig, ax = plt.subplots(figsize=(5, 6))
# field = Field()
# robot = Robot(Point(60, -80))
# ball = Ball(Point(10, 10))


# # for i in range(10):
# # kick_vector = Vector.from_angle(np.pi*5/3, 10)
# # dubins = robot.dubins(ax, ball.pos, kick_vector.angle, 20)
# # kick_vector.draw(ax, ball.pos, color='red')
# # robot.vel = Vector.from_angle(dubins.angle, 10)


# inner_k = 1
# outer_k = 2

# # Draw Field
# field.draw_heatmap(ax)
# field.draw(ax)

# # Avoid Field Borders
# robot.vel.draw(ax, robot.pos, color='black')
# nearest_border = field.nearest_border(robot.pos)
# border_point = nearest_border.nearest_point(robot.pos)
# border_vector = Vector.from_points(border_point, robot.pos)
# normal = robot.vel.normal(border_vector.angle)
# tangent = robot.vel.tangent(border_vector.angle)

# if field.is_inside(robot.pos):
#     if border_vector.collinear(normal) == -1:
#         if normal.length > border_vector.length * inner_k:
#             normal.length = border_vector.length * inner_k
#             # nearest_border.draw(ax, color='red', linewidth=2)
    
#     robot.vel = normal + tangent

#     for border in field.borders:
#         if border.is_inside(robot.pos) and border != nearest_border:
#             border_point = border.nearest_point(robot.pos)
#             border_vector = Vector.from_points(border_point, robot.pos)
#             normal = robot.vel.normal(border_vector.angle)
#             tangent = robot.vel.tangent(border_vector.angle)
#             if border_vector.collinear(normal) == -1:
#                 if normal.length > border_vector.length * inner_k:
#                     normal.length = border_vector.length * inner_k
#                     # border.draw(ax, color='yellow', linewidth=2)
#                     robot.vel = normal + tangent
# else:
#     normal.angle = border_vector.angle
#     normal.length = border_vector.length * outer_k
#     # nearest_border.draw(ax, color='red', linewidth=2)

# robot.draw(ax)
# ball.draw(ax)
# robot.vel.draw(ax, robot.pos, color='green')
# # dubins.draw(ax, color='red', linestyle='--')

# ax.set_xlim(-91, 91)
# ax.set_ylim(-121.5, 121.5)
# ax.set_aspect('equal')

# plt.show()