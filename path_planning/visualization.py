from __future__ import annotations
import importlib.util

import time
from pathlib import Path

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# ─────────────────────────── reuse the model layer ──────────────────────────
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("models", HERE / "models.py")
models = importlib.util.module_from_spec(spec)  # type: ignore
spec.loader.exec_module(models)  # type: ignore

Point = models.Point
Vector = models.Vector
Field = models.Field
Robot = models.Robot
Ball = models.Ball
Obstacle = models.Obstacle
Goal = models.Goal

# ────────────────────────── simulation constants ────────────────────────────
INNER_K = 1
OUTER_K = 2
MAX_SPEED = 200  # [cm / s]

# ─────────────────────── border-avoidance helper ────────────────────────────


def compute_robot_velocity(field: Field, rb: Robot) -> Vector:
    """Same logic you had in test.py – untouched."""
    nearest = field.nearest_border(rb.pos)
    border_pt = nearest.nearest_point(rb.pos)
    border_vec = Vector.from_points(rb.pos, border_pt)

    normal = rb.vel.normal(border_vec.angle)
    tangent = rb.vel.tangent(border_vec.angle)

    if field.is_inside(rb.pos):
        if (
            border_vec.collinear(normal) == 1
            and normal.length > border_vec.length * INNER_K
        ):
            normal.length = border_vec.length * INNER_K
        new_v = normal + tangent

        for border in field.borders:
            if border.is_inside(rb.pos) and border is not nearest:
                border_pt = border.nearest_point(rb.pos)
                border_vec = Vector.from_points(rb.pos, border_pt)
                normal = new_v.normal(border_vec.angle)
                tangent = new_v.tangent(border_vec.angle)
                if (
                    border_vec.collinear(normal) == 1
                    and normal.length > border_vec.length * INNER_K
                ):
                    normal.length = border_vec.length * INNER_K
                    new_v = normal + tangent
    else:
        normal.angle = border_vec.angle
        normal.length = max(border_vec.length * OUTER_K, normal.length)
        new_v = normal
        print("outside")

    # if new_v.length > 0:
    #     new_v.length = MAX_SPEED
    return new_v


# ────────────────────────── Simulation wrapper ──────────────────────────────


class Visualization:
    """
    Same public API as the old Matplotlib version but backed by PyQtGraph
    for ~10-20× faster redraws.
    """

    def __init__(
        self,
        field: Field,
        ball: Ball,
        robot: Robot,
        obstacles: list[Obstacle] = [],
        front_goal: Goal | None = None,
        back_goal: Goal | None = None,
        fps: int = 30,
    ):
        self.field = field
        self.ball = ball
        self.robot = robot
        self.obstacles = obstacles
        self.front_goal = front_goal
        self.back_goal = back_goal

        self.fps = fps
        self.lst_update_tm = 0
        self.started = False

    def start(self):
        # ---------- Qt application & plot ----------------------------------
        pg.setConfigOption("background", "w")
        pg.setConfigOption("foreground", "k")
        pg.setConfigOption("antialias", True)

        self.app = pg.mkQApp("Robot visualiser")

        self.win = pg.GraphicsLayoutWidget(title="Robot trajectory")
        self.view: pg.PlotItem = self.win.addPlot()  # pyright: ignore
        self._configure_view()

        # connect click-to-teleport
        self.view.scene().sigMouseClicked.connect(self._on_click)  # pyright: ignore

        self.win.show()
        self.started = True

    # ──────────────────────── interactive hook ────────────────────────────
    def _on_click(self, ev):
        if ev.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        mouse_pt = self.view.vb.mapSceneToView(ev.scenePos())  # pyright: ignore
        self.ball.pos = Point(mouse_pt.x(), mouse_pt.y())

    # ───────────────────── view helpers (axes, limits…) ────────────────────
    def _configure_view(self):
        self.view.setAspectLocked(True)  # pyright: ignore
        self.view.setXRange(-91, 91)  # pyright: ignore
        self.view.setYRange(-121.5, 121.5)  # pyright: ignore
        self.view.setLabel("bottom", "x [cm]")
        self.view.setLabel("left", "y [cm]")

        if not hasattr(self, "label"):  # create only once
            self.label = pg.LabelItem(justify="right")
            self.win.addItem(self.label, row=1, col=0)  # pyright: ignore

            # # live coordinate read-out (bottom-right)
            def _mouse_move(pos):
                if self.view.sceneBoundingRect().contains(pos):
                    p = self.view.vb.mapSceneToView(pos)  # pyright: ignore
                    self.label.setText(f"(x, y) = ({p.x():.1f}, {p.y():.1f})")

            self.view.scene().sigMouseMoved.connect(_mouse_move)  # pyright: ignore

    # ───────────────────────── drawing helpers ─────────────────────────────
    def _clear(self):
        self.view.clear()
        # keep axis limits/aspect after clear
        # self._configure_view()

    def _draw(self):
        # self.field.draw_heatmap(self.view)   # enable if you like
        self.field.draw(self.view)
        self.ball.draw(self.view)
        self.robot.draw(self.view)
        self.robot.vel.draw(self.view, self.robot.pos, color="green")
        self.front_goal.draw(self.view)  # pyright: ignore
        self.back_goal.draw(self.view)  # pyright: ignore
        for obstacle in self.obstacles:
            obstacle.draw(self.view)
        # self.robot.vel.draw(self.view, self.robot.pos, color='green')
        # Qt redraws automatically; no explicit canvas.draw_idle() needed

    # ───────────────────────── public API ──────────────────────────────────
    @property
    def is_alive(self) -> bool:
        if not self.started:
            return False
        return self.win.isVisible()

    @property
    def update_tm(self):
        return 1 / self.fps <= time.time() - self.lst_update_tm

    def step(self):
        if self.is_alive:
            self._clear()
            self._draw()
            self._update()
        self.lst_update_tm = time.time()

    def _update(self):
        self.app.processEvents()  # let Qt flush pending paints


# ─────────────────────────── standalone demo ───────────────────────────────
if __name__ == "__main__":
    field = Field()
    robot = Robot(Point(85, -80))
    ball = Ball(Point(10, 10))
    vis = Visualization(field, ball, robot)
    vis.start()
    while vis.is_alive:
        vis.step()
        time.sleep(1 / 30)
