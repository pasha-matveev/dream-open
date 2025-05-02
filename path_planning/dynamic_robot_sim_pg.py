from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# ─────────────────────────── reuse the model layer ──────────────────────────
HERE = Path(__file__).resolve().parent
import importlib.util
spec = importlib.util.spec_from_file_location("models", HERE / "models.py")
models = importlib.util.module_from_spec(spec)        # type: ignore
spec.loader.exec_module(models)                       # type: ignore

Point  = models.Point
Vector = models.Vector
Field  = models.Field
Robot  = models.Robot
Ball   = models.Ball

# ────────────────────────── simulation constants ────────────────────────────
INNER_K      = 1
OUTER_K      = 2
MAX_SPEED  = 200            # [cm / s]

# ─────────────────────── border-avoidance helper ────────────────────────────
def compute_robot_velocity(field: Field, rb: Robot) -> Vector:
    """Same logic you had in test.py – untouched."""
    nearest    = field.nearest_border(rb.pos)
    border_pt  = nearest.nearest_point(rb.pos)
    border_vec = Vector.from_points(rb.pos, border_pt)

    normal  = rb.vel.normal(border_vec.angle)
    tangent = rb.vel.tangent(border_vec.angle)

    if field.is_inside(rb.pos):
        if border_vec.collinear(normal) == 1 and normal.length > border_vec.length * INNER_K:
            normal.length = border_vec.length * INNER_K
        new_v = normal + tangent

        for border in field.borders:
            if border.is_inside(rb.pos) and border is not nearest:
                border_pt  = border.nearest_point(rb.pos)
                border_vec = Vector.from_points(rb.pos, border_pt)
                normal     = new_v.normal(border_vec.angle)
                tangent    = new_v.tangent(border_vec.angle)
                if border_vec.collinear(normal) == 1 and normal.length > border_vec.length * INNER_K:
                    normal.length = border_vec.length * INNER_K
                    new_v         = normal + tangent
    else:
        normal.angle  = border_vec.angle
        normal.length = max(border_vec.length * OUTER_K, normal.length)
        new_v         = normal
        print('outside')

    # if new_v.length > 0:
    #     new_v.length = MAX_SPEED
    return new_v

# ────────────────────────── Simulation wrapper ──────────────────────────────
class Simulation:
    """
    Same public API as the old Matplotlib version but backed by PyQtGraph
    for ~10-20× faster redraws.
    """
    def __init__(self, dt: float = 1/30, show: bool = True):
        # ---------- physics state ------------------------------------------
        self.dt      = dt
        self.field   = Field()
        self.ball    = Ball(Point(10, 10))
        self.robot   = Robot(Point(85, -80))

        # ---------- Qt application & plot ----------------------------------
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        pg.setConfigOption('antialias', True)

        self.app = pg.mkQApp("Robot visualiser")

        self.win  = pg.GraphicsLayoutWidget(title="Robot trajectory")
        self.view: pg.PlotItem = self.win.addPlot()
        self._configure_view()

        # connect click-to-teleport
        self.view.scene().sigMouseClicked.connect(self._on_click)

        if show:
            self.win.show()

    # ──────────────────────── interactive hook ────────────────────────────
    def _on_click(self, ev):
        if ev.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        mouse_pt = self.view.vb.mapSceneToView(ev.scenePos())
        self.ball.pos = Point(mouse_pt.x(), mouse_pt.y())

    # ───────────────────── view helpers (axes, limits…) ────────────────────
    def _configure_view(self):
        self.view.setAspectLocked(True)
        self.view.setXRange(-91, 91)
        self.view.setYRange(-121.5, 121.5)
        self.view.setLabel('bottom', 'x [cm]')
        self.view.setLabel('left',   'y [cm]')
        
        if not hasattr(self, 'label'):                  # create only once
            self.label = pg.LabelItem(justify='right')
            self.win.addItem(self.label, row=1, col=0)

            # # live coordinate read-out (bottom-right)
            def _mouse_move(pos):
                if self.view.sceneBoundingRect().contains(pos):
                    p = self.view.vb.mapSceneToView(pos)
                    self.label.setText(f"(x, y) = ({p.x():.1f}, {p.y():.1f})")
            self.view.scene().sigMouseMoved.connect(_mouse_move)

    # ───────────────────────── physics tick ────────────────────────────────
    def _step_physics(self):
        self.robot.vel = Vector.from_points(self.robot.pos, self.ball.pos)
        self.robot.vel.length *= 1.5
        self.robot.vel.length = MAX_SPEED if self.robot.vel.length > MAX_SPEED else self.robot.vel.length

        self.robot.vel.draw(self.view, self.robot.pos, color='k', length_scale=0.5)

        self.robot.vel = compute_robot_velocity(self.field, self.robot)
        self.robot.pos = self.robot.pos.move(self.robot.vel * self.dt)

        self.robot.vel.draw(self.view, self.robot.pos, color='green', length_scale=0.5)

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
        # self.robot.vel.draw(self.view, self.robot.pos, color='green')
        # Qt redraws automatically; no explicit canvas.draw_idle() needed

    # ───────────────────────── public API ──────────────────────────────────
    @property
    def is_alive(self) -> bool:
        """False once the window has been closed by the user."""
        return self.win.isVisible()

    def step(self):
        """Advance physics by *dt* seconds and refresh the scene."""
        self._clear()
        self._step_physics()
        self._draw()
        self.app.processEvents()          # let Qt flush pending paints

    def pause(self, delay: Optional[float] = None):
        """
        Yield to the GUI event-loop — equivalent to plt.pause().
        *delay* defaults to self.dt.
        """
        self.app.processEvents()
        time.sleep(delay if delay is not None else self.dt)

# ─────────────────────────── standalone demo ───────────────────────────────
if __name__ == "__main__":
    sim = Simulation()
    while sim.is_alive:
        sim.step()
        sim.pause()
