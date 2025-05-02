from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

# Re‑use the model layer --------------------------------------------------------
HERE = Path(__file__).resolve().parent
import importlib.util
spec = importlib.util.spec_from_file_location("models", HERE / "models.py")
models = importlib.util.module_from_spec(spec)  # type: ignore
spec.loader.exec_module(models)  # type: ignore

Point = models.Point
Vector = models.Vector
Field = models.Field
Robot = models.Robot
Ball = models.Ball

class Visualization:
    def __init__(self, dt: float):
        self.dt = dt
        self.field = Field()
        self.ball = Ball(Point(10, 10))
        self.robot = Robot(Point(60, -80))
    
    def start(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(5, 6))
        self._configure_axes()
        self._cid = self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        plt.show(block=False)
    
    def _on_click(self, event):
        if event.inaxes is not self.ax:
            return
        self.ball.pos = Point(event.xdata, event.ydata)
    
    def _configure_axes(self):
        self.ax.set_xlim(-91, 91)
        self.ax.set_ylim(-121.5, 121.5)
        self.ax.set_aspect("equal")
        self.ax.set_title("Visualization")

    def draw(self):
        self.ax.cla()
        self._configure_axes()
        self.field.draw(self.ax)
        self.ball.draw(self.ax)
        self.robot.draw(self.ax)
        self.fig.canvas.draw_idle()
        plt.pause(0.001)
    
    def pause(self, delay: Optional[float] = None):
        plt.pause(delay if delay is not None else self.dt)
    
    @property
    def is_alive(self) -> bool:
        return plt.fignum_exists(self.fig.number)

lst = time.time()
if __name__ == "__main__":
    sim = Visualization(0.25)
    sim.start()
    while sim.is_alive:
        sim.draw()
        print(time.time() - lst)
        lst = time.time()