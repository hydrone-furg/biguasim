import numpy as np
import pyqtgraph.opengl as gl
from PyQt5 import QtWidgets, QtGui

class Plot3D:
    def __init__(self, layout:QtWidgets.QLayout):
        self.view3d = gl.GLViewWidget()
        self.view3d.opts['distance'] = 40
        layout.addWidget(self.view3d, 2)

        grid = gl.GLGridItem()
        grid.setSize(1000, 1000)
        grid.setSpacing(5, 5)
        self.view3d.addItem(grid)

        self.traj_data = np.zeros((0, 3))
        self.traj_plot = gl.GLLinePlotItem(color=(0, 0, 1, 1), width=2)
        self.view3d.addItem(self.traj_plot)

        self.agent_dot = gl.GLScatterPlotItem(size=12, color=(1, 0, 0, 1), pxMode=True)
        self.view3d.addItem(self.agent_dot)

        self.orientation_line = gl.GLLinePlotItem(color=(0, 1, 0, 1), width=2)
        self.view3d.addItem(self.orientation_line)

        self.hud = QtWidgets.QLabel(self.view3d)
        self.hud.setStyleSheet("color: white; background: rgba(0,0,0,160); padding: 4px;")
        self.hud.move(10, 10)
        self.hud.resize(300, 120)
        self.hud.show()

    def update(self, pos:list[float], rpy:list[float], step:int):

        self.traj_data = np.vstack([self.traj_data, pos])
        self.traj_plot.setData(pos=self.traj_data)
        self.agent_dot.setData(pos=pos.reshape(1, 3))

        yaw, pitch = rpy[2], rpy[1]
        direction = np.array([
            np.cos(yaw) * np.cos(pitch),
            np.sin(yaw) * np.cos(pitch),
            np.sin(pitch)
        ]) * 5.0
        self.orientation_line.setData(pos=np.vstack([pos, pos + direction]))

        self.hud.setText(
            f"Step: {step}\n"
            f"Pos: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}\n"
            f"Roll={rpy[0]:.2f}, Pitch={rpy[1]:.2f}, Yaw={rpy[2]:.2f}"
        )
