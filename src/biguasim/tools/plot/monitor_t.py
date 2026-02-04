import sys, os, time
import numpy as np
from functools import reduce
from threading import Thread
from PyQt5 import QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtGui
from biguasim.shmem import Shmem
from .shm_receiver import SharedMemoryReceiver

class SharedMemoryReceiver:
    def __init__(self, uuid, agent="auv0"):
        self.uuid = uuid
        self.agent = agent
        self.position = np.zeros(3)
        self.rpy = np.zeros(3)
        self.pos_mem = Shmem(f"{agent}_LocationSensor_sensor_data", (3,), np.float32, uuid)
        self.rot_mem = Shmem(f"{agent}_RotationSensor_sensor_data", (3,), np.float32, uuid)
        self.running = True
        self.thread = Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        while self.running:
            self.position = np.copy(self.pos_mem.np_array)
            self.rpy = np.copy(self.rot_mem.np_array)
            time.sleep(0.05)

class AgentMonitor(QtWidgets.QMainWindow):
    def __init__(self, shm_receiver, plot_pos=True):
        super().__init__()
        self.shm_receiver = shm_receiver
        self.setWindowTitle(f"BiguaSim Agent Monitor - {shm_receiver.agent}")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        #plot 3d
        self.view3d = gl.GLViewWidget()
        self.view3d.opts['distance'] = 40
        layout.addWidget(self.view3d, 2)

        grid = gl.GLGridItem()
        grid.setSize(10000, 10000)
        grid.setSpacing(5, 5)
        self.view3d.addItem(grid)

        self.traj_data = np.zeros((0, 3))
        self.traj_plot = gl.GLLinePlotItem(pos=self.traj_data, color=(0, 0, 1, 1), width=2)
        self.view3d.addItem(self.traj_plot)

        self.agent_dot = gl.GLScatterPlotItem(pos=np.array([[0, 0, 0]]),
                                              size=12, color=(1, 0, 0, 1), pxMode=True)
        self.view3d.addItem(self.agent_dot)

        self.orientation_line = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 1]]),
                                                  color=(0, 1, 0, 1), width=1)
        self.view3d.addItem(self.orientation_line)

        #HUD
        self.hud = QtWidgets.QLabel(self.view3d)
        self.hud.setStyleSheet("color: white; background: rgba(0,0,0,160); padding: 4px;")
        self.hud.move(10, 10)
        self.hud.resize(300, 120)
        self.hud.show()

        self.curves = {}
        if plot_pos:
            right_panel = QtWidgets.QWidget()
            layout.addWidget(right_panel, 1)
            grid_layout = QtWidgets.QGridLayout(right_panel)

            labels = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
            colors = ['r', 'g', 'b', 'y', 'c', 'm']
            
            for i, (label, color) in enumerate(zip(labels, colors)):
                pw = pg.PlotWidget(title=label)
                pw.showGrid(x=True, y=True)
                pw.setLabel("bottom", "Step")
                pw.setLabel("left", label)
                pw.enableAutoRange('y', True)
                grid_layout.addWidget(pw, i // 2, i % 2)
                curve = pw.plot(pen=color)
                self.curves[label] = {"curve": curve, "data": []}

        self.step = 0

    def update_data(self):
        pos = self.shm_receiver.position
        rpy = self.shm_receiver.rpy

        self.traj_data = np.vstack([self.traj_data, pos])
        self.traj_plot.setData(pos=self.traj_data)
        self.agent_dot.setData(pos=pos.reshape(1, 3))

        yaw, pitch = rpy[2], rpy[1]
        direction = np.array([
            np.cos(yaw) * np.cos(pitch),
            -np.sin(yaw) * np.cos(pitch),
            -np.sin(pitch)
        ]) * 5.0

        self.orientation_line.setData(pos=np.vstack([pos, pos + direction]))

        self.hud.setText(
            f"Step: {self.step}\n"
            f"Pos: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}\n"
            f"Roll={rpy[0]:.2f}, Pitch={rpy[1]:.2f}, Yaw={rpy[2]:.2f}"
        )

        values = {"X": pos[0], "Y": pos[1], "Z": pos[2],
                  "Roll": rpy[0], "Pitch": rpy[1], "Yaw": rpy[2]}
        if self.curves:
            for k, v in values.items():
                data = self.curves[k]["data"]
                data.append(v)
                if len(data) > 100:
                    data.pop(0)
                self.curves[k]["curve"].setData(data)

        
        pos_vec = QtGui.QVector3D(pos[0], pos[1], pos[2])
        self.view3d.opts['center'] = pos_vec
        self.step += 1

def launch_agent_monitor(uuid, agent_name="auv0", plot_pos=True):
    """Inicia o visualizador em uma thread separada"""
    def run():
        app = QtWidgets.QApplication(sys.argv)
        receiver = SharedMemoryReceiver(uuid=uuid, agent=agent_name)
        monitor = AgentMonitor(receiver, plot_pos=plot_pos)
        monitor.show()
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(monitor.update_data)
        timer.start(100)
        sys.exit(app.exec_())

    Thread(target=run, daemon=True).start()
