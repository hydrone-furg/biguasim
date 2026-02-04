from PyQt5 import QtWidgets
import pyqtgraph as pg


class Plot2D:
    def __init__(self, layout: QtWidgets.QLayout, history: int = 1000):
        # ======= Tema global =======
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background', '#10151c')   # fundo azul-escuro
        pg.setConfigOption('foreground', '#e0e0e0')   # texto claro

        self.curves = {}
        self.history = history

        main_layout = QtWidgets.QHBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(20)

        # ======= Coluna de posição (X, Y, Z) =======
        pos_widget = QtWidgets.QWidget()
        pos_layout = QtWidgets.QVBoxLayout()
        pos_layout.setContentsMargins(5, 5, 5, 5)
        pos_layout.setSpacing(12)
        pos_widget.setLayout(pos_layout)

        pos_colors = {
            "X": "#00e5ff",  # ciano claro
            "Y": "#00ff99",  # verde água
            "Z": "#1e88e5"   # azul profundo
        }

        for label, color in pos_colors.items():
            pw = pg.PlotWidget(title=f"<b>{label}</b>")
            pw.showGrid(x=True, y=True, alpha=0.25)
            pw.setLabel("bottom", "Step", color='#aaaaaa')
            pw.setLabel("left", label, color='#aaaaaa')
            pw.setMinimumHeight(140)

            # Grid e eixos finos
            ax = pw.getAxis('bottom')
            ax.setPen(pg.mkPen(color='#777777', width=1))
            ay = pw.getAxis('left')
            ay.setPen(pg.mkPen(color='#777777', width=1))

            vb = pw.getViewBox()
            vb.setLimits(xMin=-1e9, xMax=1e9, yMin=-1e9, yMax=1e9)
            vb.setDefaultPadding(0.1)

            # Curva suave e ponto de destaque
            curve = pw.plot(pen=pg.mkPen(color=color, width=2.5))
            point = pw.plot([], [], pen=None, symbol='o',
                            symbolBrush=color, symbolPen=color, symbolSize=9)
            self.curves[label] = {"curve": curve, "point": point, "x": [], "y": [], "viewbox": vb}
            pos_layout.addWidget(pw)

        # ======= Coluna de orientação (Roll, Pitch, Yaw) =======
        ori_widget = QtWidgets.QWidget()
        ori_layout = QtWidgets.QVBoxLayout()
        ori_layout.setContentsMargins(5, 5, 5, 5)
        ori_layout.setSpacing(12)
        ori_widget.setLayout(ori_layout)

        ori_colors = {
            "Roll": "#ffeb3b",   # amarelo
            "Pitch": "#ff9800",  # laranja
            "Yaw": "#e91e63"     # rosa
        }

        for label, color in ori_colors.items():
            pw = pg.PlotWidget(title=f"<b>{label}</b>")
            pw.showGrid(x=True, y=True, alpha=0.25)
            pw.setLabel("bottom", "Step", color='#aaaaaa')
            pw.setLabel("left", label, color='#aaaaaa')
            pw.setMinimumHeight(140)

            ax = pw.getAxis('bottom')
            ax.setPen(pg.mkPen(color='#777777', width=1))
            ay = pw.getAxis('left')
            ay.setPen(pg.mkPen(color='#777777', width=1))

            vb = pw.getViewBox()
            vb.setLimits(xMin=-1e9, xMax=1e9, yMin=-1e9, yMax=1e9)
            vb.setDefaultPadding(0.1)

            curve = pw.plot(pen=pg.mkPen(color=color, width=2.5))
            point = pw.plot([], [], pen=None, symbol='o',
                            symbolBrush=color, symbolPen=color, symbolSize=9)
            self.curves[label] = {"curve": curve, "point": point, "x": [], "y": [], "viewbox": vb}
            ori_layout.addWidget(pw)

        main_layout.addWidget(pos_widget)
        main_layout.addWidget(ori_widget)
        layout.addLayout(main_layout, 1)

    # ======= Atualização dos dados =======
    def update(self, pos: list[float], rpy: list[float], step: int):
        values = {
            "X": pos[0], "Y": pos[1], "Z": pos[2],
            "Roll": rpy[0], "Pitch": rpy[1], "Yaw": rpy[2]
        }

        for label, value in values.items():
            curve = self.curves[label]
            curve["x"].append(step)
            curve["y"].append(value)

            if len(curve["x"]) > self.history:
                curve["x"] = curve["x"][-self.history:]
                curve["y"] = curve["y"][-self.history:]

            curve["curve"].setData(curve["x"], curve["y"])
            if curve["x"]:
                curve["point"].setData([curve["x"][-1]], [curve["y"][-1]])
