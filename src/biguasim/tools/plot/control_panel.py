from PyQt5 import QtWidgets

class ControlPanel(QtWidgets.QWidget):
    def __init__(self, plot3d, plot2d, camera_controller):
        super().__init__()
        self.plot3d = plot3d
        self.plot2d = plot2d
        self.camera_controller = camera_controller

        layout = QtWidgets.QVBoxLayout(self)

        self.show_traj = QtWidgets.QCheckBox("Show trajectory")
        self.show_traj.setChecked(True)
        self.show_traj.stateChanged.connect(self.toggle_traj)
        layout.addWidget(self.show_traj)

        self.show_ori = QtWidgets.QCheckBox("Show orientation")
        self.show_ori.setChecked(True)
        self.show_ori.stateChanged.connect(self.toggle_orientation)
        layout.addWidget(self.show_ori)

        self.show_hud = QtWidgets.QCheckBox("Mostrar HUD")
        self.show_hud.setChecked(True)
        self.show_hud.stateChanged.connect(self.toggle_hud)
        layout.addWidget(self.show_hud)

        layout.addWidget(QtWidgets.QLabel("Modo de câmera"))
        self.camera_mode = QtWidgets.QComboBox()
        self.camera_mode.addItems(["follow", "orbit", "top", "free"])
        self.camera_mode.currentTextChanged.connect(self.change_camera_mode)
        layout.addWidget(self.camera_mode)

        layout.addStretch()

    def toggle_traj(self, state):
        self.plot3d.traj_plot.setVisible(state == 2)

    def toggle_orientation(self, state):
        self.plot3d.orientation_line.setVisible(state == 2)

    def toggle_hud(self, state):
        self.plot3d.hud.setVisible(state == 2)

    def change_camera_mode(self, mode):
        self.camera_controller.mode = mode
