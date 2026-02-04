from PyQt5 import QtGui

class CameraController:
    def __init__(self, view3d):
        self.view = view3d
        self.mode = "follow"

    def update(self, pos, rpy):
        if self.mode == "follow":
            self.view.opts['center'] = QtGui.QVector3D(*pos)
