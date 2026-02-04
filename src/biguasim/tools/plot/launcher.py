import subprocess
import time
import pygetwindow as gw
from PyQt5 import QtWidgets, QtCore
from biguasim.visual.monitor import AgentMonitor


class BiguaSimLauncher(QtWidgets.QMainWindow):
    def __init__(self, biguasim_cmd, uuid="default", agent="auv0"):
        super().__init__()
        self.setWindowTitle("BiguaSim")
        self.resize(1920, 1080)

        # Layout principal dividido em 2 colunas
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.setCentralWidget(splitter)

        # Placeholder esquerdo (biguasim)
        self.holo_placeholder = QtWidgets.QLabel("Aguardando biguasim abrir...")
        self.holo_placeholder.setAlignment(QtCore.Qt.AlignCenter)
        splitter.addWidget(self.holo_placeholder)

        # Painel BiguaSim (plots 3D + 2D)
        self.monitor = AgentMonitor(uuid, agent, plot_pos=True)
        splitter.addWidget(self.monitor)

        # Proporções iniciais
        splitter.setSizes([1000, 900])

        # Iniciar biguasim
        self.biguasim_proc = subprocess.Popen(biguasim_cmd, shell=True)

        # Timer para tentar localizar a janela do biguasim
        self.check_timer = QtCore.QTimer()
        self.check_timer.timeout.connect(self.try_attach_biguasim_window)
        self.check_timer.start(1000)

    def try_attach_biguasim_window(self):
        """Tenta localizar a janela do biguasim e encaixar ao lado."""
        try:
            holo_window = next(
                w for w in gw.getWindowsWithTitle("") if "Holodeck" in w.title or "biguasim" in w.title
            )
            self.holo_placeholder.setText(f"biguasim detectado: {holo_window.title}")
            self.position_windows(holo_window)
            self.check_timer.stop()
        except StopIteration:
            print("Aguardando janela do biguasim...")

    def position_windows(self, holo_window):
        """Posiciona o biguasim e o BiguaSim lado a lado."""
        screen = QtWidgets.QApplication.desktop().availableGeometry()
        width = screen.width() // 2
        height = screen.height()

        # Reposiciona a janela do biguasim
        holo_window.moveTo(screen.left(), screen.top())
        holo_window.resizeTo(width, height)

        # Reposiciona o launcher (nossa janela Qt)
        self.move(screen.left() + width, screen.top())
        self.resize(width, height)


def run_launcher():
    import sys
    app = QtWidgets.QApplication(sys.argv)
    # exemplo de comando: 'python -m biguasim.scripts.run_world my_world'
    launcher = BiguaSimLauncher("python -m biguasim.scripts.run_world MyWorld", uuid="1234", agent="auv0")
    launcher.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    run_launcher()
