import sys
from threading import Thread
from PyQt5 import QtWidgets, QtCore
from .shm_receiver import SharedMemoryReceiver
from .plot3d import Plot3D
from .plot2d import Plot2D
from .camera import CameraController
from .control_panel import ControlPanel
import subprocess
import time
from Xlib import display

def get_dock_size():
    """
    Detecta automaticamente a largura (ou altura) da dock do Ubuntu (Dash-to-Dock).
    Retorna um valor em pixels. Se falhar, retorna 54 por padrão.
    """
    try:

        output = subprocess.check_output([
            "gsettings", "get",
            "org.gnome.shell.extensions.dash-to-dock", "dock-fixed"
        ], stderr=subprocess.DEVNULL).decode().strip()


        if not output:
            return 54


        icon_size = int(subprocess.check_output([
            "gsettings", "get",
            "org.gnome.shell.extensions.dash-to-dock", "dash-max-icon-size"
        ], stderr=subprocess.DEVNULL).decode().strip())


        return int(icon_size * 1.2 + 10)

    except Exception:
        return 54  # fallback padrão


def get_screen_resolution(single_monitor=False):
    """
    Obtém a resolução da tela.
    - single_monitor=True → usa somente o monitor principal (ex: 1920x1080)
    - single_monitor=False → usa toda a área combinada de monitores (ex: 3840x1080)
    """
    if single_monitor:
        # Usa apenas a tela principal (Qt)
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        screen = app.primaryScreen()
        size = screen.size()
        width, height = size.width(), size.height()
    else:
        # Usa a resolução total via Xlib
        try:
            dsp = display.Display()
            root = dsp.screen().root
            geometry = root.get_geometry()
            width = geometry.width
            height = geometry.height
            dsp.close()
        except Exception as e:
            print(f"[WARN] Falha ao obter resolução via Xlib: {e}. Usando Qt.")
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
            screen = app.primaryScreen()
            size = screen.size()
            width, height = size.width(), size.height()
    return width, height



def move_and_tile_windows(single_monitor=True):
    """Posiciona Holodeck e Monitor lado a lado em um único monitor, compensando a dock lateral."""
    width, height = get_screen_resolution(single_monitor=single_monitor)
    half_width = width // 2

    dock_width =  get_dock_size() 

    wmctrl_output = subprocess.check_output(["wmctrl", "-l"]).decode("utf-8").splitlines()

    holodeck_id = None
    monitor_id = None

    for line in wmctrl_output:
        if "Holodeck (64-bit Development SF_VULKAN_SM5)" in line:
            holodeck_id = line.split()[0]
        elif "BiguaSim Monitor" in line:
            monitor_id = line.split()[0]

    if not holodeck_id or not monitor_id:
        print("[WARN] Não foi possível encontrar uma ou ambas as janelas.")
        return

    # Corrige posição e largura (subtrai a dock)
    holodeck_width = half_width - dock_width // 2
    monitor_x = half_width + dock_width // 2

    subprocess.run(["wmctrl", "-ir", holodeck_id, "-e", f"0,{dock_width},0,{holodeck_width},{height}"])
    subprocess.run(["wmctrl", "-ir", monitor_id, "-e", f"0,{monitor_x},0,{holodeck_width},{height}"])

    print(f"[INFO] Janelas alinhadas lado a lado ({width}x{height}) com compensação de dock ({dock_width}px).")


from PyQt5 import QtWidgets, QtCore
from .plot3d import Plot3D
from .plot2d import Plot2D
from .camera import CameraController
from .control_panel import ControlPanel
from .shm_receiver import SharedMemoryReceiver


class AgentMonitor(QtWidgets.QMainWindow):
    """
    Interface gráfica do BiguaSim Monitor.
    Layout:
    ┌───────────────────────────────┬──────────────────────────┐
    │           Plot 3D             │                          │
    │-------------------------------│ Painel de controle       │
    │           Plot 2D             │                          │
    └───────────────────────────────┴──────────────────────────┘
    """

    def __init__(self, uuid: str, agent: str = "auv0", plot_pos: bool = True) -> None:
        super().__init__()
        self.receiver = SharedMemoryReceiver(uuid, agent)
        self.step = 0

        self.setWindowTitle(f"BiguaSim Monitor - {agent}")

        # === CENTRAL ===
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        # Layout principal (horizontal)
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(10)

        # Coluna esquerda (3D em cima, 2D embaixo)
        left_col = QtWidgets.QVBoxLayout()
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.setSpacing(10)

        # --- Plot 3D ---
        self.plot3d = Plot3D(left_col)
        self.camera = CameraController(self.plot3d.view3d)

        # --- Plot 2D ---
        if plot_pos:
            self.plot2d = Plot2D(left_col)

        # --- Adiciona coluna esquerda ---
        main_layout.addLayout(left_col, stretch=4)

        # --- Painel de controle ---
        self.controls = ControlPanel(self.plot3d, getattr(self, 'plot2d', None), self.camera)
        self.controls.setMinimumWidth(220)
        main_layout.addWidget(self.controls, stretch=1)

        # === Timer de atualização ===
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)

        self.resize(1600, 900)

    def update_data(self) -> None:
        """Atualiza dados e plots"""
        pos, rpy = self.receiver.position, self.receiver.rpy
        self.plot3d.update(pos, rpy, self.step)
        if hasattr(self, "plot2d"):
            self.plot2d.update(pos, rpy, self.step)
        self.camera.update(pos, rpy)
        self.step += 1



def launch_agent_monitor(uuid: str, agent_name: str = "auv0", plot_pos: bool = True) -> None:
    """Lança o visualizador em thread separada e posiciona lado a lado com o Holodeck."""
    def run():
        app = QtWidgets.QApplication(sys.argv)
        monitor = AgentMonitor(uuid, agent_name, plot_pos)
        monitor.show()

        # Espera a janela aparecer e tenta mover
        app.processEvents()
        QtCore.QTimer.singleShot(200, lambda:move_and_tile_windows(single_monitor=True))

        sys.exit(app.exec_())

    Thread(target=run, daemon=True).start()