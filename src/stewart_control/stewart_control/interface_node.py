#!/home/rem/rtimulib-env/bin/python3

import sys
import os
import signal
import subprocess
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QFrame,
    QDoubleSpinBox,
    QPushButton,
)
from PySide6.QtGui import QImage, QPixmap, QColor, QPainter, QPen
from PySide6.QtCore import QTimer, Qt


from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib

matplotlib.rcParams.update(
    {
        "figure.facecolor": "#1e1e2e",
        "axes.facecolor": "#2a2a3c",
        "axes.edgecolor": "#444466",
        "axes.labelcolor": "#b0b0cc",
        "text.color": "#c0c0dd",
        "xtick.color": "#888899",
        "ytick.color": "#888899",
        "grid.color": "#3a3a50",
        "grid.alpha": 0.5,
    }
)

# ─── Couleurs du thème ───────────────────────────────────────────────────
BG_DARK = "#12121c"
BG_CARD = "#1e1e2e"
BG_INPUT = "#2a2a3c"
ACCENT = "#5b8def"
ACCENT_HOVER = "#7aa5f7"
SUCCESS = "#43b581"
SUCCESS_HOVER = "#5bcf96"
DANGER = "#ed4245"
DANGER_HOVER = "#f05d60"
WARNING = "#faa61a"
TEXT_PRIMARY = "#e0e0ee"
TEXT_SECONDARY = "#9999aa"
TEXT_MUTED = "#666680"
BORDER = "#2e2e42"
BORDER_ACCENT = "#5b8def40"

GLOBAL_STYLESHEET = f"""
    QWidget {{
        background-color: {BG_DARK};
        color: {TEXT_PRIMARY};
        font-family: 'Segoe UI', 'Roboto', 'Helvetica Neue', Arial, sans-serif;
        font-size: 12px;
    }}
    QDoubleSpinBox {{
        background-color: {BG_INPUT};
        color: {TEXT_PRIMARY};
        border: 1px solid {BORDER};
        border-radius: 6px;
        padding: 4px 8px;
        font-size: 13px;
        min-height: 28px;
        selection-background-color: {ACCENT};
    }}
    QDoubleSpinBox:focus {{
        border: 1px solid {ACCENT};
    }}
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
        width: 18px;
        border: none;
        background: {BG_INPUT};
    }}
    QDoubleSpinBox::up-arrow {{
        image: none;
        border-left: 4px solid transparent;
        border-right: 4px solid transparent;
        border-bottom: 5px solid {TEXT_SECONDARY};
    }}
    QDoubleSpinBox::down-arrow {{
        image: none;
        border-left: 4px solid transparent;
        border-right: 4px solid transparent;
        border-top: 5px solid {TEXT_SECONDARY};
    }}
"""


class StatusLed(QWidget):
    """Petit indicateur LED rond (vert/orange/rouge/gris)."""

    COLOR_MAP = {
        "ok": QColor(SUCCESS),
        "warn": QColor(WARNING),
        "error": QColor(DANGER),
        "off": QColor(TEXT_MUTED),
    }

    def __init__(self, size=10, parent=None):
        super().__init__(parent)
        self._state = "off"
        self.setFixedSize(size, size)

    def set_state(self, state: str):
        if state != self._state:
            self._state = state
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        color = self.COLOR_MAP.get(self._state, self.COLOR_MAP["off"])
        p.setBrush(color)
        p.setPen(QPen(color.darker(130), 1))
        p.drawEllipse(1, 1, self.width() - 2, self.height() - 2)
        p.end()


def _card(title: str, icon: str = "") -> QFrame:
    """Crée un cadre de type carte avec titre."""
    frame = QFrame()
    frame.setObjectName("card")
    frame.setStyleSheet(f"""
        QFrame#card {{
            background-color: {BG_CARD};
            border: 1px solid {BORDER};
            border-radius: 10px;
            padding: 0px;
        }}
    """)
    return frame


def _section_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold;"
        " letter-spacing: 1px; text-transform: uppercase; padding: 0;"
    )
    return lbl


def _make_button(text, color, hover, text_color="white", icon=""):
    btn = QPushButton(f"{icon}  {text}" if icon else text)
    btn.setCursor(Qt.PointingHandCursor)
    btn.setStyleSheet(f"""
        QPushButton {{
            background-color: {color};
            color: {text_color};
            border: none;
            border-radius: 8px;
            padding: 8px 14px;
            font-size: 12px;
            font-weight: 600;
            min-height: 32px;
        }}
        QPushButton:hover {{
            background-color: {hover};
        }}
        QPushButton:pressed {{
            background-color: {color};
            padding-top: 10px;
        }}
    """)
    return btn


class MotorPlotWidget(QWidget):
    """Widget graphe moteur individuel — thème sombre."""

    def __init__(self, motor_number, max_len=100):
        super().__init__()
        self.motor_number = motor_number
        self.max_len = max_len
        self.feedback_data = deque([0] * max_len, maxlen=max_len)
        self.current_consigne = 0.0

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        self.fig = Figure(figsize=(2.2, 1.4), dpi=80)
        self.fig.subplots_adjust(left=0.15, right=0.95, top=0.88, bottom=0.18)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setStyleSheet("background: transparent;")
        layout.addWidget(self.canvas)

        self.ax = self.fig.add_subplot(111)
        self.ax.set_title(
            f"M{motor_number}", fontsize=10, fontweight="bold", color="#c0c0dd", pad=4
        )
        self.ax.set_ylabel("L (cm)", fontsize=7, labelpad=2)
        self.ax.tick_params(labelsize=6, length=3, pad=2)
        self.ax.grid(True, linewidth=0.5)

        (self.line_consigne,) = self.ax.plot(
            [],
            [],
            color="#5b8def",
            linewidth=1.8,
            linestyle="--",
            label="Consigne",
            zorder=3,
        )
        (self.line_feedback,) = self.ax.plot(
            range(self.max_len),
            self.feedback_data,
            color="#ed4245",
            linewidth=1.2,
            label="Feedback",
            zorder=2,
        )
        self.ax.legend(fontsize=6, loc="upper right", framealpha=0.6)
        self.ax.set_ylim(0, 0.5)
        self.ax.set_xlim(0, self.max_len)

    def update_plot(self, consigne_value, feedback_value):
        self.current_consigne = consigne_value
        self.line_consigne.set_data([0, self.max_len], [consigne_value, consigne_value])
        self.feedback_data.append(feedback_value)
        self.line_feedback.set_ydata(self.feedback_data)

        all_values = list(self.feedback_data) + [self.current_consigne]
        if all_values:
            mn = min(all_values)
            mx = max(all_values)
            margin = (mx - mn) * 0.15 if mx > mn else 0.1
            self.ax.set_ylim(mn - margin, mx + margin)

        self.canvas.draw_idle()


class InterfaceGUI(QWidget):
    # Chemin du logo — cherche source, install-share, workspace-root, etc.
    _LOGO_FILENAME = "abmi_logo.png"
    _LOGO_SEARCH_PATHS = [
        # Source-tree: stewart_control/stewart_control/../assets/
        os.path.join(os.path.dirname(__file__), "..", "assets", "abmi_logo.png"),
        # Source-tree fallback: stewart_control/stewart_control/assets/
        os.path.join(os.path.dirname(__file__), "assets", "abmi_logo.png"),
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stewart Platform — ABMI Groupe")
        self.setMinimumSize(1100, 680)
        self.setStyleSheet(GLOBAL_STYLESHEET)

        # Publishers ROS2
        self.pos_pub = None
        self.ori_pub = None

        # Mode actif
        self._active_mode = None  # "acquisition", "auto", "manual", "home"

        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # ══════════ COLONNE GAUCHE ══════════
        left_layout = QVBoxLayout()
        left_layout.setSpacing(8)

        # ── Header Logo + Titre ──
        header_frame = QFrame()
        header_frame.setStyleSheet(
            f"background-color: {BG_CARD}; border: 1px solid {BORDER};"
            " border-radius: 10px;"
        )
        header_inner = QHBoxLayout(header_frame)
        header_inner.setContentsMargins(10, 6, 10, 6)
        header_inner.setSpacing(10)

        self.logo_label = QLabel()
        self.logo_label.setFixedSize(44, 44)
        self.logo_label.setScaledContents(True)
        self.logo_label.setStyleSheet("border: none; background: transparent;")
        logo_path = self._find_logo()
        if logo_path:
            pix = QPixmap(logo_path)
            if not pix.isNull():
                self.logo_label.setPixmap(pix)
            else:
                self.logo_label.setText("ABMI")
                self.logo_label.setStyleSheet(
                    f"color: {ACCENT}; font-weight: bold; font-size: 11px;"
                    " border: none; background: transparent;"
                )
        else:
            self.logo_label.setText("ABMI")
            self.logo_label.setStyleSheet(
                f"color: {ACCENT}; font-weight: bold; font-size: 11px;"
                " border: none; background: transparent;"
            )
        header_inner.addWidget(self.logo_label)

        title_block = QVBoxLayout()
        title_block.setSpacing(0)
        app_title = QLabel("Stewart Platform")
        app_title.setStyleSheet(
            f"color: {TEXT_PRIMARY}; font-size: 15px; font-weight: bold;"
            " border: none; background: transparent;"
        )
        app_subtitle = QLabel("ABMI Groupe — Contrôle & Monitoring")
        app_subtitle.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px;"
            " border: none; background: transparent;"
        )
        title_block.addWidget(app_title)
        title_block.addWidget(app_subtitle)
        header_inner.addLayout(title_block)
        header_inner.addStretch()

        left_layout.addWidget(header_frame)

        # ── Carte Contrôle ──
        ctrl_card = _card("Contrôle")
        ctrl_inner = QVBoxLayout(ctrl_card)
        ctrl_inner.setContentsMargins(12, 10, 12, 10)
        ctrl_inner.setSpacing(6)

        ctrl_header = QHBoxLayout()
        ctrl_header.addWidget(QLabel("⚙"))
        ctrl_title = QLabel("CONTRÔLE MANUEL")
        ctrl_title.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold;"
            " letter-spacing: 1.5px;"
        )
        ctrl_header.addWidget(ctrl_title)
        ctrl_header.addStretch()
        ctrl_inner.addLayout(ctrl_header)

        input_grid = QGridLayout()
        input_grid.setSpacing(6)

        self.input_x = self._spin(-1, 1, 3, 0.01)
        self.input_y = self._spin(-1, 1, 3, 0.01)
        self.input_z = self._spin(-1, 1, 3, 0.01)
        self.input_roll = self._spin(-180, 180, 1, 1.0)
        self.input_pitch = self._spin(-180, 180, 1, 1.0)
        self.input_yaw = self._spin(-180, 180, 1, 1.0)

        for row, (label, spin) in enumerate(
            [
                ("X (cm)", self.input_x),
                ("Y (cm)", self.input_y),
                ("Z (cm)", self.input_z),
            ]
        ):
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 11px;")
            input_grid.addWidget(lbl, row, 0)
            input_grid.addWidget(spin, row, 1)

        for row, (label, spin) in enumerate(
            [
                ("Roll (°)", self.input_roll),
                ("Pitch (°)", self.input_pitch),
                ("Yaw (°)", self.input_yaw),
            ]
        ):
            lbl = QLabel(label)
            lbl.setStyleSheet(f"color: {TEXT_SECONDARY}; font-size: 11px;")
            input_grid.addWidget(lbl, row, 2)
            input_grid.addWidget(spin, row, 3)

        ctrl_inner.addLayout(input_grid)

        self.button_update_manual = _make_button(
            "Appliquer", ACCENT, ACCENT_HOVER, icon="▶"
        )
        self.button_update_manual.clicked.connect(self.send_manual_pose)
        ctrl_inner.addWidget(self.button_update_manual)

        left_layout.addWidget(ctrl_card)

        # ── Carte Actions ──
        act_card = _card("Actions")
        act_inner = QVBoxLayout(act_card)
        act_inner.setContentsMargins(12, 10, 12, 10)
        act_inner.setSpacing(6)

        act_header = QHBoxLayout()
        act_header.addWidget(QLabel("🎮"))
        act_title = QLabel("ACTIONS")
        act_title.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold;"
            " letter-spacing: 1.5px;"
        )
        act_header.addWidget(act_title)
        act_header.addStretch()
        # Mode indicator label
        self.mode_label = QLabel("")
        self.mode_label.setStyleSheet(
            f"color: {SUCCESS}; font-size: 10px; font-weight: bold;"
        )
        act_header.addWidget(self.mode_label)
        act_inner.addLayout(act_header)

        btn_grid = QGridLayout()
        btn_grid.setSpacing(6)

        self.button_start_acq = _make_button(
            "Acquisition", "#2d5aa0", "#3668b5", icon="📡"
        )
        self.button_start_acq.clicked.connect(self.start_acquisition)
        self.button_start_att = _make_button(
            "Automatique", "#2d8a4e", "#38a05e", icon="🤖"
        )
        self.button_start_att.clicked.connect(self.start_attelage)
        self.button_manual = _make_button("Manuel", "#7c5cbf", "#9070d0", icon="🕹️")
        self.button_manual.clicked.connect(self.start_manual_node)
        self.button_uart = _make_button("État initial", "#5c7080", "#6d8494", icon="🏠")
        self.button_uart.clicked.connect(self.start_uart_node)
        self.button_stop = _make_button("ARRÊTER", DANGER, DANGER_HOVER, icon="⏹")
        self.button_stop.clicked.connect(self.stop_ros_launch)

        btn_grid.addWidget(self.button_start_acq, 0, 0)
        btn_grid.addWidget(self.button_start_att, 0, 1)
        btn_grid.addWidget(self.button_manual, 1, 0)
        btn_grid.addWidget(self.button_uart, 1, 1)
        act_inner.addLayout(btn_grid)
        act_inner.addWidget(self.button_stop)

        left_layout.addWidget(act_card)

        # ── Carte Caméra ──
        cam_card = _card("Caméra")
        cam_inner = QVBoxLayout(cam_card)
        cam_inner.setContentsMargins(8, 8, 8, 8)
        cam_inner.setSpacing(4)

        cam_header = QHBoxLayout()
        cam_header.addWidget(QLabel("📹"))
        cam_title = QLabel("CAMÉRA ARUCO")
        cam_title.setStyleSheet(
            f"color: {TEXT_SECONDARY}; font-size: 10px; font-weight: bold;"
            " letter-spacing: 1.5px;"
        )
        cam_header.addWidget(cam_title)
        cam_header.addStretch()
        self.cam_led = StatusLed(8)
        cam_header.addWidget(self.cam_led)
        cam_inner.addLayout(cam_header)

        self.video_label = QLabel()
        self.video_label.setMinimumSize(280, 210)
        self.video_label.setStyleSheet(
            f"border: 1px solid {BORDER}; border-radius: 6px;"
            f" background-color: #0a0a14;"
        )
        self.video_label.setScaledContents(True)
        self.video_label.setAlignment(Qt.AlignCenter)
        cam_inner.addWidget(self.video_label, stretch=1)

        left_layout.addWidget(cam_card, stretch=1)
        main_layout.addLayout(left_layout, 30)

        # ══════════ COLONNE DROITE ══════════
        right_layout = QVBoxLayout()
        right_layout.setSpacing(8)

        # ── Ligne de capteurs (ArUco + IMU + Fusion) ──
        sensors_row = QHBoxLayout()
        sensors_row.setSpacing(8)

        # ArUco card
        aruco_card = _card("ArUco")
        aruco_inner = QVBoxLayout(aruco_card)
        aruco_inner.setContentsMargins(12, 8, 12, 8)
        aruco_inner.setSpacing(4)
        aruco_h = QHBoxLayout()
        self.aruco_led = StatusLed(8)
        aruco_h.addWidget(self.aruco_led)
        aruco_h.addWidget(_section_label("ARUCO"))
        aruco_h.addStretch()
        aruco_inner.addLayout(aruco_h)
        self.label_aruco = QLabel("Position:  —")
        self.label_aruco.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 13px;")
        self.label_aruco_ori = QLabel("Orientation:  —")
        self.label_aruco_ori.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 12px;")
        aruco_inner.addWidget(self.label_aruco)
        aruco_inner.addWidget(self.label_aruco_ori)
        sensors_row.addWidget(aruco_card)

        # IMU card
        imu_card = _card("IMU")
        imu_inner = QVBoxLayout(imu_card)
        imu_inner.setContentsMargins(12, 8, 12, 8)
        imu_inner.setSpacing(4)
        imu_h = QHBoxLayout()
        self.imu_led = StatusLed(8)
        imu_h.addWidget(self.imu_led)
        imu_h.addWidget(_section_label("IMU"))
        imu_h.addStretch()
        imu_inner.addLayout(imu_h)
        self.label_imu = QLabel("R: —   P: —   Y: —")
        self.label_imu.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 13px;")
        imu_inner.addWidget(self.label_imu)
        sensors_row.addWidget(imu_card)

        # Fusion card
        fusion_card = _card("Fusion")
        fusion_inner = QVBoxLayout(fusion_card)
        fusion_inner.setContentsMargins(12, 8, 12, 8)
        fusion_inner.setSpacing(4)
        fusion_h = QHBoxLayout()
        self.fusion_led = StatusLed(8)
        fusion_h.addWidget(self.fusion_led)
        fusion_h.addWidget(_section_label("FUSION"))
        fusion_h.addStretch()
        fusion_inner.addLayout(fusion_h)
        self.label_fusion = QLabel("R: —   P: —   Y: —")
        self.label_fusion.setStyleSheet(f"color: {TEXT_PRIMARY}; font-size: 13px;")
        fusion_inner.addWidget(self.label_fusion)
        sensors_row.addWidget(fusion_card)

        right_layout.addLayout(sensors_row)

        # ── Carte Moteurs ──
        motor_card = _card("Moteurs")
        motor_inner = QVBoxLayout(motor_card)
        motor_inner.setContentsMargins(12, 8, 12, 8)
        motor_inner.setSpacing(4)
        motor_h = QHBoxLayout()
        self.motor_led = StatusLed(8)
        motor_h.addWidget(self.motor_led)
        motor_h.addWidget(_section_label("MOTEURS"))
        motor_h.addStretch()
        motor_inner.addLayout(motor_h)

        self.label_verins = QLabel("Consignes:  — cm")
        self.label_verins.setStyleSheet(
            f"color: {ACCENT}; font-size: 12px; font-family: 'Consolas', monospace;"
        )
        self.label_feedback = QLabel("Feedback:  — cm")
        self.label_feedback.setStyleSheet(
            f"color: {WARNING}; font-size: 12px; font-family: 'Consolas', monospace;"
        )
        motor_inner.addWidget(self.label_verins)
        motor_inner.addWidget(self.label_feedback)
        right_layout.addWidget(motor_card)

        # ── Carte Graphes (3×2) ──
        plots_card = _card("Graphes")
        plots_inner = QVBoxLayout(plots_card)
        plots_inner.setContentsMargins(4, 4, 4, 4)
        plots_inner.setSpacing(2)

        plots_grid = QGridLayout()
        plots_grid.setSpacing(2)
        self.motor_plots = []
        for i in range(6):
            plot = MotorPlotWidget(i + 1)
            self.motor_plots.append(plot)
            row, col = divmod(i, 3)
            plots_grid.addWidget(plot, row, col)
        plots_inner.addLayout(plots_grid)
        right_layout.addWidget(plots_card, stretch=1)

        main_layout.addLayout(right_layout, 70)

        self.setLayout(main_layout)

        # ── État des processus ──
        self.acquisition_process = None
        self.fusion_process = None
        self.attelage_process = None
        self.uart_process = None
        self.manual_process = None

        self.last_consigne = [0] * 6

        # Timer pour surveiller la vivacité des données capteurs
        self._last_imu_t = 0
        self._last_aruco_t = 0
        self._last_fusion_t = 0
        self._last_motor_t = 0

        import time

        self._time = time
        self._led_timer = QTimer()
        self._led_timer.timeout.connect(self._check_leds)
        self._led_timer.start(2000)

    # ── helpers ──
    @classmethod
    def _find_logo(cls):
        """Search for the ABMI logo in several locations (source tree, colcon share)."""
        # 1. Static search paths (source tree)
        for p in cls._LOGO_SEARCH_PATHS:
            abspath = os.path.abspath(p)
            if os.path.isfile(abspath):
                return abspath

        # 2. Colcon share directory (installed package)
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = get_package_share_directory("stewart_control")
            share_logo = os.path.join(share_dir, "assets", cls._LOGO_FILENAME)
            if os.path.isfile(share_logo):
                return share_logo
        except Exception:
            pass

        # 3. Workspace-root relative fallback (cwd-based)
        for rel in [
            os.path.join("src", "stewart_control", "assets", cls._LOGO_FILENAME),
            os.path.join("assets", cls._LOGO_FILENAME),
        ]:
            if os.path.isfile(rel):
                return os.path.abspath(rel)

        return None

    @staticmethod
    def _spin(lo, hi, decimals, step):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(decimals)
        s.setSingleStep(step)
        return s

    def _is_running(self, proc):
        return proc is not None and proc.poll() is None

    def _start_process(self, cmd):
        return subprocess.Popen(cmd, preexec_fn=os.setsid)

    def _set_mode(self, mode_name):
        self._active_mode = mode_name
        self.mode_label.setText(f"● {mode_name.upper()}")
        self.mode_label.setStyleSheet(
            f"color: {SUCCESS}; font-size: 10px; font-weight: bold;"
        )

    def _clear_mode(self):
        self._active_mode = None
        self.mode_label.setText("")

    def _check_leds(self):
        now = self._time.time()
        timeout = 3.0
        self.imu_led.set_state("ok" if now - self._last_imu_t < timeout else "off")
        self.aruco_led.set_state("ok" if now - self._last_aruco_t < timeout else "off")
        self.fusion_led.set_state(
            "ok" if now - self._last_fusion_t < timeout else "off"
        )
        self.motor_led.set_state("ok" if now - self._last_motor_t < timeout else "off")
        self.cam_led.set_state("ok" if now - self._last_aruco_t < timeout else "off")

    # --- Fonctions boutons ---
    def start_acquisition(self):
        if self._is_running(self.acquisition_process):
            return
        self.acquisition_process = subprocess.Popen(
            ["ros2", "launch", "stewart_control", "stewart_ordered_launch.py"],
            preexec_fn=os.setsid,
        )
        if not self._is_running(self.fusion_process):
            self.fusion_process = self._start_process(
                ["ros2", "run", "stewart_control", "fusion_node"]
            )
        self._set_mode("acquisition")

    def start_attelage(self):
        if self._is_running(self.attelage_process):
            return
        self.attelage_process = subprocess.Popen(
            ["ros2", "launch", "stewart_control", "stewart.launch.py"],
            preexec_fn=os.setsid,
        )
        self._set_mode("automatique")

    def start_manual_node(self):
        if self._is_running(self.manual_process):
            return
        self.manual_process = subprocess.Popen(
            ["ros2", "launch", "stewart_control", "manual_launch.py"],
            preexec_fn=os.setsid,
        )
        self._set_mode("manuel")

    def start_uart_node(self):
        if self._is_running(self.uart_process):
            return
        self.uart_process = subprocess.Popen(
            ["ros2", "launch", "stewart_control", "launch_posHome.py"],
            preexec_fn=os.setsid,
        )
        self._set_mode("état initial")

    def stop_ros_launch(self):
        for proc in [
            self.fusion_process,
            self.acquisition_process,
            self.attelage_process,
            self.uart_process,
            self.manual_process,
        ]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=5)
                except (OSError, subprocess.TimeoutExpired):
                    pass
                finally:
                    proc.kill()
        subprocess.call(["pkill", "-f", "ros2 launch"])
        self.fusion_process = self.acquisition_process = self.attelage_process = (
            self.uart_process
        ) = self.manual_process = None
        self._clear_mode()

    def send_manual_pose(self):
        if not self.manual_process:
            return
        pos_msg = Float32MultiArray()
        pos_msg.data = [
            self.input_x.value(),
            self.input_y.value(),
            self.input_z.value(),
        ]
        self.pos_pub.publish(pos_msg)
        ori_msg = Float32MultiArray()
        ori_msg.data = [
            self.input_roll.value(),
            self.input_pitch.value(),
            self.input_yaw.value(),
        ]
        self.ori_pub.publish(ori_msg)

    # --- Callbacks ROS ---
    def imu_callback(self, msg):
        if len(msg.data) >= 3:
            self._last_imu_t = self._time.time()
            r, p, y = [round(v, 2) for v in msg.data[:3]]
            self.label_imu.setText(f"R: {r:+.2f}°   P: {p:+.2f}°   Y: {y:+.2f}°")

    def fusion_callback(self, msg):
        if len(msg.data) >= 3:
            self._last_fusion_t = self._time.time()
            r, p, y = [round(v, 2) for v in msg.data[:3]]
            self.label_fusion.setText(f"R: {r:+.2f}°   P: {p:+.2f}°   Y: {y:+.2f}°")

    def verins_callback(self, msg):
        if len(msg.data) >= 6:
            self._last_motor_t = self._time.time()
            vals = [round(v, 3) for v in msg.data[:6]]
            self.label_verins.setText(
                "C: " + "  ".join([f"V{i + 1}:{v:.3f} cm" for i, v in enumerate(vals)])
            )
            self.last_consigne = vals

    def feedback_callback(self, msg):
        if len(msg.data) >= 6:
            self._last_motor_t = self._time.time()
            vals = [round(v, 3) for v in msg.data[:6]]
            self.label_feedback.setText(
                "F: " + "  ".join([f"V{i + 1}:{v:.3f} cm" for i, v in enumerate(vals)])
            )
            for i in range(6):
                self.motor_plots[i].update_plot(self.last_consigne[i], vals[i])

    def aruco_callback(self, msg):
        if len(msg.data) >= 3:
            self._last_aruco_t = self._time.time()
            x, y, z = [round(v, 3) for v in msg.data[:3]]
            self.label_aruco.setText(f"Pos:  X={x:+.3f}  Y={y:+.3f}  Z={z:+.3f}")

    def aruco_orientation_callback(self, msg):
        if len(msg.data) >= 3:
            self._last_aruco_t = self._time.time()
            r, p, y = [round(v, 1) for v in msg.data[:3]]
            self.label_aruco_ori.setText(f"Ori:  R={r:+.1f}°  P={p:+.1f}°  Y={y:+.1f}°")

    def image_callback(self, msg):
        try:
            cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)
            self.video_label.setPixmap(QPixmap.fromImage(qt_img))
        except Exception as e:
            print(f"Erreur conversion image: {e}")


class InterfaceNode(Node):
    def __init__(self, gui):
        super().__init__("interface_node_gui")
        self.gui = gui
        self.bridge = CvBridge()
        self.create_subscription(
            Float32MultiArray, "imu_error", self.gui.imu_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "F_orientation", self.gui.fusion_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "stewart/longueurs", self.gui.verins_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "feedback_motors", self.gui.feedback_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "aruco_position", self.gui.aruco_callback, 10
        )
        self.create_subscription(Image, "camera/image_raw", self.gui.image_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            "aruco_orientation",
            self.gui.aruco_orientation_callback,
            10,
        )
        self.gui.pos_pub = self.create_publisher(
            Float32MultiArray, "manual_position", 10
        )
        self.gui.ori_pub = self.create_publisher(
            Float32MultiArray, "manual_orientation", 10
        )
        self.gui.node = self


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = InterfaceGUI()
    node = InterfaceNode(gui)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(30)
    gui.show()
    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
