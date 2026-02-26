#!/usr/bin/env python3
"""
Standalone demo of the Stewart Platform GUI — no ROS2 required.

Usage:
    python src/stewart_control/test/demo_interface.py

Simulates live sensor data (ArUco, IMU, Fusion, Motors) so you can
visually validate the interface layout, theme, LEDs and graphs.
"""

import sys
import os
import math
import random
import time as _time

# ── Patch: mock ROS2 imports before importing the real module ──────────
# This lets us run InterfaceGUI without rclpy / sensor_msgs / cv_bridge.
import types

# Minimal mock for rclpy
rclpy_mock = types.ModuleType("rclpy")
rclpy_node = types.ModuleType("rclpy.node")


class _FakeNode:
    def __init__(self, *a, **kw):
        pass

    def create_subscription(self, *a, **kw):
        pass

    def create_publisher(self, *a, **kw):
        return type("P", (), {"publish": lambda s, m: None})()

    def destroy_node(self):
        pass


rclpy_node.Node = _FakeNode
rclpy_mock.node = rclpy_node
rclpy_mock.init = lambda *a, **kw: None
rclpy_mock.shutdown = lambda *a, **kw: None
rclpy_mock.spin_once = lambda *a, **kw: None
sys.modules["rclpy"] = rclpy_mock
sys.modules["rclpy.node"] = rclpy_node

# Minimal mock for std_msgs
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")


class _FakeMsg:
    def __init__(self):
        self.data = []


std_msgs_msg.Float32MultiArray = _FakeMsg
std_msgs_msg.String = _FakeMsg
std_msgs.msg = std_msgs_msg
sys.modules["std_msgs"] = std_msgs
sys.modules["std_msgs.msg"] = std_msgs_msg

# Minimal mock for sensor_msgs
sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")


class _FakeImage:
    pass


sensor_msgs_msg.Image = _FakeImage
sensor_msgs.msg = sensor_msgs_msg
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

# Minimal mock for cv_bridge
cv_bridge_mod = types.ModuleType("cv_bridge")


class _FakeBridge:
    def imgmsg_to_cv2(self, *a, **kw):
        return None


cv_bridge_mod.CvBridge = _FakeBridge
sys.modules["cv_bridge"] = cv_bridge_mod

# ── Now safe to import the real GUI ───────────────────────────────────
# Add package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "stewart_control"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from stewart_control.interface_node import InterfaceGUI  # noqa: E402

from PySide6.QtWidgets import QApplication  # noqa: E402
from PySide6.QtCore import QTimer  # noqa: E402


class DemoRunner:
    """Feeds simulated data into the GUI at ~20 Hz."""

    def __init__(self, gui: InterfaceGUI):
        self.gui = gui
        self.t0 = _time.time()
        self.tick = 0

    def step(self):
        t = _time.time() - self.t0
        self.tick += 1

        # ── Simulated IMU data ──
        msg_imu = _FakeMsg()
        msg_imu.data = [
            round(2.5 * math.sin(t * 0.8) + random.gauss(0, 0.1), 3),
            round(1.8 * math.cos(t * 0.6) + random.gauss(0, 0.1), 3),
            round(0.5 * math.sin(t * 0.3) + random.gauss(0, 0.05), 3),
        ]
        self.gui.imu_callback(msg_imu)

        # ── Simulated ArUco position ──
        msg_aruco = _FakeMsg()
        msg_aruco.data = [
            round(0.048 + 0.005 * math.sin(t * 0.5), 4),
            round(-0.008 + 0.003 * math.cos(t * 0.4), 4),
            round(0.009 + 0.002 * math.sin(t * 0.7), 4),
        ]
        self.gui.aruco_callback(msg_aruco)

        # ── Simulated ArUco orientation ──
        msg_aruco_ori = _FakeMsg()
        msg_aruco_ori.data = [
            round(2.2 + 0.5 * math.sin(t * 0.8), 1),
            round(2.9 + 0.3 * math.cos(t * 0.6), 1),
            round(-4.0 + 0.8 * math.sin(t * 0.4), 1),
        ]
        self.gui.aruco_orientation_callback(msg_aruco_ori)

        # ── Simulated Fusion ──
        msg_fusion = _FakeMsg()
        msg_fusion.data = [
            round(2.3 * math.sin(t * 0.8), 2),
            round(1.5 * math.cos(t * 0.6), 2),
            round(-0.4 * math.sin(t * 0.3), 2),
        ]
        self.gui.fusion_callback(msg_fusion)

        # ── Simulated motor consignes ──
        base = [6.1, 5.8, 8.3, 7.9, 6.8, 7.5]
        msg_verins = _FakeMsg()
        msg_verins.data = [
            round(b + 0.3 * math.sin(t * 0.5 + i * 0.5), 3) for i, b in enumerate(base)
        ]
        self.gui.verins_callback(msg_verins)

        # ── Simulated motor feedback (follows consigne with lag + noise) ──
        msg_fb = _FakeMsg()
        lag = 0.4
        msg_fb.data = [
            round(
                b + 0.3 * math.sin((t - lag) * 0.5 + i * 0.5) + random.gauss(0, 0.02),
                3,
            )
            for i, b in enumerate(base)
        ]
        self.gui.feedback_callback(msg_fb)


def main():
    app = QApplication(sys.argv)
    gui = InterfaceGUI()

    # Disable subprocess buttons (no ROS2 in demo mode)
    gui.button_start_acq.setEnabled(False)
    gui.button_start_att.setEnabled(False)
    gui.button_manual.setEnabled(False)
    gui.button_uart.setEnabled(False)
    gui.button_stop.setEnabled(False)

    # Show demo mode indicator
    gui.mode_label.setText("● DEMO MODE")
    gui.mode_label.setStyleSheet("color: #faa61a; font-size: 10px; font-weight: bold;")

    runner = DemoRunner(gui)

    timer = QTimer()
    timer.timeout.connect(runner.step)
    timer.start(50)  # ~20 Hz

    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
