#!/usr/bin/env python3

"""Tests unitaires pour le module config_loader."""

import os
import tempfile
import unittest

import yaml

from stewart_control.config_loader import load_config, get_config

SAMPLE_CONFIG = {
    "stewart_platform": {
        "radius_base": 0.075,
        "radius_platform": 0.04,
        "gamma_base": 11.3,
        "gamma_platform": 17.3,
        "home_position": [0.0, 0.0, 0.185],
        "L0": [18.86, 18.86, 18.86, 18.86, 18.86, 18.86],
    },
    "serial": {
        "port": "/dev/ttyACM0",
        "baudrate": 115200,
        "timeout": 1,
        "startup_delay": 2.0,
    },
    "actuators": {
        "length_threshold": 1.0,
        "max_displacement": 10.0,
        "feedback_timer_period": 0.05,
        "home_vector": [0.10, 0.10, 0.10, 0.10, 0.10, 0.10],
    },
    "aruco": {
        "dictionary": "DICT_4X4_50",
        "marker_size": 0.022,
        "fixed_marker_id": 34,
        "mobile_marker_id": 28,
        "loop_rate": 0.1,
    },
    "imu": {
        "i2c_bus": 1,
        "imu1_address": 105,
        "imu2_address": 104,
        "calib1_path": "/tmp/calib1.json",
        "calib2_path": "/tmp/calib2.json",
        "publish_rate": 0.1,
        "auto_calib_seconds": 3.0,
    },
    "fusion": {
        "kalman_roll": {"q": 0.02, "r": 1.0},
        "kalman_pitch": {"q": 0.02, "r": 1.0},
        "kalman_yaw": {"q": 0.05, "r": 2.0},
        "alpha_yaw_mean": 0.05,
    },
}


class TestConfigLoader(unittest.TestCase):
    """Tests pour le chargeur de configuration."""

    def setUp(self):
        """Crée un fichier YAML temporaire."""
        self.tmpfile = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        yaml.dump(SAMPLE_CONFIG, self.tmpfile)
        self.tmpfile.close()
        self.config_path = self.tmpfile.name

        # Reset le cache global
        import stewart_control.config_loader as cl

        cl._config_cache = None

    def tearDown(self):
        os.unlink(self.config_path)
        import stewart_control.config_loader as cl

        cl._config_cache = None

    def test_load_config_explicit_path(self):
        """Charge un fichier YAML avec chemin explicite."""
        cfg = load_config(self.config_path)
        self.assertIsInstance(cfg, dict)
        self.assertIn("stewart_platform", cfg)
        self.assertIn("serial", cfg)
        self.assertIn("actuators", cfg)
        self.assertIn("aruco", cfg)
        self.assertIn("imu", cfg)
        self.assertIn("fusion", cfg)

    def test_stewart_platform_params(self):
        """Vérifie les paramètres de la plateforme Stewart."""
        cfg = load_config(self.config_path)
        sp = cfg["stewart_platform"]
        self.assertAlmostEqual(sp["radius_base"], 0.075)
        self.assertAlmostEqual(sp["radius_platform"], 0.04)
        self.assertAlmostEqual(sp["gamma_base"], 11.3)
        self.assertAlmostEqual(sp["gamma_platform"], 17.3)
        self.assertEqual(sp["home_position"], [0.0, 0.0, 0.185])
        self.assertEqual(len(sp["L0"]), 6)

    def test_serial_params(self):
        """Vérifie les paramètres série."""
        cfg = load_config(self.config_path)
        ser = cfg["serial"]
        self.assertEqual(ser["port"], "/dev/ttyACM0")
        self.assertEqual(ser["baudrate"], 115200)

    def test_fusion_params(self):
        """Vérifie les paramètres de fusion Kalman."""
        cfg = load_config(self.config_path)
        fus = cfg["fusion"]
        self.assertAlmostEqual(fus["kalman_roll"]["q"], 0.02)
        self.assertAlmostEqual(fus["kalman_yaw"]["r"], 2.0)
        self.assertAlmostEqual(fus["alpha_yaw_mean"], 0.05)

    def test_get_config_caching(self):
        """Vérifie que get_config met en cache le résultat."""
        cfg1 = get_config(self.config_path)
        cfg2 = get_config()  # Doit retourner le même objet (caché)
        self.assertIs(cfg1, cfg2)

    def test_get_config_force_reload(self):
        """Vérifie que passer un chemin force le rechargement."""
        cfg1 = get_config(self.config_path)
        cfg2 = get_config(self.config_path)
        # Même contenu mais rechargé
        self.assertEqual(cfg1, cfg2)

    def test_load_config_missing_file(self):
        """Vérifie l'erreur si le fichier n'existe pas."""
        with self.assertRaises(FileNotFoundError):
            load_config("/nonexistent/path/config.yaml")

    def test_env_variable_override(self):
        """Vérifie le chargement via STEWART_CONFIG."""
        os.environ["STEWART_CONFIG"] = self.config_path
        try:
            cfg = load_config()
            self.assertIn("stewart_platform", cfg)
        finally:
            del os.environ["STEWART_CONFIG"]

    def test_aruco_params(self):
        """Vérifie les paramètres ArUco."""
        cfg = load_config(self.config_path)
        aruco = cfg["aruco"]
        self.assertEqual(aruco["dictionary"], "DICT_4X4_50")
        self.assertAlmostEqual(aruco["marker_size"], 0.022)
        self.assertEqual(aruco["fixed_marker_id"], 34)
        self.assertEqual(aruco["mobile_marker_id"], 28)

    def test_imu_params(self):
        """Vérifie les paramètres IMU."""
        cfg = load_config(self.config_path)
        imu = cfg["imu"]
        self.assertEqual(imu["i2c_bus"], 1)
        self.assertAlmostEqual(imu["auto_calib_seconds"], 3.0)


if __name__ == "__main__":
    unittest.main()
