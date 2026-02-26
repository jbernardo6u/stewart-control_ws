#!/home/rem/rtimulib-env/bin/python3

import numpy as np
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from stewart_control.inv_kinematics import StewartPlatform
from stewart_control.config_loader import get_config


class StewartNode(Node):
    def __init__(self):
        super().__init__("stewart_node")

        cfg = get_config()
        sp = cfg["stewart_platform"]
        ser_cfg = cfg["serial"]
        act = cfg["actuators"]

        # Publisher pour les longueurs calculées à envoyer à l'Arduino
        self.publisher_ = self.create_publisher(
            Float32MultiArray, "stewart/longueurs", 10
        )

        # Publisher pour le feedback reçu de l'Arduino
        self.publisher_feedback = self.create_publisher(
            Float32MultiArray, "feedback_motors", 10
        )

        # Subscribers
        self.pos_sub = self.create_subscription(
            Float32MultiArray, "manual_position", self.pos_callback, 10
        )
        self.ori_sub = self.create_subscription(
            Float32MultiArray, "manual_orientation", self.ori_callback, 10
        )

        # Configuration UART vers Arduino
        self.ser = serial.Serial(
            ser_cfg["port"], ser_cfg["baudrate"], timeout=ser_cfg["timeout"]
        )
        time.sleep(ser_cfg["startup_delay"])

        # Vider le buffer série
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # Configuration de la plateforme Stewart
        self.platform = StewartPlatform(
            sp["radius_base"],
            sp["radius_platform"],
            sp["gamma_base"],
            sp["gamma_platform"],
            home_position=sp["home_position"],
        )
        self.L0 = np.array(sp["L0"])

        # Variables d’état
        self.position = np.zeros(3)
        self.orientation = np.zeros(3)
        self.continuous = False
        self.last_deplacement = [0.0] * 6
        self.length_threshold = act["length_threshold"]
        self.max_displacement = act["max_displacement"]

        # Timer pour lecture série
        self.create_timer(act["feedback_timer_period"], self.read_feedback)

    def pos_callback(self, msg):
        if len(msg.data) >= 3:
            self.position = np.array(msg.data)
            self.update_actuators()

    def ori_callback(self, msg):
        if len(msg.data) >= 3:
            self.orientation = np.array(msg.data)
            self.update_actuators()

    def update_actuators(self):
        rotation = self.orientation
        trans = self.position

        longueurs_m = self.platform.solve(trans, rotation)
        longueurs_cm = longueurs_m * 100
        deplacement = longueurs_cm - self.L0

        send_update = any(
            abs(c - l) >= self.length_threshold
            for c, l in zip(deplacement, self.last_deplacement)
        )

        if send_update:
            out_msg = Float32MultiArray()
            out_msg.data = deplacement.tolist()
            self.publisher_.publish(out_msg)

            # Vérification avant envoi UART : aucune valeur ne doit dépasser 10cm

            if all(abs(d) <= self.max_displacement for d in deplacement):

                consigne = ",".join([f"{d:.2f}" for d in deplacement])
                self.ser.write((consigne + "\n").encode())
                self.get_logger().info(f"Consigne envoyée : {consigne}")

            else:
                self.get_logger().warn(
                    f"Valeur trop grande, envoi UART annulé : {deplacement}"
                )

            self.last_deplacement = deplacement.copy()

            if not self.continuous:
                self.get_logger().info("Consigne envoyée une fois, arrêt du callback.")

    def read_feedback(self):
        """Lecture du feedback depuis Arduino et publication ROS2"""
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors="ignore").strip()

            if not line or "," not in line:
                continue

            try:
                feedback = [float(x) for x in line.split(",") if x.strip() != ""]

                if len(feedback) != 6:
                    self.get_logger().warn(f"Ligne série incomplète ignorée : {line}")
                    continue

                msg = Float32MultiArray()
                msg.data = feedback
                self.publisher_feedback.publish(msg)
                self.get_logger().info(f"Feedback reçu : {feedback}")

            except ValueError:
                self.get_logger().warn(f"Ligne série invalide : {line}")


def main(args=None):
    rclpy.init(args=args)
    node = StewartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
