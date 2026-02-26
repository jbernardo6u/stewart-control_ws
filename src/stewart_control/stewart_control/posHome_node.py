#!/home/rem/rtimulib-env/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
from stewart_control.config_loader import get_config


class PosHomeNode(Node):
    def __init__(self):
        super().__init__("pos_home_node")

        cfg = get_config()
        ser_cfg = cfg["serial"]
        act = cfg["actuators"]

        # Publisher pour le feedback
        self.publisher_feedback = self.create_publisher(
            Float32MultiArray, "feedback_motors", 10
        )

        # Configuration UART
        self.ser = serial.Serial(
            ser_cfg["port"], ser_cfg["baudrate"], timeout=ser_cfg["timeout"]
        )
        time.sleep(ser_cfg["startup_delay"])
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.get_logger().info("Port UART ouvert et prêt.")

        # Vecteur à envoyer
        self.vecteur = act["home_vector"]

        # Timer pour envoi régulier
        self.create_timer(act["feedback_timer_period"], self.send_vecteur)

    def send_vecteur(self):
        message = ",".join([f"{v:.2f}" for v in self.vecteur]) + "\n"
        try:
            self.ser.write(message.encode())
            self.get_logger().info(f"Vecteur envoyé : {message.strip()}")

            # Lecture du feedback et publication
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    try:
                        feedback = [
                            float(x) for x in line.split(",") if x.strip() != ""
                        ]
                        if len(feedback) == 6:
                            msg = Float32MultiArray()
                            msg.data = feedback
                            self.publisher_feedback.publish(msg)
                            self.get_logger().info(f"Feedback publié : {feedback}")
                        else:
                            self.get_logger().warn(
                                f"Ligne série incomplète ignorée : {line}"
                            )
                    except ValueError:
                        self.get_logger().warn(f"Ligne série invalide : {line}")

        except Exception as e:
            self.get_logger().error(f"Erreur UART : {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PosHomeNode()
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
