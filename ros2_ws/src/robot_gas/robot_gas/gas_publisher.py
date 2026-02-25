import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import random
import json


class GasPublisher(Node):
    def __init__(self):
        super().__init__('gas_publisher')

        self.concentration_pub = self.create_publisher(Float32, 'gas_concentration', 10)
        self.sample_pub = self.create_publisher(String, 'gas_sample_xy', 10)

        # Definimos la grilla de patrullaje
        self.x_min, self.x_max, self.x_step = 0.0, 10.0, 0.5
        self.y_min, self.y_max, self.y_step = 0.0, 10.0, 0.5

        # Estado del “robot”
        self.current_x = self.x_min
        self.current_y = self.y_min
        self.direction = 1  # 1: x crece, -1: x decrece

        self.timer = self.create_timer(0.5, self.publish_values)
        self.get_logger().info("Gas publisher node started with sweep pattern.")

    def update_position(self):
        # Avanza en X según la dirección actual
        self.current_x += self.direction * self.x_step

        # Si se sale del rango, cambia de fila (Y) y revierte dirección
        if self.current_x > self.x_max or self.current_x < self.x_min:
            # Corrige X para que quede dentro
            self.current_x = max(min(self.current_x, self.x_max), self.x_min)

            # Avanza una fila en Y
            self.current_y += self.y_step

            # Si se sale en Y, reinicia al inicio
            if self.current_y > self.y_max:
                self.current_y = self.y_min

            # Cambia dirección en X (efecto zigzag)
            self.direction *= -1

    def simulate_concentration(self, x, y):
        # Ejemplo: concentración más alta en una “zona caliente” alrededor de (5,5)
        base = 30.0
        hotspot = 60.0 * max(0.0, 1.0 - ((x - 5.0) ** 2 + (y - 5.0) ** 2) / 16.0)
        noise = random.uniform(-5.0, 5.0)
        return max(0.0, base + hotspot + noise)

    def publish_values(self):
        # Actualiza posición simulada
        self.update_position()

        # Calcula concentración “realista”
        concentration = self.simulate_concentration(self.current_x, self.current_y)

        # 1) Publicar solo concentración (para la gráfica temporal)
        conc_msg = Float32()
        conc_msg.data = concentration
        self.concentration_pub.publish(conc_msg)

        # 2) Publicar muestra completa como JSON
        sample = {
            "x": self.current_x,
            "y": self.current_y,
            "concentration": concentration
        }
        sample_msg = String()
        sample_msg.data = json.dumps(sample)
        self.sample_pub.publish(sample_msg)

        self.get_logger().info(
            f"Gas: {concentration:.2f} ppm | x: {self.current_x:.2f}, y: {self.current_y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GasPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
