import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class GasPublisher(Node):
    def __init__(self):
        super().__init__('gas_publisher')

        # Publicaremos en el tópico /gas_concentration
        self.publisher_ = self.create_publisher(Float32, 'gas_concentration', 10)

        # Timer para publicar cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_gas_value)

        self.get_logger().info("Gas publisher node started.")

    def publish_gas_value(self):
        # Simulación de un valor de concentración de gas (ppm)
        value = random.uniform(10.0, 50.0)
        msg = Float32()
        msg.data = value

        self.publisher_.publish(msg)
        self.get_logger().info(f"Gas concentration: {value:.2f} ppm")

def main(args=None):
    rclpy.init(args=args)
    node = GasPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
