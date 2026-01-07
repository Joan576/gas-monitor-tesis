#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class GasSensor(Node):
    def __init__(self):
        super().__init__('gas_sensor')
        self.publisher_ = self.create_publisher(Float32, 'gas_concentration', 10)
        self.timer = self.create_timer(1.0, self.publish_value)

    def publish_value(self):
        value = random.uniform(60.0, 130.0)   # valor falso de gas
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Gas sensor reading: {value:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = GasSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
