import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from influxdb_client import InfluxDBClient, Point

class InfluxNode(Node):
    def __init__(self):
        super().__init__('influx_node')

        # ---- CONFIG INFLUX ----
        self.token = "NIuEJwL_3sdzCMFRnlJW4AH4ZhsgHgxVhCFXcWl1eLnUeSkqnCk-FxuKf5IkxnJEH_qeD9l0eF_u_Q1UI06XYQ=="
        self.org = "sebas"
        self.bucket = "gas_data"

        self.client = InfluxDBClient(
            url="http://localhost:8086",
            token=self.token,
            org=self.org
        )

        self.write_api = self.client.write_api()

        # ---- SUBSCRIBER ----
        self.subscription = self.create_subscription(
            Float32,
            '/gas_concentration',
            self.callback,
            10
        )

    def callback(self, msg):
        point = (
            Point("sensor")
            .tag("robot_id", "sim_robot_1")
            .tag("sensor_type", "gas")
            .field("value", float(msg.data))
        )

        self.write_api.write(
            bucket=self.bucket,
            org=self.org,
            record=point
        )

        self.get_logger().info(f"Valor guardado: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = InfluxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
