import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from influxdb_client import InfluxDBClient, Point
import json


class InfluxXYNode(Node):
    def __init__(self):
        super().__init__('influx_xy_node')

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
            String,
            '/gas_sample_xy',
            self.callback,
            10
        )

        self.get_logger().info("InfluxXYNode started, listening on /gas_sample_xy")

    def callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            concentration = float(data.get("concentration", 0.0))

            point = (
                Point("sensor_xy")
                .tag("robot_id", "sim_robot_1")
                .tag("sensor_type", "gas_xy")
                .field("x", x)
                .field("y", y)
                .field("concentration", concentration)
            )

            self.write_api.write(
                bucket=self.bucket,
                org=self.org,
                record=point
            )

            self.get_logger().info(
                f"XY guardado -> Gas: {concentration:.2f} ppm | x: {x:.2f}, y: {y:.2f}"
            )
        except Exception as e:
            self.get_logger().error(f"Error procesando mensaje JSON: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = InfluxXYNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
