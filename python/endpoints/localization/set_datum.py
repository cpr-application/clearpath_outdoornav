from clearpath_config.common.utils.yaml import read_yaml
from clearpath_localization_msgs.srv import SetDatum
import rclpy
from rclpy.node import Node

ROBOT_CONFIG_PATH = '/etc/clearpath/robot.yaml'
LOGGING_NAME = 'SetDatum'

# TODO: Enter your datum latitude/longitude
DATUM_LATITUDE: float =
DATUM_LONGITUDE: float =


class SetDatumCLient(Node):

    def __init__(self, namespace: str):
        super().__init__('set_datum_client')
        self._set_datum_client = self.create_client(SetDatum, f'/{namespace}/localization/set_datum')

        while not self._set_datum_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'[{LOGGING_NAME}] /{namespace}/localization/set_datum service not available, waiting again...')

    def set_datum(self, latitude, longitude):
        req = SetDatum.Request()
        req.lat = latitude
        req.lon = longitude
        self.future = self._set_datum_client.call_async(req)
        self.future.add_done_callback(self.set_datum_srv_response_cb)

    def set_datum_srv_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'[{LOGGING_NAME}] Set datum successfully!')
            else:
                self.get_logger().error(f'[{LOGGING_NAME}] Failed to set datum')
        except Exception as e:
            self.get_logger().error(f'[{LOGGING_NAME}] Service call failed: {e}')

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    robot_config = read_yaml(ROBOT_CONFIG_PATH)
    namespace = robot_config['system']['ros2']['namespace']

    set_datum_client = SetDatumCLient(namespace)
    set_datum_client.set_datum(DATUM_LATITUDE, DATUM_LONGITUDE)

    rclpy.spin(set_datum_client)


if __name__ == '__main__':
    main()
