import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import logging
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

logging.basicConfig(level=logging.ERROR)

class CrazyfliePublisher(Node):

    def __init__(self, link_uri):

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.open_link(link_uri)

        super().__init__('crazyflie_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def _connected(self, link_uri):

        self.get_logger().info('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_pos = LogConfig(name='Stabilizer', period_in_ms=1000)
        self._lg_pos.add_variable('stateEstimate.x', 'float')
        self._lg_pos.add_variable('stateEstimate.y', 'float')
        self._lg_pos.add_variable('stateEstimate.z', 'float')
        self._cf.log.add_config(self._lg_pos)
        self._lg_pos.data_received_cb.add_callback(self._pos_log_data)
        self._lg_pos.start()

    def _pos_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        msg = String()
        msg.data = 'Hello! I am %s and I have data from timestamp %d!' % (self._cf.link_uri, timestamp)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        #print(f'[{timestamp}][{logconf.name}]: ', end='')
        #for name, value in data.items():
        #    print(f'{name}: {value:3.3f} ', end='')
        #print()


def main(args=None):
    cflib.crtp.init_drivers()

    rclpy.init(args=args)

    uri = "radio://0/40/2M/E7E7E7E704"
    crazyflie_publisher = CrazyfliePublisher(uri)

    rclpy.spin(crazyflie_publisher)

    crazyflie_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()