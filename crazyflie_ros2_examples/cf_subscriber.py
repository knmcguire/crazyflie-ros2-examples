import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import logging

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie

##ros2 topic pub /topic std_msgs/msg/String '{data: "hello"}'

logging.basicConfig(level=logging.ERROR)

class CrazyflieSubscriber(Node):

    def __init__(self, link_uri):

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.open_link(link_uri)

        super().__init__('crazyflie_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self._listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def _listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def _connected(self, link_uri):
        self.get_logger().info('Connected to %s' % link_uri)


def main(args=None):
    cflib.crtp.init_drivers()
    uri = "radio://0/40/2M/E7E7E7E704"

    rclpy.init(args=args)

    crazyflie_subscriber = CrazyflieSubscriber(uri)

    rclpy.spin(crazyflie_subscriber)

    crazyflie_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()