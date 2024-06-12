import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import logging

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie

# ros2 topic pub /topic std_msgs/msg/String '{data: "hello"}''
# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

logging.basicConfig(level=logging.ERROR)

class CrazyflieSubscriber(Node):

    def __init__(self, link_uri):

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.open_link(link_uri)

        super().__init__('crazyflie_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self._listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.fixed_height = 0.3

    def _listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        self._cf.commander.send_hover_setpoint(msg.linear.x, msg.linear.y, msg.angular.z,  self.fixed_height)


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