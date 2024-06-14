# ROS 2 imports
import rclpy
from rclpy.node import Node

#msg imports
from std_msgs.msg import String
from geometry_msgs.msg import Point32

#Crazyflie imports
import logging
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

logging.basicConfig(level=logging.ERROR)

class CrazyfliePublisher(Node):
    def __init__(self, link_uri):

        # Crazyflie open link
        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.open_link(link_uri)

        #ros2 specific
        super().__init__('crazyflie_publisher')
        self.publisher_ = self.create_publisher(Point32, 'topic', 10)

    # crazyflie connected
    def _connected(self, link_uri):

        self.get_logger().info('Connected to %s' % link_uri)

        self._lg_pos = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_pos.add_variable('stateEstimate.x', 'float')
        self._lg_pos.add_variable('stateEstimate.y', 'float')
        self._lg_pos.add_variable('stateEstimate.z', 'float')
        self._cf.log.add_config(self._lg_pos)
        self._lg_pos.data_received_cb.add_callback(self._pos_log_data)
        self._lg_pos.start()

    # log pos call back, and it sends out the topic
    def _pos_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        #msg = String()
        #msg.data = 'Hello! I am %s and I have data from timestamp %d!' % (self._cf.link_uri, timestamp)
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

        msg = Point32()
        msg.x = data['stateEstimate.x']
        msg.y = data['stateEstimate.y']
        msg.z = data['stateEstimate.z']
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    cflib.crtp.init_drivers()
    uri = "radio://0/40/2M/E7E7E7E704"

    rclpy.init(args=args)
    crazyflie_publisher = CrazyfliePublisher(uri)
    rclpy.spin(crazyflie_publisher)
    crazyflie_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()