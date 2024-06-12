import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger

import logging

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie

##ros2 topic pub /topic std_msgs/msg/String '{data: "hello"}'

logging.basicConfig(level=logging.ERROR)

class CrazyflieService(Node):

    def __init__(self, link_uri):

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.open_link(link_uri)

        super().__init__('crazyflie_service')
        self.create_service(Trigger, 'takeoff', self._take_off_callback)
        self.create_service(Trigger, 'land', self._land_callback)


    def _take_off_callback(self, request, response):
        self.get_logger().info('Taking off!')
        self._cf.high_level_commander.takeoff(0.5, 1.0)
        response.success = True
        return response
    
    def _land_callback(self, request, response):
        self.get_logger().info('Landing!')
        self._cf.high_level_commander.land(0.0, 1.0)
        response.success = True
        return response
    
    def _connected(self, link_uri):
        self.get_logger().info('Connected to %s' % link_uri)


def main(args=None):
    cflib.crtp.init_drivers()
    uri = "radio://0/40/2M/E7E7E7E704"

    rclpy.init(args=args)

    crazyflie_service = CrazyflieService(uri)

    rclpy.spin(crazyflie_service)

    crazyflie_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()