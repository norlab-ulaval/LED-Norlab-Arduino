#| /usr/bin/env python3

import rclpy

from clearpath_platform_msgs.msg import Lights
from std_msgs.msg import Bool

from rclpy.node import Node

import serial

import time

class led_test(Node):


    def __init__(self):

        super().__init__('color_test')
        
        self.arduino = serial.Serial()
        self.arduino.port = '/dev/ttyUSB0'
        self.arduino.baudrate = 115200
        self.arduino.timeout = 2
        self.arduino.open()
        self.estop_bool = True
        self.publisher_ = self.create_publisher(Bool, '/warthog/platform/emergency_stop', 10)
        self.cntr = 0
        self.subscription = self.create_subscription(
            Lights,
            '/cmd_lights',
            self.callback,  
            10)
        

    def callback(self, msg):
        
        self.data = []
        for item in msg.lights:
           self.data.extend([item.red, item.green, item.blue])
            
        log = ','.join([str(d*8) for d in self.data])
        log += "\n"
        self.get_logger().info(log)
        self.arduino.write(log.encode('utf-8'))
     

def main():
    rclpy.init()
    node = led_test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()


