#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from clearpath_platform_msgs.msg import Lights, RGB
import serial


class Publisher_Usb(Node):


    def __init__(self):
        super().__init__('basic')
        timer_period = 0.1
        self.publisher_ = self.create_publisher(Lights, 'platform/cmd_lights', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)

    def timer_callback(self):
        msg = Lights
        
        R = 1000
        G = 1000
        B = 1000
         
        msg.data.lights = [RGB(R,G,B),RGB(R,G,B),RGB(R,G,B),RGB(R,G,B)]
        self.publisher_.publish(msg.data)

        self.i += 1




def main(args=None):

    rclpy.init(args=args)
    basic = Publisher_Usb()

    rclpy.spin(basic)

    basic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

