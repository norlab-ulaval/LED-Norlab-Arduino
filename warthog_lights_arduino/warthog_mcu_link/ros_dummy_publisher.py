#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from clearpath_platform_msgs.msg import Lights, RGB
import serial


class Publisher_Usb(Node):


    def __init__(self):
        super().__init__('basic')
        timer_period = 0.5
        self.publisher_ = self.create_publisher(Lights, '/cmd_lights', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.arduino = serial.Serial('/dev/ttyACM0', 9600)

    def timer_callback(self):
        msg = Lights()
        colors_0 = RGB()
        colors_0.red = 0
        colors_0.green = 0
        colors_0.blue = 255
         
        colors_1 = RGB()
        colors_1.red = 0
        colors_1.green = 0
        colors_1.blue = 255
                
        colors_2 = RGB()
        colors_2.red = 0
        colors_2.green = 0
        colors_2.blue = 255

        colors_3 = RGB()    
        colors_3.red = 0
        colors_3.green = 0
        colors_3.blue = 255

        msg.lights = [colors_0,colors_1,colors_2,colors_3]
        
        self.publisher_.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    basic = Publisher_Usb()

    rclpy.spin(basic)

    basic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

