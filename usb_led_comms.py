#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial


class Publisher_Usb(Node):


    def __init__(self):
        super().__init__('basic')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)

    def timer_callback(self):
        msg = String()
        
        R = input("enter R component : ")
        G = input("enter G component : ")
        B = input("enter B component : ")
         
        msg.data = f"{R},{G},{B},{R},{G},{B},{R},{G},{B},{R},{G},{B}\n"

       

        self.arduino.write(msg.data.encode('utf-8'))        

        self.i += 1




def main(args=None):

    rclpy.init(args=args)
    basic = Publisher_Usb()

    rclpy.spin(basic)

    basic.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
