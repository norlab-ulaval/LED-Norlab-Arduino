#| /usr/bin/env python3

import rclpy

from std_msgs.msg import String

from rclpy.node import Node

import serial;

class led_test(Node):


    def __init__(self):

        super().__init__('color_test')
        timer_period = 0.1
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
    def sub(self):
        print("sub bro")

    def timer_callback(self):
        print("mo cul")        


        

        
        
        
        

def main():
    rclpy.init()
    node = led_test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()


        
        




