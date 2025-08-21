#| /usr/bin/env python3

import rclpy

from clearpath_platform_msgs.msg import Lights
from std_msgs.msg import Bool

from rclpy.node import Node

import serial

import time

class estop_pub(Node):


    def __init__(self):

        super().__init__('estate_state_publisher')
        
        self.arduino = serial.Serial()
        self.arduino.port = '/dev/ttyUSB0'
        self.arduino.baudrate = 115200
        self.arduino.timeout = 2
        self.arduino.open()
        #self.initialise_link()
        self.estop_bool = True
        self.publisher_ = self.create_publisher(Bool, '/warthog/platform/emergency_stop', 10)
        self.timer = self.create_timer(0.01, self.estop_state_publisher)
        self.cntr = 0
    
    def estop_state_publisher(self):
        msg = Bool()

        line = self.arduino.readline().strip()
            
        if (line == b'1'):
            self.estop_bool = True
            self.cntr = 0
        else:
            self.cntr += 1
            if (self.cntr == 5):
                self.estop_bool = False
                self.cntr = 0
                
        msg.data = self.estop_bool
        # self.get_logger().info(str(msg.data))
        self.publisher_.publish(msg)



def main():
    rclpy.init()
    node = estop_pub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()