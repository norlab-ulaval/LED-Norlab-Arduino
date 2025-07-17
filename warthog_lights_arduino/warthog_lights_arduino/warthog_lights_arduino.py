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
        self.arduino.port = '/dev/ttyACM0'
        self.arduino.baudrate = 115200
        self.arduino.timeout = 1
        self.arduino.open()
        #self.initialise_link()
        self.estop_bool = 1
        self.reset_cntr = 0
        self.publisher_ = self.create_publisher(Bool, '/emergency_stop', 10)
        self.timer = self.create_timer(0.01, self.estop_state_publisher)
        # self.subscription = self.create_subscription(
        #     Lights,
        #     '/cmd_lights',
        #     self.callback,  
        #     10)
        

    #def initialise_link(self):
    #    cntr = 0
   #     while cntr < 10:
  #          self.arduino.write("boot\n".encode('utf-8'))
 #           cntr+=1
#            time.sleep(0.1)

    def estop_state_publisher(self):
        self.reset_cntr +=1
        msg = Bool()

        if self.arduino.in_waiting:
        #     self.estop_bool = self.arduino.readline().decode('utf-8').strip()
            line = self.arduino.readline().strip()
            self.estop_bool = (line == b'1')

        msg.data = self.estop_bool
        #self.get_logger().info(str(msg.data))
        self.publisher_.publish(msg)

        if(self.reset_cntr > 100):
            self.arduino.reset_input_buffer()
            self.reset_cntr = 0


    # def callback(self, msg):

    #     #self.data = []
    #     #for item in msg.lights:
    #     #    self.data.extend([item.red, item.green, item.blue])
            
    #     #log = ','.join([str(d*8) for d in self.data])
    #     # log += "\n"
    #     # self.get_logger().info(log)
    #     if ser.in_waiting > 0:
    #         self.estop_state_publisher
    #     # self.arduino.write(log.encode('utf-8'))      



def main():
    rclpy.init()
    node = led_test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()


