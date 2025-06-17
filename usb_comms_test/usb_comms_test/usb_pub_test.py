#| /usr/bin/env python3

import rclpy

from clearpath_platform_msgs.msg import Lights

from rclpy.node import Node

import serial;

class led_test(Node):


    def __init__(self):

        super().__init__('color_test')
        
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        
        self.initialise_link()
        self.subscription = self.create_subscription(
            Lights,
            'platform/cmd_lights',
            self.callback,
            10)
        

    def initialise_link(self):
        comms = False
        while comms == False:
            if self.in_waiting > 0:
                if self.arduino.read(1).decode('utf-8') == 'A':
                    comms = True
            else:
                self.arduino.write("boot\n".encode('utf-8'))                 


    def callback(self, msg):
        self.data = []
        for item in msg.data.lights:
            self.data.extend([item.red,item.green, item.blue])
            
        log = f"{','.join(self.data)}\n"
        self.get_logger().info(log)
        #self.arduino.write(log.encode('utf-8'))      



def main():
    rclpy.init()
    node = led_test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":

    main()


        
        




