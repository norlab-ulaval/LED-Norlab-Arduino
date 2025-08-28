#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from clearpath_platform_msgs.msg import Lights
from std_msgs.msg import Bool

import serial

class WarthogArduinoInterface(Node):

    def __init__(self):
        super().__init__('warthog_arduino_interface')

        # Serial connection setup
        self.arduino = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=2
        )

        self.estop_bool = True
        self.cntr = 0

        self.estop_publisher = self.create_publisher(Bool, '/warthog/platform/emergency_stop', 10)

        self.lights_subscription = self.create_subscription(
            Lights,
            '/cmd_lights',
            self.lights_callback,
            10
        )

        # Timer for reading estop state
        self.estop_timer = self.create_timer(0.01, self.estop_state_publisher)

    def lights_callback(self, msg):
        data = []
        for item in msg.lights:
            data.extend([item.red, item.green, item.blue])

        log = ','.join([str(int(d * 8)) for d in data]) + "\n"
        self.arduino.write(log.encode('utf-8'))

    def estop_state_publisher(self):
        msg = Bool()
        line = self.arduino.readline().strip()

        if line == b'1':
            self.estop_bool = True
            self.cntr = 0
        #Filtering the false readings
        else:
            self.cntr += 1
            if self.cntr >= 5:
                self.estop_bool = False
                self.cntr = 0

        msg.data = self.estop_bool
        self.estop_publisher.publish(msg)

def main():
    rclpy.init()
    node = WarthogArduinoInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
