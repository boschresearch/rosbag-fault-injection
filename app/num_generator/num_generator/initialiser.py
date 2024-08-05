""" This node initliases the value of  y and publishes
it depending on the condition"""
#!/usr/bin/env python3
#
# Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries.
# This program and the accompanying materials are made available under
# the terms of the Bosch Internal Open Source License v4
# which accompanies this distribution, and is available at
# http://bios.intranet.bosch.com/bioslv4.txt

__author__ = "Min Hee Jo"
__copyright__ = "Copyright 2024, Robert Bosch GmbH"
__license__ = "BIOSL"
__version__ = "4.0"
__email__ = "minhee.jo@de.bosch.com"

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Header
from interfaces.msg import IntWithHeader


class Initialiser(Node):

    def __init__(self):
        super().__init__('Initialiser')
        self.publisher_generator = self.create_publisher(IntWithHeader, 'initializer2generator', 1)
        self.subscription = self.create_subscription(
            IntWithHeader, 'generator2initializer', self.num_initialiser, 1)
        # initialising variables
        self.y = 1

    def num_initialiser(self, msg):
        # create message
        y_value = IntWithHeader()

        # header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        y_value.header = header

        # data
        if msg.data < 0:
            y_value.data = self.y = 1
        else:
            y_value.data = self.y = self.y - msg.data

        print(self.y)

        # publish
        self.publisher_generator.publish(y_value)


def main(args=None):
    rclpy.init(args=args)
    node = Initialiser()
    try:
        # sleep statement to spin the node after 0.2 seconds
        time.sleep(0.2)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
