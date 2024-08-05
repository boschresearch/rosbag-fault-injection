"""" This node is used to multiply the number by a factor of 2 """
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


class NumberMultiplier(Node):

    def __init__(self):
        super().__init__('NumberMultiplier')
        self.subscription = self.create_subscription(
            IntWithHeader, 'generator2multiplier', self.num_multiplier, 1)
        self.publisher = self.create_publisher(IntWithHeader, 'multiplier2feedback', 1)

    def num_multiplier(self, msg):
        # create message
        new_msg = IntWithHeader()

        # header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        new_msg.header = header

        # data
        new_msg.data = self.z = msg.data * 2
        
        print(new_msg.data)
        
        # publish
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberMultiplier()
    try:
        # sleep statement to spin the node after 0.2 seconds
        time.sleep(0.2)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
