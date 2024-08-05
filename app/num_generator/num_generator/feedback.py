"""" This node is used to provide feedback to the generator
if a condition is fulfilled"""
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
from interfaces.msg import BoolWithHeader, IntWithHeader


class Feedback(Node):

    def __init__(self):
        super().__init__('Feedback')
        self.reset_signal = True
        self.subscription = self.create_subscription(
            IntWithHeader, 'multiplier2feedback', self.num_feedback, 1)
        self.publisher = self.create_publisher(BoolWithHeader, 'feedback2generator', 1)

    def num_feedback(self, msg):
        # create message
        bool_msg = BoolWithHeader()

        # header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        bool_msg.header = header

        # data
        if msg.data < 10:
            bool_msg.data = self.reset_signal = False  # if condition not met set to False
        else:
            bool_msg.data = self.reset_signal = True  # if condition met set to True

        print(bool_msg.data)

        # publish
        self.publisher.publish(bool_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Feedback()
    try:
        # sleep statement to spin the node after 0.2 seconds
        time.sleep(0.2)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
