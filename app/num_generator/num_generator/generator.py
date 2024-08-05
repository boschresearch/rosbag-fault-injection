"""" This node is used to publish the numbers on two different topics """
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


class NumberGenerator(Node):

    def __init__(self):
        super().__init__('NumberGenerator')
        self.publisher_multiplier = self.create_publisher(IntWithHeader, 'generator2multiplier', 1)
        self.publisher_initialiser = self.create_publisher(IntWithHeader, 'generator2initializer', 1)
        self.subscription = self.create_subscription(
            BoolWithHeader, 'feedback2generator', self.num_feedback, 1)
        self.subscription = self.create_subscription(
            IntWithHeader, 'initializer2generator', self.num_y_generator, 1)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 0.05 seconds can be changed by user
        # initialising variables
        self.x = 1
        self.initialiser2generator = 0

    def timer_callback(self):  # Timer callback for publishing x values
        # create message
        msg = IntWithHeader()

        # header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        msg.header = header

        # data
        msg.data = self.x
        
        print(msg.data)

        # publish
        self.publisher_multiplier.publish(msg)
        self.publisher_initialiser.publish(msg)

        # update
        self.x += 1

    def num_feedback(self, msg):  # Bool output
        if msg.data is True:
            self.x = self.initialiser2generator
            print("The value of x is:", self.x)

    def num_y_generator(self, msg_y):  # Int32 Output
        self.initialiser2generator = msg_y.data


def main(args=None):
    rclpy.init(args=args)
    node = NumberGenerator()
    try:
        # sleep statement to spin the node after 0.5 seconds
        time.sleep(0.5)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
