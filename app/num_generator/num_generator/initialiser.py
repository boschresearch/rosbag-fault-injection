""" This node initliases the value of  y and publishes
it depending on the condition"""
#!/usr/bin/env python3
#
# Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not  use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

__author__ = "Min Hee Jo"
__copyright__ = "Copyright 2024, Robert Bosch GmbH"
__license__ = " Apache"
__version__ = "2.0"
__email__ = minhee.jo@de.bosch.com


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
