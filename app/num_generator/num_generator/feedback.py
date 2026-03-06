"""" This node is used to provide feedback to the generator
if a condition is fulfilled"""
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
