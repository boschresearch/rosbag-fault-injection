"""" This node is used to publish the numbers on two different topics """
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
