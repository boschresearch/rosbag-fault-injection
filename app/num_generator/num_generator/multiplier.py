"""" This node is used to multiply the number by a factor of 2 """
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
