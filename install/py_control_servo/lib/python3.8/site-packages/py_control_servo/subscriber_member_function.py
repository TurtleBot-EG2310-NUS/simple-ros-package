# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# import numpy as np

#import time 
#import RPi.GPIO as GPIO

topic = 'scan'
#buffer = 0.1 # 0.1 meter
#distance = 1.0
front_ros = 0

qos_policy = rclpy.qos.QoSProfile(reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFOR, history = rclpy.qos.HistoryPolicy.KEEP_LAST, depth = 1)

#GPIO.setmode(GPIO.BCM)
#servo_pin = 21
#GPIO.setup(servo_pin, GPIO.OUT)

#p = GPIO.PWM(servo_pin, 50)
# p.start(7.5)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            topic,
            self.listener_callback,
            qos_profile = qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        dist_head = msg.ranges[front_ros]
        self.get_logger().info('Front Distant: %.2f m' % dist_head)
        # servo_sweep(float(dist_head))

        # laser_range = np.array(msg.ranges)
        # laser_range[laser_range == 0] = np.nan
        # lr2i = np.nanargmin(laser_range)
        # self.get_logger().info('Shortest distance at %i ' % lr2i)


#def servo_sweep(_dis):
#    if (_dis >= (distance - buffer) and _dis <= (distance - buffer)):
#        self.get_logger().info('trigger')



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print('Starting')
    main()
    print('Done.')
