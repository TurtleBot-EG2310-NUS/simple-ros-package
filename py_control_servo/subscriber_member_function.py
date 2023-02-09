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

from sensor_msgs.msg import LaserScan

import time 
import RPi.GPIO as GPIO

topic = 'scan'
front_ros = 0

buffer = 0.1 # 0.1 meter
distance = 1.0

a = distance - buffer
b = distance + buffer

# Better reading
qos_policy = rclpy.qos.QoSProfile(reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history = rclpy.qos.HistoryPolicy.KEEP_LAST, depth = 10)

def deg_to_dutycycle(_v):
    return (int(_v) / 180.0)*10.0+2.5

GPIO.setmode(GPIO.BCM)
servo_pin = 16
solenoid_pin = 26

GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(solenoid_pin, GPIO.OUT)

GPIO.output(solenoid_pin, GPIO.LOW)
p = GPIO.PWM(servo_pin, 50)
p.start(deg_to_dutycycle(0))

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            topic,
            self.listener_callback,
            qos_profile = qos_policy)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        dist_head = msg.ranges[front_ros]
        self.get_logger().info('Front Distant: %.2f m' % dist_head)
        
        if dist_head >= a and dist_head <= b:
            servo_sweep(float(dist_head))
            self.get_logger().info('Trigger')

def servo_sweep(_dis):
   GPIO.output(solenoid_pin, GPIO.HIGH)
   p.ChangeDutyCycle(deg_to_dutycycle(0))
   time.sleep(1)
   GPIO.output(solenoid_pin, GPIO.LOW)
   p.ChangeDutyCycle(deg_to_dutycycle(45))


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
    main()
