import sys
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile

nodenames = []
topicnames = []

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

def printOK():
    print(f'{OKGREEN}OK{ENDC}')

def printFail():
    print(f'{FAIL}FAIL{ENDC}')

class ROSChecker(Node):

    def __init__(self):
        super().__init__('ros_checker')

        # ROS 2 Transform Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables to store the status of topics
        self.odom_count = 0
        self.odom_frame = ''
        self.robot_pose = [0, 0, 0]
        self.laser_count = 0
        self.laser_frame = ''
        self.camera_count = 0
        self.camera_frame = ''

    def check_ros(self):
        """ Check if ROS 2 nodes and topics are available. """
        print('----------------------------------------')
        print('Check ROS 2...')
        try:
            node_list = self.get_node_names()
            print(f'  -- Nodes: {node_list}')
            topic_list = self.get_topic_names_and_types()
            print(f'  -- Topics: {topic_list}')
            printOK()
        except Exception as e:
            print(e)
            printFail()

    def check_odom(self):
        """ Check if the odometry topic is publishing. """
        print('----------------------------------------')
        print('Check odometry...')

        self.odom_count = 0
        odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        rclpy.spin_once(self, timeout_sec=2)

        if self.odom_count > 0:
            print(f'  -- Odometry frame = {self.odom_frame}')
            printOK()
        else:
            printFail()

    def odom_cb(self, msg):
        """ Callback for Odometry messages. """
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.odom_frame = msg.header.frame_id
        self.odom_count += 1



    def check_laser(self):
        """ Check if the laser scan topic is available and publishing. """
        print('----------------------------------------')
        print('Check laser scan...')

        self.laser_count = 0
        laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_cb, 10)
        rclpy.spin_once(self, timeout_sec=2)

        if self.laser_count > 0:
            print(f'  -- Laser frame = {self.laser_frame}')
            printOK()
        else:
            printFail()

    def laser_cb(self, msg):
        """ Callback for laser scan data. """
        self.laser_frame = msg.header.frame_id
        self.laser_count += 1

    def check_rgb_camera(self):
        """ Check if the RGB camera topics are available and publishing. """
        print('----------------------------------------')
        print('Check RGB camera...')

        self.camera_count = 0
        camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        rclpy.spin_once(self, timeout_sec=2)

        if self.camera_count > 0:
            print(f'  -- Camera frame = {self.camera_frame}')
            printOK()
        else:
            printFail()

    def camera_cb(self, msg):
        """ Callback for camera image data. """
        self.camera_frame = msg.header.frame_id
        self.camera_count += 1



def main(args=None):
    rclpy.init(args=args)
    checker = ROSChecker()

    try:
        checker.check_ros()
        checker.check_laser()
        checker.check_rgb_camera()
     
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
