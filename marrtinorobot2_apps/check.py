from __future__ import print_function

import sys, os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer, TransformException
from rclpy.duration import Duration

HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

nodenames = []
topicnames = []
tf_buffer = None
tf_listener = None

odomcount = 0
odomframe = ''
robot_pose = [0, 0, 0]

sonarcount = 0
sonarframe = ''
sonarvalues = [0, 0, 0, 0, 0, 0, 0, 0]
idsonar = 0

lasercount = 0
laserframe = ''

cameracount = 0
cameraframe = ''

def printOK():
    print('%sOK%s' %(OKGREEN,ENDC))

def printFail():
    print('%sFAIL%s' %(FAIL,ENDC))

def get_ROS_nodes(node):
    global nodenames
    try:
        node_names_and_types = node.get_node_names_and_namespaces()
        nodenames = [name[0] for name in node_names_and_types]
    except Exception as e:
        pass

def get_ROS_topics(node):
    global topicnames
    try:
        topic_names_and_types = node.get_topic_names_and_types()
        topicnames = [(name, type) for name, type in topic_names_and_types]
    except Exception as e:
        pass

def check_ROS(node):
    global nodenames, topicnames
    print('----------------------------------------')
    print('Check ROS...')
    nodenames = []
    topicnames = []
    try:
        get_ROS_nodes(node)
        print('  -- Nodes: %s' %nodenames)
        get_ROS_topics(node)
        print('  -- Topics: %s' %topicnames)
        printOK()
        return True
    except Exception as e:
        print(e)
        printFail()
        return False

def print_result(r):
    if r:
        printOK()
    else:
        printFail()

def odom_cb(data):
    global robot_pose, odomcount, odomframe
    robot_pose[0] = data.pose.pose.position.x
    robot_pose[1] = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    robot_pose[2] = euler[2]  # yaw
    odomcount += 1
    odomframe = data.header.frame_id

def check_odom(node):
    global topicnames, odomcount, odomframe
    print('----------------------------------------')
    print('Check odometry ...')
    get_ROS_topics(node)
    r = ('/odom', 'nav_msgs/msg/Odometry') in topicnames

    if r:
        try:
            odom_sub = node.create_subscription(Odometry, 'odom', odom_cb, 10)
            odomcount = 0
            trycount = 0
            tott = 0
            dt = 1.0
            time.sleep(dt)
            tott += dt
            dt = 0.2
            while odomcount == 0 and trycount < 5:
                time.sleep(dt)
                tott += dt
                trycount += dt
            odom_sub.destroy()
            odomrate = round(odomcount / tott, 2)
            print('  -- Odometry rate = %.2f Hz' % odomrate)
            print('  -- Odometry frame = %s' % odomframe)
        except Exception as e:
            print(e)
            r = False
    print_result(r)
    return r

def sonar_cb(data):
    global sonarcount, sonarframe, sonarvalues, idsonar
    sonarframe = data.header.frame_id
    r = data.range
    sonarvalues[idsonar] = r
    sonarcount += 1

def check_sonar(node):
    global topicnames, sonarcount, sonarframe, sonarvalues, idsonar
    print('----------------------------------------')
    print('Check sonar ...')
    for i in range(0, 4):
        sname = 'sonar_%d' % i
        idsonar = i
        if ('/'+sname, 'sensor_msgs/msg/Range') in topicnames:
            v = readSonarValue(node, i)
            if sonarcount > 0:
                print('  -- Sonar %d frame = %s' % (i, sonarframe))
                print('  -- Sonar %d range = %.2f' % (i, sonarvalues[i]))
                return True
    print_result(False)
    return False

def readSonarValue(node, i):
    global sonarvalues, idsonar, sonarcount
    idsonar = i
    sname = 'sonar_%d' % i
    try:
        sonar_sub = node.create_subscription(Range, sname, sonar_cb, 10)
        sonarcount = 0
        trycount = 0
        dt = 0.2
        tott = 0
        while sonarcount == 0 and trycount < 5:
            time.sleep(dt)
            tott += dt
            trycount += dt
        sonar_sub.destroy()
    except Exception as e:
        print(e)
    return sonarvalues[i] if sonarcount > 0 else -1

def laser_cb(data):
    global lasercount, laserframe
    lasercount += 1
    laserframe = data.header.frame_id

def check_laser(node):
    global topicnames, lasercount, laserframe
    print('----------------------------------------')
    print('Check laser scan ...')
    laserrate = 0
    get_ROS_topics(node)
    r = ('/scan', 'sensor_msgs/msg/LaserScan') in topicnames
    if r:
        laser_sub = node.create_subscription(LaserScan, 'scan', laser_cb, 10)
        lasercount = 0
        trycount = 0
        tott = 0
        dt = 1.0
        time.sleep(dt)
        tott += dt
        dt = 0.2
        while lasercount == 0 and trycount < 5:
            time.sleep(dt)
            tott += dt
            trycount += dt
        laser_sub.destroy()
        if lasercount > 0:
            laserrate = round(lasercount / tott, 2)
            print('  -- Laser scan rate = %.2f Hz' % laserrate)
            print('  -- Laser frame = %s' % laserframe)
        else:
            r = False
    print_result(r)
    return r

def image_cb(data):
    global cameracount, cameraframe
    cameracount += 1
    cameraframe = data.header.frame_id

def findImageTopic():
    global topicnames
    if ('/kinect/rgb/image_raw', 'sensor_msgs/msg/Image') in topicnames:
        return '/kinect/rgb/image_raw'
    elif ('/rgb/image_raw', 'sensor_msgs/msg/Image') in topicnames:
        return '/rgb/image_raw'
    elif ('/usbcam/image_raw', 'sensor_msgs/msg/Image') in topicnames:
        return '/usbcam/image_raw'
    else:
        return None

def check_rgb_camera(node):
    global topicnames, cameracount, cameraframe
    print('----------------------------------------')
    print('Check RGB camera ...')
    get_ROS_topics(node)
    topicim = findImageTopic()
    r = topicim is not None
    if r:
        cameracount = 0
        camera_sub = node.create_subscription(Image, topicim, image_cb, 10)
        dt = 5.0
        time.sleep(dt)
        camera_sub.destroy()
        camerarate = cameracount / dt
        print('  -- RGB camera rate = %.2f Hz' % camerarate)
        print('  -- RGB camera frame = %s' % cameraframe)
    print_result(r)
    return r

def check_tf(node, source, target):
    global tf_listener, tf_buffer
    if tf_buffer is None:
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
    print("  -- TF %s -> %s  " % (source, target), end="")
    try:
        trans = tf_buffer.lookup_transform(source, target, rclpy.time.Time(), Duration(seconds=1.0))
        printOK()
        return True
    except TransformException as e:
        print(" %s " % e, end="")
        printFail()
        return False

def check_tfs(node):
    print('----------------------------------------')
    print('Check transforms ...')
    check_tf(node, 'map', 'odom')
    check_tf(node, 'odom', 'base_link')

def check_kinect(node):
    global nodenames
    print('----------------------------------------')
    print('Check Kinect ...')
    r = ('/kinect2_bridge', '') in nodenames
    print_result(r)
    return r

def check_nodes(node):
    print('----------------------------------------')
    print('Check nodes ...')
    for n in nodenames:
        print("  -- Node: %s" % n)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('marrtino_check')

    r = check_ROS(node)
    if r:
        check_odom(node)
        check_sonar(node)
        check_laser(node)
        check_rgb_camera(node)
        check_tfs(node)
        check_kinect(node)
        check_nodes(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
