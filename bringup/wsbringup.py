import threading
import socket
import time
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, Image
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile

# Install Tornado for websocket handling
try:
    import tornado.httpserver
    import tornado.websocket
    import tornado.ioloop
    import tornado.web
except ImportError:
    print('Install tornado: pip install tornado')
    sys.exit(0)

from tmuxsend import TmuxSend  # Assuming tmuxsend.py is adapted for ROS 2
import check
from check import *

# Global variables
websocket_server = None     # WebSocket handler
run = True                  # Main loop run flag
server_name = 'Bringup'     # Server name
server_port = 9912          # Web server port
status = "Idle"             # Robot status sent to WebSocket

usenetcat = True  # Assume you use netcat for some operations

# WebSocket server handler
class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def check_status(self, what='ALL'):
        self.set_status('Checking...')
        r = check_ros()
        self.write_message(f'RESULT ros {r}')
        if r:
            rclpy.init(args=None)
            node = rclpy.create_node('marrtino_bringup')
            self.write_message(f'VALUE rosnodes {check.nodenames}')
            self.write_message(f'VALUE rostopics {check.topicnames}')
            rclpy.shutdown()

        if what == 'robot' or what == 'ALL':
            r = check_robot()
            self.write_message(f'RESULT robot {r}')
            r = check_turtle()
            self.write_message(f'RESULT turtle {r}')
            r = check_simrobot()
            self.write_message(f'RESULT simrobot {r}')
            r = check_odom()
            self.write_message(f'RESULT odom {r}')

        if what == 'sonar' or what == 'ALL':
            r = check_sonar()
            self.write_message(f'RESULT sonar {r}')

        if what == 'laser' or what == 'cameralaser' or what == 'ALL':
            r = check_laser()
            self.write_message(f'RESULT laser {r}')
            r = check_kinect()
            self.write_message(f'RESULT kinect {r}')

        if what == 'camera' or what == 'cameralaser' or what == 'ALL':
            r = check_rgb_camera()
            self.write_message(f'RESULT rgb {r}')
            r = check_depth_camera()
            self.write_message(f'RESULT depth {r}')

        # Check tf transforms
        print("Checking tf ...")
        r = check_tf('map', 'odom')
        self.write_message(f'RESULT tf_map_odom {r}')
        r = check_tf('odom', 'base_frame')
        self.write_message(f'RESULT tf_odom_base {r}')
        r = check_tf('base_frame', 'laser_frame')
        self.write_message(f'RESULT tf_base_laser {r}')
        r = check_tf('base_frame', 'rgb_camera_frame')
        self.write_message(f'RESULT tf_base_rgb {r}')
        r = check_tf('base_frame', 'depth_camera_frame')
        self.write_message(f'RESULT tf_base_depth {r}')

        # Check nodes
        rr = check_nodes()
        for [m, t] in rr:
            self.write_message(f'RESULT {m} {t}')

        self.set_status('Idle')
        time.sleep(1)
        self.set_status('Idle')

    def set_status(self, st):
        global status
        status = st
        self.write_message(f'STATUS {status}')

    def open(self):
        global websocket_server, run
        websocket_server = self
        print('>>> New connection <<<')
        self.set_status('Executing...')
        self.winlist = ['cmd', 'roscore', 'quit', 'wsrobot', 'modim', 'robot', 'laser',
                        'camera', 'imgproc', 'joystick', 'audio', 'map_loc', 'navigation',
                        'playground', 'netcat', 'social']
        self.tmux = TmuxSend('bringup', self.winlist)
        self.tmux.roscore(self.winlist.index('roscore'))
        time.sleep(1)
        self.tmux.cmd(self.winlist.index('modim'), 'cd $MODIM_HOME/src/GUI')
        self.tmux.cmd(self.winlist.index('modim'), 'python ws_server.py -robot marrtino')
        time.sleep(1)
        self.wsrobot()

        self.check_status()

    def on_message(self, message):
        print(f'>>> MESSAGE RECEIVED: {message} <<<')
        self.set_status(message)
        try:
            self.process_message(message)
        except Exception as e:
            print(f"Error in message: {message} - {e}")
        self.set_status('Idle')

    def process_message(self, message):
        if message == 'check':
            self.check_status()

        elif message == 'ros_quit':
            self.tmux.quitall(range(5, len(self.winlist)))
            self.check_status()

        elif message == 'robot_start':
            if usenetcat:
                self.tmux.cmd(self.winlist.index('netcat'), "echo '@robot' | netcat -w 1 localhost 9236")
            else:
                self.tmux.roslaunch(self.winlist.index('robot'), 'robot', 'robot')
            self.waitfor('robot', 5)
            self.waitfor('odom', 1)
            self.waitfor('sonar', 1)
        
        # Handle other commands like turtle_start, simrobot_start, sonar, laser, etc.

    def waitfor(self, what, timeout):
        time.sleep(2)
        r = check_it(what)
        while not r and timeout > 0:
            time.sleep(1)
            timeout -= 1
            r = check_it(what)
        self.write_message(f'RESULT {what} {r}')

    def wsrobot(self):
        pass

    def on_close(self):
        print('Connection closed')

    def check_origin(self, origin):
        return True  # Allow WebSocket connections from any origin


# Main loop (asynchronous thread)
def main_loop():
    global run, websocket_server, status
    while run:
        time.sleep(2)
        if websocket_server:
            try:
                websocket_server.write_message(f"STATUS {status}")
            except tornado.websocket.WebSocketClosedError:
                websocket_server = None
    print("Main loop quit.")


# Main program
if __name__ == "__main__":
    # Start main thread
    main_thread = threading.Thread(target=main_loop)
    main_thread.start()

    # Run the web server
    application = tornado.web.Application([(r'/websocketserver', MyWebSocketServer),])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print(f"{server_name} WebSocket server listening on port {server_port}")
    sys.stdout.flush()

    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print("-- Keyboard interrupt --")

    if websocket_server:
        websocket_server.close()

    print(f"{server_name} WebSocket server quit.")
    run = False
    print("Waiting for main loop to quit...")
