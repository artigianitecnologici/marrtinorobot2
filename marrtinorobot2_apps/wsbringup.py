from __future__ import print_function

import socket
import time
import os

from threading import Thread
import sys
import subprocess

try:
    import tornado.httpserver
    import tornado.websocket
    import tornado.ioloop
    import tornado.web
except Exception as e:
    print(e)
    print('Install tornado: pip install --user tornado')
    sys.exit(0)

import check  # Assuming check.py contains ROS 2 related functionality
from check import *
from tmuxsend import TmuxSend

# Global variables
websocket_server = None     # WebSocket handler
run = True                  # Main loop run flag
server_name = 'Bringup'     # Server name
server_port = 9912          # WebSocket server port
status = "Idle"             # Robot status sent to WebSocket
usenetcat = True            # Flag for using netcat

# WebSocket server handler class
class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def checkStatus(self, what='ALL'):
        """
        Function to check the status of ROS 2 system, including nodes and topics.
        """
        self.setStatus('Checking...')
        
        # Checking ROS 2 system status (assumes check_ROS is compatible with ROS 2)
        r = check_ROS()
        self.write_message('RESULT ros2 ' + str(r))
        if r:
            # ROS 2 initialization (rclpy)
            rclpy.init()  # Initialize ROS 2 (instead of rospy)
            self.write_message('VALUE ros2 nodes %r' % check.nodenames)
            self.write_message('VALUE ros2 topics %r' % check.topicnames)

        # Example checks for ROS 2
        if what == 'robot' or what == 'ALL':
            r = check_robot()
            self.write_message('RESULT robot ' + str(r))
            r = check_odom()
            self.write_message('RESULT odom ' + str(r))

        if what in ['laser', 'cameralaser', 'ALL']:
            r = check_laser()
            self.write_message('RESULT laser ' + str(r))
            r = check_kinect()
            self.write_message('RESULT kinect ' + str(r))
        if what in ['camera', 'cameralaser', 'ALL']:
            r = check_rgb_camera()
            self.write_message('RESULT rgb ' + str(r))
            r = check_depth_camera()
            self.write_message('RESULT depth ' + str(r))

        self.setStatus('Idle')
        time.sleep(1)

    def setStatus(self, st):
        """
        Update and send the robot's status to the WebSocket clients.
        """
        global status
        status = st
        self.write_message('STATUS %s' % status)

    def open(self):
        """
        Called when a new WebSocket connection is established.
        """
        global websocket_server, run
        websocket_server = self
        print('>>> New connection <<<')
        self.setStatus('Executing...')

        # List of tmux windows for managing various ROS 2 components
        self.winlist = ['cmd', 'cmd2', 'quit', 'wsrobot', 'modim',
                        'robot', 'laser', 'camera', 'imgproc', 'joystick', 'audio',
                        'map_loc', 'navigation', 'playground', 'netcat', 'social']

        # Indexes for the tmux windows
        self.wroscore = self.winlist.index('roscore')
        self.wrobot = self.winlist.index('robot')
        self.wlaser = self.winlist.index('laser')
        self.wcamera = self.winlist.index('camera')
        self.wimgproc = self.winlist.index('imgproc')
        self.wjoystick = self.winlist.index('joystick')
        self.waudio = self.winlist.index('audio')
        self.wwsrobot = self.winlist.index('wsrobot')
        self.wquit = self.winlist.index('quit')
        self.wmodim = self.winlist.index('modim')
        self.wmaploc = self.winlist.index('map_loc')
        self.wnav = self.winlist.index('navigation')
        self.wplayground = self.winlist.index('playground')
        self.wnet = self.winlist.index('netcat')
        self.wsocial = self.winlist.index('social')

        # Initialize tmux session
        self.tmux = TmuxSend('bringup', self.winlist)
        self.tmux.roscore(self.wroscore)  # Starts ROS core
        time.sleep(1)

        # Start modim services
        self.tmux.cmd(self.wmodim, 'cd $MODIM_HOME/src/GUI')
        self.tmux.cmd(self.wmodim, 'python ws_server.py -robot marrtino')

        time.sleep(1)
        self.wsrobot()  # Call to wsrobot function
        self.checkStatus()

    def on_message(self, message):
        """
        Handle incoming WebSocket messages.
        """
        print('>>> MESSAGE RECEIVED: %s <<<' % message)
        self.setStatus(message)

        try:
            self.process_message(message)
        except Exception as e:
            print(f"Error in message {message}: {e}")

        self.setStatus('Idle')

    def process_message(self, message):
        """
        Process different control messages and trigger corresponding ROS actions.
        """
        if message == 'stop':
            print('!!! EMERGENCY STOP !!!')
            self.checkStatus()

        elif message == 'check':
            self.checkStatus()

        elif message == 'ros_quit':
            self.tmux.quitall(range(5, len(self.winlist)))
            self.checkStatus()

        elif message == 'robot_start':
            if usenetcat:
                self.tmux.cmd(self.wnet, "echo '@robot' | netcat -w 1 localhost 9236")
            else:
                self.tmux.roslaunch(self.wrobot, 'robot', 'robot')
            self.waitfor('robot', 5)
            self.waitfor('odom', 1)
            self.waitfor('sonar', 1)

        elif message == 'robot_kill':
            if usenetcat:
                self.tmux.cmd(self.wnet, "echo '@robotkill' | netcat -w 1 localhost 9236")
            else:
                self.tmux.roskill('orazio')
                self.tmux.roskill('state_pub_robot')
                time.sleep(1)
                self.tmux.killall(self.wrobot)
            time.sleep(1)
            self.write_message('RESULT robot False')

        # Social robot control
        elif message == 'social_robot_start':
            self.tmux.cmd(self.wnet, "echo '@robotsocial' | netcat -w 1 localhost 9250")
        elif message == 'social_robot_kill':
            self.tmux.cmd(self.wnet, "echo '@robotsocialkill' | netcat -w 1 localhost 9250")

    def on_close(self):
        """
        Handle WebSocket connection closure.
        """
        print('Connection closed')

    def check_origin(self, origin):
        """
        Allow cross-origin WebSocket requests.
        """
        return True

# Main loop
def main_loop(data):
    global run, websocket_server, status
    while run:
        time.sleep(2)
        if websocket_server:
            try:
                websocket_server.write_message("STATUS " + status)
            except tornado.websocket.WebSocketClosedError:
                websocket_server = None
    print("Main loop quit.")

# Main program
if __name__ == "__main__":
    # Start main loop thread
    t = Thread(target=main_loop, args=(None,))
    t.start()

    # Start WebSocket server
    application = tornado.web.Application([(r'/websocketserver', MyWebSocketServer)])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print(f"{server_name} WebSocket server listening on port {server_port}")

    try:
        tornado
