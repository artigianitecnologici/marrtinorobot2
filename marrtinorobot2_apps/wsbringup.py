from __future__ import print_function

import socket
import time
import os

from threading import Thread

import sys

# http://www.html.it/pag/53419/websocket-server-con-python/
# sudo -H pip install tornado

try:
    import tornado.httpserver
    import tornado.websocket
    import tornado.ioloop
    import tornado.web
except Exception as e:
    print(e)
    print('Install tornado: pip install --user tornado')
    sys.exit(0)
##
# sys.path.append('../program') 
# sys.path.append('../scripts')

import check
from check import *

from tmuxsend import TmuxSend

# Global variables

websocket_server = None     # websocket handler
run = True                  # main_loop run flag
server_name = 'Bringup'     # server name
server_port = 9912          # web server port
status = "Idle"             # robot status sent to websocket

usenetcat = True

# Websocket server handler

class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def checkStatus(self, what='ALL'):

        self.setStatus('Checking...')

        r = check_ROS()
        self.write_message('RESULT ros '+str(r))
        if (r):
            rospy.init_node('marrtino_bringup', disable_signals=True)
            self.write_message('VALUE rosnodes %r' %check.nodenames)
            self.write_message('VALUE rostopics %r' %check.topicnames)

        if (what=='robot' or what=='ALL'):
            r = check_robot()
            self.write_message('RESULT robot '+str(r))
           
            r = check_odom()
            self.write_message('RESULT odom '+str(r))

        if (what=='sonar' or what=='ALL'):
            r = check_sonar()
            self.write_message('RESULT sonar '+str(r))

        if (what=='laser' or what=='cameralaser' or what=='ALL'):
            r = check_laser()
            self.write_message('RESULT laser '+str(r))
            r = check_kinect()
            self.write_message('RESULT kinect '+str(r))

        if (what=='camera' or what=='cameralaser' or what=='ALL'):
            r = check_rgb_camera()
            self.write_message('RESULT rgb '+str(r))
            r = check_depth_camera()
            self.write_message('RESULT depth '+str(r))

        print("Checking tf ...")
        # r = check_tf('map', 'odom')
        # self.write_message('RESULT tf_map_odom '+str(r))
        # r = check_tf('odom', 'base_frame')
        # self.write_message('RESULT tf_odom_base '+str(r))
        # r = check_tf('base_frame', 'laser_frame')
        # self.write_message('RESULT tf_base_laser '+str(r))
        # r = check_tf('base_frame', 'rgb_camera_frame')
        # self.write_message('RESULT tf_base_rgb '+str(r))
        # r = check_tf('base_frame', 'depth_camera_frame')
        # self.write_message('RESULT tf_base_depth '+str(r))
        # rr = check_nodes()
        # for [m,t] in rr:
        #     self.write_message('RESULT %s %s ' %(m,t))

        self.setStatus('Idle')
        time.sleep(1)
        self.setStatus('Idle')


    def setStatus(self, st):
        global status
        status = st
        self.write_message('STATUS %s' %status)


    def open(self):
        global websocket_server, run
        websocket_server = self
        print('>>> New connection <<<')
        self.setStatus('Executing...')
        self.winlist = ['cmd','roscore','quit','wsrobot','modim',
                        'robot','laser','camera','imgproc','joystick','audio',
                        'map_loc','navigation','playground','netcat','social']

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

        self.tmux = TmuxSend('bringup',self.winlist)
        self.tmux.roscore(self.wroscore)
        time.sleep(1)
        self.tmux.cmd(self.wmodim,'cd $MODIM_HOME/src/GUI')
        self.tmux.cmd(self.wmodim,'python ws_server.py -robot marrtino')
        time.sleep(1)
        self.wsrobot()
        #time.sleep(3)

        self.checkStatus()

        print("----")
        sys.stdout.flush()

    def waitfor(self, what, timeout):
        time.sleep(2)
        r = check_it(what)
        while not r and timeout>0:
            time.sleep(1)
            timeout -= 1
            r = check_it(what)
        self.write_message('RESULT %s %s' %(what,str(r)))



    def on_message(self, message):    
        print('>>> MESSAGE RECEIVED: %s <<<' %message)
        self.setStatus(message)

        try:
            self.process_message(message)
        except:
            print("Error in message %s" %message)

        print("----")
        sys.stdout.flush()

        self.setStatus('Idle')

    def process_message(self, message):
        global code, status

        if (message=='stop'):
            print('!!! EMERGENCY STOP !!!')
            self.checkStatus()

        elif (message=='check'):
            self.checkStatus()

        elif (message=='ros_quit'):
            self.tmux.quitall(range(5,len(self.winlist)))
            self.checkStatus()

        # robot start/stop
        elif (message=='robot_start'):
            if usenetcat:
                self.tmux.cmd(self.wnet,"echo '@robot' | netcat -w 1 localhost 9236")
            else:
                self.tmux.roslaunch(self.wrobot,'robot','robot')
            self.waitfor('robot',5)
            self.waitfor('odom',1)
            self.waitfor('sonar',1)
        elif (message=='robot_kill'):
            if usenetcat:
                self.tmux.cmd(self.wnet,"echo '@robotkill' | netcat -w 1 localhost 9236")
            else:
                self.tmux.roskill('orazio')
                self.tmux.roskill('state_pub_robot')
                time.sleep(1)
                self.tmux.killall(self.wrobot)
            time.sleep(1)
            if check_robot():
                self.tmux.cmd(wquit,"kill -9 `ps ax | grep websocket_robot | awk '{print $1}'`")
                time.sleep(1)
            while check_robot():
                time.sleep(1)
            self.write_message('RESULT robot False')
            #self.checkStatus('robot')


        

        # ************************
        #    S O C I A L 
        # ************************

        # social robot
        elif (message=='social_robot_start'):
            self.tmux.cmd(self.wnet,"echo '@robotsocial' | netcat -w 1 localhost 9250")
            time.sleep(1)
        elif (message=='social_robot_kill'):
            self.tmux.cmd(self.wnet,"echo '@robotsocialkill' | netcat -w 1 localhost 9250")

        # social  (no tracker)
        elif (message=='socialnt_start'):
            self.tmux.cmd(self.wnet,"echo '@socialnotracker' | netcat -w 1 localhost 9250")
            time.sleep(1)
        elif (message=='socialnt_kill'):
            self.tmux.cmd(self.wnet,"echo '@socialnotrackerkill' | netcat -w 1 localhost 9250")

        # social no servo (dynamixel) demo
        elif (message=='socialns_start'):
            self.tmux.cmd(self.wnet,"echo '@socialnoservo' | netcat -w 1 localhost 9250")
            time.sleep(1)
        elif (message=='socialns_kill'):
            self.tmux.cmd(self.wnet,"echo '@socialnoservokill' | netcat -w 1 localhost 9250")

        # update social apps
        elif (message=='updatesocialapps'):
            self.tmux.cmd(self.wnet,"echo '@updatesocialapps' | netcat -w 1 localhost 9250")
            time.sleep(1)
        
        # social no servo (dynamixel) demo
        elif (message=='interactive_start'):
            self.tmux.cmd(self.wnet,"echo '@interactive' | netcat -w 1 localhost 9250")
            time.sleep(1)
        elif (message=='interactive_kill'):
            self.tmux.cmd(self.wnet,"echo '@interactivekill' | netcat -w 1 localhost 9250")

        # social no servo (dynamixel) demo
        elif (message=='asroffline_start'):
            self.tmux.cmd(self.wnet,"echo '@asroffline' | netcat -w 1 localhost 9252")
            time.sleep(1)
        elif (message=='asroffline_kill'):
            self.tmux.cmd(self.wnet,"echo '@asrofflinekill' | netcat -w 1 localhost 9252")
        # ***************************
        #   S O C I A L  -  E N D
        # ***************************



        # shutdown
        elif (message=='shutdown'):
            #self.tmux.quitall()
            #self.checkStatus()
            self.tmux.cmd(self.wquit,'touch ~/log/shutdownrequest')
            self.tmux.cmd(self.wquit,'sleep 10 && sudo shutdown -h now')
            self.setStatus('shutdown')

        else:
            print('Code received:\n%s' %message)
            if (status=='Idle'):
                t = Thread(target=run_code, args=(message,))
                t.start()
            else:
                print('Program running. This code is discarded.')



    def on_close(self):
        print('Connection closed')

    def on_ping(self, data):
        print('ping received: %s' %(data))

    def on_pong(self, data):
        print('pong received: %s' %(data))

    def check_origin(self, origin):
        #print("-- Request from %s" %(origin))
        return True


    def wsrobot(self):
        #self.tmux.python(self.wwsrobot,'blockly','websocket_robot.py')
        #time.sleep(3)
        pass


# Main loop (asynchrounous thread)

def main_loop(data):
    global run, websocket_server, status
    while (run):
        time.sleep(2)
        if (run and not websocket_server is None):
            try:
                websocket_server.write_message("STATUS "+status)
                #print(status)
            except tornado.websocket.WebSocketClosedError:
                # print('-- WebSocketClosedError --')
                websocket_server = None
    print("Main loop quit.")


def run_code(code):
    global status
    if (code is None):
        return





# Main program

if __name__ == "__main__":

    # Run main thread
    t = Thread(target=main_loop, args=(None,))
    t.start()

    # Run web server
    application = tornado.web.Application([
        (r'/websocketserver', MyWebSocketServer),])  
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print("%s Websocket server listening on port %d" %(server_name,server_port))
    sys.stdout.flush()
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print("-- Keyboard interrupt --")

    if (not websocket_server is None):
        websocket_server.close()
    print("%s Websocket server quit." %server_name)
    run = False    
    print("Waiting for main loop to quit...")


