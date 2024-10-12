import threading
import time
import os
import sys

# Install Tornado for WebSocket handling
try:
    import tornado.httpserver
    import tornado.websocket
    import tornado.ioloop
    import tornado.web
except ImportError:
    print('Install tornado: pip install tornado')
    sys.exit(0)

from tmuxsend import TmuxSend  # Assuming tmuxsend.py is adapted for ROS 2

# Global variables
websocket_server = None     # WebSocket handler
run = True                  # Main loop run flag
server_name = 'Bringup'     # Server name
server_port = 9912          # Web server port
status = "Idle"             # Robot status sent to WebSocket

usenetcat = True  # Assume you use netcat for some operations

# WebSocket server handler
class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def initialize(self):
        # Initialize tmux
        self.tmux = TmuxSend('bringup', ['robot', 'cmd', 'laser', 'camera'])  

    def set_custom_status(self, st):
        global status
        status = st
        # Schedule the message to be sent on the main thread using add_callback
        tornado.ioloop.IOLoop.current().add_callback(lambda: self.write_message(f'STATUS {status}'))

    def open(self):
        global websocket_server, run
        websocket_server = self
        print('>>> New connection <<<')
        self.set_custom_status('Executing...')
        self.initialize()  # Initialize tmux when a new connection is opened

    def on_message(self, message):
        print(f'>>> MESSAGE RECEIVED: {message} <<<')
        self.set_custom_status(message)
        try:
            self.process_message(message)
        except Exception as e:
            print(f"Error in message: {message} - {e}")
        self.set_custom_status('Idle')

    def process_message(self, message):
        if message == 'robot_start':
            self.tmux.cmd(self.tmux.winlist.index('robot'), "echo '@robot_start' | netcat -w 1 localhost 9236")
        elif message == 'robot_kill':
            self.tmux.cmd(self.tmux.winlist.index('robot'), "echo '@robot_kill' | netcat -w 1 localhost 9236")

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
            # Schedule sending the status update using add_callback from the IOLoop in the main thread
            tornado.ioloop.IOLoop.current().add_callback(lambda: websocket_server.write_message(f"STATUS {status}"))
    print("Main loop quit.")


# Main program
if __name__ == "__main__":
    # Run the web server in the main thread
    application = tornado.web.Application([(r'/websocketserver', MyWebSocketServer),])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print(f"{server_name} WebSocket server listening on port {server_port}")
    sys.stdout.flush()

    # Start the main loop in a separate thread
    main_thread = threading.Thread(target=main_loop)
    main_thread.start()

    try:
        tornado.ioloop.IOLoop.current().start()  # Start Tornado I/O loop in the main thread
    except KeyboardInterrupt:
        print("-- Keyboard interrupt --")

    if websocket_server:
        websocket_server.close()

    print(f"{server_name} WebSocket server quit.")
    run = False
    print("Waiting for main loop to quit...")
