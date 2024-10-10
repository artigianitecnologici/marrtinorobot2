#!/usr/bin/env python3#
import subprocess
import rclpy
from rclpy.node import Node
from threading import Thread
import time
import sys

# Inizializzazione del server WebSocket (con Tornado)
try:
    import tornado.httpserver
    import tornado.websocket
    import tornado.ioloop
    import tornado.web
except Exception as e:
    print(e)
    print('Install tornado: pip install --user tornado')
    sys.exit(0)

# Variabili globali
websocket_server = None  # WebSocket handler
run = True               # Flag per il ciclo principale
server_name = 'Bringup'  # Nome del server
server_port = 9912       # Porta del server Web
status = "Idle"          # Stato del robot inviato tramite WebSocket

# WebSocket server handler
class MyWebSocketServer(tornado.websocket.WebSocketHandler):

    def setStatus(self, st):
        global status
        status = st
        self.write_message('STATUS %s' % status)

    def open(self):
        global websocket_server
        websocket_server = self
        print('>>> New connection <<<')
        self.setStatus('Executing...')
        self.tmux = TmuxControl()
        self.tmux.tmux_init()  # Inizializza sessione tmux
        self.checkStatus()

    def checkStatus(self):
        self.setStatus('Checking...')
        # Puoi implementare qui le funzioni di verifica per ROS o tmux
        self.setStatus('Idle')

    def on_message(self, message):
        print('>>> MESSAGE RECEIVED: %s <<<' % message)
        self.setStatus(message)
        if message == 'robot_start':
            self.tmux.start_robot()
        elif message == 'robot_kill':
            self.tmux.stop_robot()
        elif message == 'shutdown':
            self.tmux.shutdown_system()

        self.setStatus('Idle')

    def on_close(self):
        print('Connection closed')

# Classe per il controllo di tmux
class TmuxControl:
    def __init__(self):
        # Lista delle finestre di tmux
        self.winlist = ['robot', 'laser', 'camera', 'navigation', 'playground']
        self.session_name = 'ros2_tmux_session'

    def tmux_init(self):
        # Crea una nuova sessione tmux e aggiungi le finestre
        subprocess.run(f"tmux new-session -d -s {self.session_name} -n {self.winlist[0]}", shell=True)
        for window in self.winlist[1:]:
            subprocess.run(f"tmux new-window -t {self.session_name} -n {window}", shell=True)
        print(f"Sessione tmux {self.session_name} creata con finestre: {self.winlist}")

    def tmux_cmd(self, window_name, cmd):
        # Esegui un comando in una finestra tmux
        subprocess.run(f"tmux send-keys -t {self.session_name}:{window_name} '{cmd}' C-m", shell=True)

    def tmux_kill_window(self, window_name):
        # Chiudi una finestra tmux
        subprocess.run(f"tmux kill-window -t {self.session_name}:{window_name}", shell=True)
        print(f"Finestra {window_name} chiusa.")

    def start_robot(self):
        self.tmux_cmd('robot', 'ros2 launch marrtinorobot2_bringup bringup.launch.py joy:=true')
        print('Lanciato il robot bringup.')

    def stop_robot(self):
        self.tmux_kill_window('robot')

    def shutdown_system(self):
        subprocess.run("sudo shutdown -h now", shell=True)
        print('Sistema in spegnimento.')

# Main loop del WebSocket e gestione tmux
def main_loop(data):
    global run, websocket_server, status
    while run:
        time.sleep(2)
        if run and websocket_server is not None:
            try:
                websocket_server.write_message("STATUS " + status)
            except tornado.websocket.WebSocketClosedError:
                websocket_server = None
    print("Main loop quit.")

# Main program
def main(args=None):
    # Avvia il thread del ciclo principale
    main_thread = Thread(target=main_loop, args=(None,))
    main_thread.start()

    # Avvia il WebSocket Server
    application = tornado.web.Application([(r'/websocketserver', MyWebSocketServer),])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(server_port)
    print(f"{server_name} Websocket server listening on port {server_port}")
    sys.stdout.flush()

    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:
        print("-- Keyboard interrupt --")

    if websocket_server is not None:
        websocket_server.close()
    print(f"{server_name} Websocket server quit.")
    global run
    run = False
    print("Waiting for main loop to quit...")

if __name__ == "__main__":
    main()
