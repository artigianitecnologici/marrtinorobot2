#!/usr/bin/env python3

from __future__ import print_function

import threading
import socket
import argparse
import sys, time, os

from tmuxsend import TmuxSend  # Assuming tmuxsend is already adapted for ROS 2

# Function to run the server
def run_server(port):

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the socket to the port
    server_address = ('', port)
    sock.bind(server_address)
    sock.listen(1)
    print(f"MARRtino pantilt server started on port {port} ...")

    tmux = TmuxSend('bringup', ['webcam', 'joy4w'])

    connected = False
    dorun = True
    while dorun:

        if not connected:
            print("-- Waiting for connection ...")
        while (dorun and not connected):
            try:
                # Wait for a connection
                connection, client_address = sock.accept()
                connected = True
                print(f'-- Connection from {client_address[0]}')
            except KeyboardInterrupt:
                print("User interrupt (quit)")
                dorun = False
            except Exception as e:
                print(e)
                pass  # keep listening
    
        if not dorun:
            return

        # Wait for data from the client
        data = None
        while dorun and connected and data is None:
            try:
                # Receive data
                data = connection.recv(320)  # blocking
                data = data.strip().decode('utf-8')  # Decode byte data to string
            except KeyboardInterrupt:
                print("User interrupt (quit)")
                dorun = False
            except socket.timeout:
                data = None
                print("socket timeout")

        if data is not None:
            if len(data) == 0:
                connected = False
            else:
                print(data)
                # Check if the MARRTINOROBOT_WS environment variable is set
                pfolder = os.getenv('MARRTINOROBOT2_WS')
                if pfolder is None:
                    print("Error: MARRTINOROBOT2_WS environment variable is not set.")
                    return

                pfolder += "/"  # Add trailing slash
                if data == '@robot_start':
                    tmux.cmd(0, f'cd {pfolder}')
                    tmux.cmd(0, './bringup.sh')  # Assuming bringup.sh exists
                    # Automatically attach to the tmux session
                    os.system('tmux attach-session -t bringup')
                elif data == '@robot_kill':
                    tmux.Cc(0)  # Send Ctrl+C to stop the process
                if data == '@webcam_start':
                    tmux.cmd(1, f'cd {pfolder}')
                    tmux.cmd(1, './webcam.sh')  # Assuming bringu
                    # Automatically attach to the tmux session
                    os.system('tmux attach-session -t webcam')
                elif data == '@webcam_kill':
                    tmux.Cc(0)  # Send Ctrl+C to stop the process

                    print(f'Unknown command: {data}')


if __name__ == '__main__':

    default_port = 9236

    # Argument parser for command-line options
    parser = argparse.ArgumentParser(description='robot bringup')
    parser.add_argument('-server_port', type=int, default=default_port, help='server port')

    # Parse arguments
    args = parser.parse_args()

    # Run the server
    run_server(args.server_port)
