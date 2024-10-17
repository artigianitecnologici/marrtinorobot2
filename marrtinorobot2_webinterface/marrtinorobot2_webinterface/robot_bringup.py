#!/usr/bin/env python3

from __future__ import print_function
import threading
import socket
import argparse
import sys, time, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

from tmuxsend import TmuxSend  # Assuming tmuxsend is already adapted for ROS 2

# ROS Node to publish log messages
class LogPublisher(Node):
    def __init__(self):
        super().__init__('log_publisher')
        self.log_pub = self.create_publisher(String, 'log_msg', 10)
        self.nodes_list_pub = self.create_publisher(String, 'nodes_list', 10)  # Cambiato a String per la lista dei nodi

    def publish_log(self, message):
        log_msg = String()
        log_msg.data = message
        self.log_pub.publish(log_msg)
        self.get_logger().info(f'Published log: {message}')
        
    def publish_nodes_list(self):
        """Esegue il comando ros2 node list e pubblica la lista dei nodi attivi"""
        try:
            # Utilizza subprocess per eseguire il comando 'ros2 node list'
            result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE)
            nodes_list = result.stdout.decode('utf-8').splitlines()  # Ottiene la lista dei nodi
            nodes_str = ', '.join(nodes_list)  # Converte la lista in una stringa separata da virgole
            msg_to_publish = String()
            msg_to_publish.data = nodes_str
            self.nodes_list_pub.publish(msg_to_publish)  # Pubblica la lista dei nodi
            self.get_logger().info(f'Pubblicata lista dei nodi attivi: {nodes_str}')
        except Exception as e:
            self.get_logger().error(f"Errore durante l'esecuzione di ros2 node list: {str(e)}")


# Function to run the server
def run_server(port, log_publisher):

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the socket to the port
    server_address = ('', port)
    sock.bind(server_address)
    sock.listen(1)
    print(f"MARRtino Robot ROS2 server started on port {port} ...")
    
    # Log the startup message
    log_publisher.publish_log(f"MARRtino Robot ROS2 server started on port {port}")

    tmux = TmuxSend('bringup', ['cmd', 'webcam','joy','tts','face'])

    connected = False
    dorun = True
    while dorun:

        if not connected:
            print("-- Waiting for connection ...")
            log_publisher.publish_log("-- Waiting for connection ...")
            
        while (dorun and not connected):
            try:
                # Wait for a connection
                connection, client_address = sock.accept()
                connected = True
                print(f'-- Connection from {client_address[0]}')
                log_publisher.publish_log(f'-- Connection from {client_address[0]}')
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
                log_publisher.publish_log(data)  # Publish the received command
                log_publisher.publish_nodes_list()  # Pubblica la lista dei nodi attivi
      
                
                # Check if the MARRTINOROBOT_WS environment variable is set
                pfolder = os.getenv('MARRTINOROBOT2_WS')
                if pfolder is None:
                    log_publisher.publish_log("Error: MARRTINOROBOT2_WS environment variable is not set.")
                    print("Error: MARRTINOROBOT2_WS environment variable is not set.")
                    return

                pfolder += "/"  # Add trailing slash

                # Comandi per gestire il robot e altri componenti
                if data == 'robot_start':
                    tmux.cmd(0, f'cd {pfolder}')
                    tmux.cmd(0, './bringup.sh')  # Assuming bringup.sh exists
                    log_publisher.publish_log("Robot started")
                elif data == 'robot_kill':
                    tmux.Cc(0)  # Send Ctrl+C to stop the process
                    log_publisher.publish_log("Robot process killed")
                elif data == 'webcam_start':
                    tmux.cmd(1, f'cd {pfolder}')
                    tmux.cmd(1, './webcam.sh')  
                    log_publisher.publish_log("Webcam started")
                elif data == 'webcam_kill':
                    tmux.Cc(1)  
                    log_publisher.publish_log("Webcam process killed")
                elif data == 'facetracker_start':
                    tmux.cmd(4, f'cd {pfolder}')
                    tmux.cmd(4, './face_tracker.sh')  
                    log_publisher.publish_log("Face Tracker Controller started")
                elif data == 'facetracker_kill':
                    tmux.Cc(4)  
                    log_publisher.publish_log("Face Tracker Controller process killed")
                elif data == 'slam_start':
                    tmux.cmd(1, f'cd {pfolder}')
                    tmux.cmd(1, './slam.sh')  
                    log_publisher.publish_log("Slam started")
                elif data == 'slam_kill':
                    tmux.Cc(1)  
                    log_publisher.publish_log("Slam process killed")
                elif data == 'tts_start':
                    tmux.cmd(3, f'cd {pfolder}')
                    tmux.cmd(3, './tts.sh')  
                    log_publisher.publish_log("tts started")
                elif data == 'tts_stop':
                    tmux.Cc(3)  
                    log_publisher.publish_log("tts process killed")
                
                # Aggiungi il comando per la lista dei nodi
                elif data == 'nodes_count':
                    log_publisher.publish_nodes_list()  # Pubblica la lista dei nodi attivi
                
                else:
                    log_publisher.publish_log(f'Unknown command: {data}')
                    print(f'Unknown command: {data}')


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    # Create the log publisher node
    log_publisher = LogPublisher()

    default_port = 9236

    # Argument parser for command-line options
    parser = argparse.ArgumentParser(description='robot bringup')
    parser.add_argument('-server_port', type=int, default=default_port, help='server port')

    # Parse arguments
    args = parser.parse_args()

    # Start server in a separate thread to keep ROS spinning
    server_thread = threading.Thread(target=run_server, args=(args.server_port, log_publisher))
    server_thread.start()

    try:
        # Spin the ROS node
        rclpy.spin(log_publisher)
    except KeyboardInterrupt:
        print("Shutting down server.")
    finally:
        server_thread.join()  # Wait for the server to finish
        log_publisher.destroy_node()
        rclpy.shutdown()
