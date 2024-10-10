import subprocess
import rclpy
from rclpy.node import Node
from threading import Thread
import time

class TmuxControl(Node):
    def __init__(self):
        super().__init__('tmux_control_node')

        # Lista delle finestre di tmux
        self.winlist = ['roscore', 'robot', 'laser', 'camera', 'navigation']
        self.session_name = 'ros2_tmux_session'

        # Crea la sessione tmux all'inizio
        self.tmux_init()

        # Thread per gestire lo stato
        self.main_thread = Thread(target=self.main_loop)
        self.main_thread.start()

    def tmux_init(self):
        # Avvia una sessione tmux e crea finestre
        subprocess.run(f"tmux new-session -d -s {self.session_name} -n {self.winlist[0]}", shell=True)
        for window in self.winlist[1:]:
            subprocess.run(f"tmux new-window -t {self.session_name} -n {window}", shell=True)
        self.get_logger().info(f"Sessione tmux {self.session_name} creata con finestre: {self.winlist}")

    def tmux_cmd(self, window_name, cmd):
        # Esegui un comando in una finestra tmux specifica
        subprocess.run(f"tmux send-keys -t {self.session_name}:{window_name} '{cmd}' C-m", shell=True)

    def tmux_kill_window(self, window_name):
        # Chiudi una finestra tmux
        subprocess.run(f"tmux kill-window -t {self.session_name}:{window_name}", shell=True)
        self.get_logger().info(f"Finestra {window_name} chiusa.")

    def start_roscore(self):
        self.tmux_cmd('roscore', 'ros2 daemon start')
        self.get_logger().info('ROS 2 daemon avviato.')

    def start_robot(self):
        self.tmux_cmd('robot', 'ros2 launch my_robot_bringup robot.launch.py')
        self.get_logger().info('Lanciato il robot bringup.')

    def stop_robot(self):
        self.tmux_kill_window('robot')

    def main_loop(self):
        while rclpy.ok():
            time.sleep(2)  # Simula un ciclo di aggiornamento continuo
            self.get_logger().info('Sistema operativo ROS e tmux attivi.')

def main(args=None):
    rclpy.init(args=args)
    tmux_control = TmuxControl()

    try:
        rclpy.spin(tmux_control)
    except KeyboardInterrupt:
        tmux_control.get_logger().info('Interruzione ricevuta, chiusura...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
