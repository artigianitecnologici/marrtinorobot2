import libtmux

class TmuxController:
    def __init__(self, session_name, window_list):
        self.session_name = session_name
        self.window_list = window_list
        self.tmux_server = libtmux.Server()
        self.tmux_session = self.tmux_server.find_session(session_name)
        if not self.tmux_session:
            self.tmux_session = self.tmux_server.new_session(session_name)

    def get_window(self, window_name):
        for window in self.tmux_session.windows:
            if window.name == window_name:
                return window
        return None

    def run_command(self, window_name, command):
        window = self.get_window(window_name)
        if window:
            window.send_keys(command)

    def roscore(self):
        self.run_command('roscore', 'roscore')

    def cmd(self, window_name, command):
        self.run_command(window_name, command)

    def python(self, window_name, script_name):
        self.run_command(window_name, f"python {script_name}")

    def quitall(self):
        self.tmux_session.kill_session()

    def roslaunch(self, window_name, package, launch_file):
        self.run_command(window_name, f"roslaunch {package} {launch_file}")

    def roskill(self, process_name):
        self.run_command('default', f"pkill -f {process_name}")

    def killall(self):
        self.tmux_session.kill_session()
