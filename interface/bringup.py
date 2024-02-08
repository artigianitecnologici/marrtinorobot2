from flask import Flask, render_template, request
import json
import subprocess

app = Flask(__name__)

JSON_FILE = 'bringup_state.json'

def save_options_state(joy, webcam, audio):
    options_state = {'joy': joy, 'webcam': webcam, 'audio': audio}
    with open(JSON_FILE, 'w') as file:
        json.dump(options_state, file)

def read_options_state():
    try:
        with open(JSON_FILE, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        return {'joy': False, 'webcam': False, 'audio': False}

# Load the options state from the file on app startup
initial_options_state = read_options_state()
save_options_state(initial_options_state['joy'], initial_options_state['webcam'], initial_options_state['audio'])


# Definisci una lista di voci del menu con i loro percorsi
menu_items = [
    {"name": "Home", "url": "/"},
    {"name": "Bringup", "url": "/bringup"},
    {"name": "Slam", "url": "/slam"},
    {"name": "Navigation", "url": "/navigation"},
    {"name": "Programming", "url": "/programming"}
]

@app.route('/')
def index():
    return render_template('index.html', menu_items=menu_items)


@app.route('/bringup')
def bringup():
    options_state = read_options_state()
    return render_template('bringup.html', options_state=options_state)

@app.route('/execute_bringup', methods=['POST'])
def execute_bringup():
    joy = request.form.get('joy') == 'on'
    webcam = request.form.get('webcam') == 'on'
    audio = request.form.get('audio') == 'on'

    save_options_state(joy, webcam, audio)

    commands = ['ros2 launch package_name bringup.launch']
    if joy:
        commands.append('ros2 launch joy_launch joy_launch.py')
    if webcam:
        commands.append('ros2 launch webcam_package webcam_launch.py')
    if audio:
        commands.append('ros2 launch audio_package audio_launch.py')

    try:
        result = subprocess.check_output(' && '.join(commands), shell=True, stderr=subprocess.STDOUT, text=True)
        return render_template('result.html', result=result)
    except subprocess.CalledProcessError as e:
        return render_template('result.html', result=f"Error executing commands: {e.output}")
    
@app.route('/blockly')
def blockly():
    return render_template('blockly.html')

@app.route('/slam')
def slam():
    options_state = read_options_state()
    return render_template('slam.html', options_state=options_state)

@app.route('/execute_slam')
def execute_slam():
    rviz = request.form.get('rviz') == 'on'
    sim = request.form.get('sim') == 'on'
    commands = ['ros2 launch marrtinorobot2_navigation slam.launch.py ']
    if rviz:
        commands = ['ros2 launch marrtinorobot2_navigation slam.launch.py rviz:=true']
    if sim:
        commands = ['ros2 launch marrtinorobot2_navigation slam.launch.py sim:=true']
    if ( rviz and sim):
        commands = ['ros2 launch marrtinorobot2_navigation slam.launch.py rviz:=true sim:=true']
    try:
        result = subprocess.check_output(' && '.join(commands), shell=True, stderr=subprocess.STDOUT, text=True)
        return render_template('result.html', result=result)
    except subprocess.CalledProcessError as e:
        return render_template('result.html', result=f"Error executing commands: {e.output}")
     

    return render_template('blockly.html')

def execute_shutdown():
    try:
        # Execute the shutdown command
        subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=True)
        return render_template('shutdown.html')  # You can create a new template for shutdown confirmation if needed
    except subprocess.CalledProcessError as e:
        return render_template('error.html', error_message=f"Error executing shutdown command: {e}")


if __name__ == '__main__':
    app.run(debug=True,port=8080)
