from flask import Flask, render_template, request, redirect, url_for
import subprocess

app = Flask(__name__)

@app.route('/')
def index():
    # Get a list of available WiFi networks
    wifi_list = get_wifi_list()
    return render_template('config-wifi.html', wifi_list=wifi_list)

@app.route('/configure_wifi', methods=['POST'])
def configure_wifi():
    # Get the selected WiFi network and password from the form
    selected_wifi = request.form['selected_wifi']
    password = request.form['password']

    # Connect to the selected WiFi network
    connect_to_wifi(selected_wifi, password)

    return redirect(url_for('index'))

def get_wifi_list():
    # Use nmcli to get a list of available WiFi networks
    result = subprocess.run(['nmcli', '-f', 'SSID', 'device', 'wifi', 'list'], stdout=subprocess.PIPE, text=True)
    wifi_list = result.stdout.splitlines()[2:]
    return wifi_list

def connect_to_wifi(ssid, password):
    # Use nmcli to connect to the specified WiFi network
    subprocess.run(['nmcli', 'device', 'wifi', 'connect', ssid, 'password', password])

if __name__ == '__main__':
    #app.run(debug=True, port=5001)  # Cambia 5001 con la porta desiderata

    app.run(debug=True,port=8080)
