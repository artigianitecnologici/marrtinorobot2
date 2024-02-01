from flask import Flask, render_template, request
import subprocess
# Bash script file
bash_script = "config-routing.bash"

app = Flask(__name__)

@app.route('/')
def home():
    return render_template('config-routing.html')

@app.route('/config', methods=['POST'])
def update_config():
    if request.method == 'POST':
        # Leggi i dati del form
        wlan0_dev = request.form['wlan0_dev']
        wlan1_dev = request.form['wlan1_dev']
        bridge = request.form['bridge']
        bridge_ip = request.form['bridge_ip']
        gateway = request.form['gateway']

        # Esegui qui le operazioni di configurazione
        # ...
        # Construct the command to run the Bash script with parameters
        
        command = ["bash", bash_script, wlan0_dev, wlan1_dev, bridge , bridge_ip ,gateway ]

        # Launch the Bash script from Python
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        # Wait for the process to complete and get the output
        output, error = process.communicate()

        # Check the exit code
        exit_code = process.returncode

        # Display the output and error messages
        print("Output:", output)
        print("Error:", error)
        print("Exit Code:", exit_code)


        # Restituisci una risposta o un redirect alla homepage
        return render_template('index.html', message='Configurazione aggiornata')

if __name__ == '__main__':
    app.run(debug=True)
