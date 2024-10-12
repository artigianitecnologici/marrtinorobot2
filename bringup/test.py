import subprocess

def run_robot_start():
    try:
        # Eseguire il comando utilizzando subprocess.run
        result = subprocess.run("echo '@robot_start' | netcat -w 1 localhost 9236", 
                                shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print("Command Output:", result.stdout)
    except subprocess.CalledProcessError as e:
        # In caso di errore, stampare l'errore
        print(f"Error occurred: {e.stderr}")

if __name__ == '__main__':
    run_robot_start()