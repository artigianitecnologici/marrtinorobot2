from flask import Flask, render_template, request, jsonify
import rclpy
from geometry_msgs.msg import Twist

app = Flask(__name__)
node = None

def move_robot(linear_velocity, angular_velocity):
    global node
    if node is None:
        rclpy.init()
        node = rclpy.create_node('robot_controller')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity
    publisher.publish(twist)

@app.route('/')
def index():
    return render_template('code.html')

@app.route('/generate_code', methods=['POST'])
def generate_code():
    code = request.form['code']
    # Esegui l'elaborazione del codice e invia i comandi al robot
    # Questo è solo un esempio di logica, sostituiscilo con la tua logica effettiva
    if 'move_forward' in code:
        move_robot(0.2, 0)
    elif 'turn_left' in code:
        move_robot(0, 0.5)
    elif 'turn_right' in code:
        move_robot(0, -0.5)
    return jsonify({'success': True})

if __name__ == '__main__':
    app.run(debug=True)
