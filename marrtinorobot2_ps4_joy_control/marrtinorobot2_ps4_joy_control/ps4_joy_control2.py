import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS4JoyControl(Node):
    def __init__(self):
        super().__init__('ps4_joy_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.linear_axis = 1  # Asse verticale per avanti/indietro
        self.angular_axis = 0  # Asse orizzontale per destra/sinistra
        self.enable_button = 5  # L1 per abilitare i movimenti

    def joy_callback(self, joy_msg):
        twist = Twist()

        if joy_msg.buttons[self.enable_button]:  # Verifica se L1 è premuto
            twist.linear.x = joy_msg.axes[self.linear_axis]  # Avanti/indietro
            twist.angular.z = joy_msg.axes[self.angular_axis]  # Rotazione

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PS4JoyControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
