# marrtinorobot2_ps4_joy_control/ps4_joy_control.py
import evdev
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PS4Controller(Node):
    def __init__(self):
        super().__init__('ps4_joy_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.device = evdev.InputDevice('/dev/input/js0')  # Assicurati che il controller sia su js0
        self.get_logger().info(f"Controller {self.device.name} connected")

    def read_events(self):
        twist = Twist()
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                # Asse 0 - Movimento angolare (sinistra/destra)
                if event.code == evdev.ecodes.ABS_X:
                    twist.angular.z = event.value / 32767.0 * 2.0  # Normalizza valore su [-2.0, 2.0]
                # Asse 1 - Movimento lineare (avanti/indietro)
                if event.code == evdev.ecodes.ABS_Y:
                    twist.linear.x = -(event.value / 32767.0) * 1.0  # Normalizza valore su [-1.0, 1.0]
                self.publisher_.publish(twist)
                self.get_logger().info(f'Linear: {twist.linear.x}, Angular: {twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    ps4_controller = PS4Controller()
    ps4_controller.read_events()

    rclpy.spin(ps4_controller)
    ps4_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
