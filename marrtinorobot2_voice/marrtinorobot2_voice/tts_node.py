import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os


class TTSNode(Node):

    def __init__(self):
        super().__init__('tts_node')
        self.publisher_ = self.create_publisher(String, 'tts_output', 10)
        self.subscription = self.create_subscription(
            String,
            'tts_input',
            self.tts_callback,
            10)
        self.subscription

    def tts_callback(self, msg):
        text = msg.data
        self.get_logger().info('Received text: "%s"' % text)
        #tts = gTTS(text, lang='it')

        # Convert text to speech
        tts = gTTS(text,lang='it')
        tts.save('output.mp3')
        os.system('mpg321 output.mp3')

        # Publish the fact that the TTS is done
        self.publisher_.publish(String(data='TTS done'))


def main(args=None):
    rclpy.init(args=args)

    tts_node = TTSNode()

    rclpy.spin(tts_node)

    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
