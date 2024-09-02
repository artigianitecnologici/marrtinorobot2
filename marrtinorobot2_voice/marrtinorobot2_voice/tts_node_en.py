#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import socket
import subprocess

class TTSNode(Node):

    def __init__(self):
        super().__init__('tts_node_en')
        self.get_logger().info('Start tts_node en')
        self.publisher_ = self.create_publisher(String, '/speech/status', 10)
        self.subscription = self.create_subscription(
            String,
            '/speech/to_speak_en',
            self.tts_callback,
            10)
        self.subscription
        msgstart = "ros2 topic pub  -1 /speech/to_speak_en std_msgs/msg/String '{data: "' "tutti i sistemi sono operativi"'"}'"
        os.system(msgstart)


    def tts_callback(self, msg):
        text = msg.data
        self.get_logger().info('Received text: "%s"' % text)
        self.finished_speaking = False
        self.loop_count_down = 0

        # Check internet connectivity
        if self.is_connected():
            # Convert text to speech
            tts = gTTS(text, lang='en')
            filename="/tmp/output.mp3"
            tts.save(filename)
            os.system('mpg321 ' + filename)
            # Publish the fact that the TTS is done
            self.publisher_.publish(String(data='TTS done'))
        else:
            # self.get_logger().error('No internet connection')
            filename = "/tmp/robot_speach.wav"
            # create wav file using pico2wav from adjusted text.
            cmd = ['pico2wave', '--wave=' + filename, '--lang=' + voice, voice]
            subprocess.call(cmd)
            # Play created wav file using sox play
            cmd = ['play', filename, '--norm', '-q']
            subprocess.call(cmd)
            # Set up to send talking finished
            self.finished_speaking = True
            self.loop_count_down = int(self.LOOP_FREQUENCY * 2)

    def speaking_finished(self):
        if self.finished_speaking:
            self.loop_count_down -= 1
            if self.loop_count_down <= 0:
                self.get_logger().error('speaking finished')
                self.finished_speaking = False
                self.publisher_.publish(String(data='TTS done'))

    def is_connected(self):
        try:
            # Try to connect to a well-known website
            socket.create_connection(("www.google.com", 80))
            return True
        except OSError:
            return False
        
def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()

    rclpy.spin(tts_node)

    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
