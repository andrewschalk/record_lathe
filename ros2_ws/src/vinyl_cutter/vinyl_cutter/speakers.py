import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import pygame
from std_msgs.msg import Bool

class Speakers(Node):
    def __init__(self):
        super().__init__('speakers')
    
        self.subscription_play_audio = self.create_subscription(
            Bool,
            'play_audio',
            self.play_audio_callback,
            10)
        self.publisher_audio_done = self.create_publisher(Bool, 'audio_done', 10)
    
    def play_audio_callback(self, msg):
        """Plays the audio file stored at ./audio/audio.wav when the play_audio topic is published to"""
        self.get_logger().info('Playing audio')
        
        # Use Pygame to play the audio file
        # Uses the default audio output on the host. Set to the 3.5mm audio jack
        pygame.mixer.init()
        pygame.mixer.music.load("audio/audio.wav")
        pygame.mixer.music.play()

        # Blocks the code while the audio file plays
        while pygame.mixer.music.get_busy():
            pass

        # Once the audio has finished playing, publish to the audio_done topic
        msg = Bool()
        msg.data = True
        self.publisher_audio_done.publish(msg)

def main(args=None):
        try:
            with rclpy.init(args=args):
                speakers = Speakers()
                rclpy.spin(speakers)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass