import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import time

class VinylCutter(Node):
    def __init__(self):
        super().__init__('vinyl_cutter')

        self.subscription_inner_dia = self.create_subscription(
            Int32,
            'inner_dia',
            self.inner_dia_callback,
            10)
        self.subscription_outer_dia = self.create_subscription(
            Int32,
            'outer_dia',
            self.outer_dia_callback,
            10)
        self.inner_dia = None
        self.outer_dia = None

        self.publisher_play_audio = self.create_publisher(Bool, 'play_audio', 10)

        self.subscription_audio_done = self.create_subscription(
            Bool,
            'audio_done',
            self.audio_done_callback,
            10)


        self.publisher_r_setpoint = self.create_publisher(Float32, 'r_setpoint', 10)
        self.publisher_theta_setpoint = self.create_publisher(Float32, 'theta_setpoint', 10)
        self.publisher_r_pos_set = self.create_publisher(Float32, 'r_pos_set', 10)

        self.subscription_cycle_start = self.create_subscription(
            Bool,
            'cycle_start',
            self.cycle_start_callback,
            10)

    def inner_dia_callback(self, msg):
        self.inner_dia = msg.data
        self.get_logger().info(f'Received inner diameter: {self.inner_dia}')

    def outer_dia_callback(self, msg):
        self.outer_dia = msg.data
        self.get_logger().info(f'Received outer diameter: {self.outer_dia}')

    def cycle_start_callback(self,msg):
        # Home r axis
        msg = Float32()
        self.publisher_r_setpoint()
        
        # Spin up disk
        self.publisher_theta_setpoint.publish(33.333) # Spin the motor at 33.333 rpm

        # Calculate start position
        start_position = 1 #TODO determine equation of start position

        # Move to start position
        msg = Float32()
        msg.data = start_position
        self.publisher_r_pos_set.publish(msg)

        # Start moving r-axis at cutting speed
        msg.data = 1 # TODO determine cutting speed
        self.publisher_theta_setpoint.publish(msg)

        # Play audio. Does not block but triggers a callback when the audio is finished playing
        msg = Bool()
        msg.data = True
        self.publisher_play_audio.publish(msg)

    # This is called after the audio file has finished playing, and finishes off the cut
    def audio_done_callback(self, future_audio):
        # Stop R-axis
        msg = Float32()
        msg.data = 0
        self.publisher_theta_setpoint.publish(msg)

        # Let disk spin for a moment to trap the needle at the end of playing
        time.sleep(0.555) # Spin for one rotation

        # Turn off theta axis
        self.get_logger().info('Cut Finished')

def main(args=None):
        try:
            with rclpy.init(args=args):
                vinyl_cutter = VinylCutter()

                rclpy.spin(vinyl_cutter)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass