#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class Encoder(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # ==== CONFIG ====
        self.pin_a = 17   # GPIO pin for encoder channel A
        self.pin_b = 27   # GPIO pin for encoder channel B
        self.publish_rate = 100.0  # Hz for publishing count

        # ==== SETUP GPIO ====
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # ==== VARIABLES ====
        self.encoder_count = 0
        self.last_a = GPIO.input(self.pin_a)
        self.last_b = GPIO.input(self.pin_b)

        # ==== INTERRUPT SETUP ====
        GPIO.add_event_detect(self.pin_a, GPIO.RISING, callback=self.encoder_callback)

        # ==== ROS SETUP ====
        self.pub = self.create_publisher(Int32, 'encoder_ticks', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_count)

        self.get_logger().info("Encoder node started (RPi.GPIO interrupts active)")

    def encoder_callback(self, channel):
        """Called on Rising edge of A"""
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)

        # direction logic
        if a == b:
            self.encoder_count += 1
        else:
            self.encoder_count -= 1

        self.last_a = a
        self.last_b = b

    def publish_count(self):
        msg = Int32()
        msg.data = self.encoder_count
        self.pub.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Encoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
