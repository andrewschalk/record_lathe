import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import Button

PIN = 18

class LimitSwitch(Node):
    def __init__(self):
        super().__init__('limit_switch')
        self.publisher_ = self.create_publisher(Bool, 'limit_switch', 10)

        self.button = Button(2)
        self.button.when_pressed = self.limit_switch_callback


    def limit_switch_callback(self, channel):
        """Called when the limit switch is triggered. Publishes "True to the limit_switch topic"""
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info(f"Limit Switch triggered")


def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitch()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
