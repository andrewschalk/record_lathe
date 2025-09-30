import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from .DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

class ThetaMotor(Node):
    def __init__(self):
        super().__init__('theta_motor')
        
        self.subscription_theta_setpoint = self.create_subscription(
            Float32,
            'theta_setpoint',
            self.theta_setpoint_callback,
            10)
        
        # PID subscriptions. Allows for the changing of pid constants while testing
        self.subscription_theta_P = self.create_subscription(
            Float32,
            'theta_P',
            self.theta_P_callback,
            10)
        self.subscription_theta_I = self.create_subscription(
            Float32,
            'theta_I',
            self.theta_I_callback,
            10)
        self.subscription_theta_D = self.create_subscription(
            Float32,
            'theta_D',
            self.theta_D_callback,
            10)

        # PID parameters
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05
        
        # PID state
        self.theta_setpoint = 0
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.speed = 0

        # Motor controller initialization
        self.motor_controller = Board(1,0x10)
        self.motor_controller.begin()
        self.motor_controller.set_moter_pwm_frequency(20000)
        self.motor_controller.set_encoder_enable(self.motor_controller.ALL)

        # Control loop, run every 0.01 s
        self.timer = self.create_timer(0.01, self.control_loop)

    def theta_setpoint_callback(self,msg):
        self.setpoint=msg.data

    def theta_P_callback(self,msg):
        self.kp=msg.data

    def theta_I_callback(self,msg):
        self.ki=msg.data

    def theta_D_callback(self,msg):
        self.kd=msg.data

    def control_loop(self):
        """Controls the PID for the motor"""
        now = time.time()
        dt  = now - self.last_time # Calculate dt because it can be variable
        self.last_time = now

        # If the speed setpoint is zero, just turn off the motor, this prevents strange behavior
        if self.theta_setpoint == 0:
            self.last_error = 0
            self.integral   = 0
            self.motor_controller.motor_movement([self.motor_controller.M1], self.motor_controller.CW,0)
            return

        # Find and save current speed
        self.speed = self.motor_controller.get_encoder_speed(self.motor_controller.M1)

        # Compute error
        error = self.theta_setpoint - self.speed

        # PID terms
        self.integral += error * dt
        derivative    = (error - self.last_error) / dt
        output        = self.kp*error + self.ki*self.integral + self.kd*derivative
        
        # Stop commands above full
        if output>100:
            output = 100
        if output<-100:
            output = -100

        # Determine whether CW or CCW
        if output>0:
            self.motor_controller.motor_movement([self.motor_controller.M1], self.motor_controller.CW,output)
        else:
            self.motor_controller.motor_movement([self.motor_controller.M1], self.motor_controller.CCW,abs(output))
        
        self.last_error = error

def main(args=None):
        try:
            with rclpy.init(args=args):
                theta_motor = ThetaMotor()
                rclpy.spin(theta_motor)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass