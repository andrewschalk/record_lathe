import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from .DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
import numpy

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
        self.subscription_theta_ks = self.create_subscription(
            Float32,
            'theta_ks',
            self.theta_ks_callback,
            10)
        self.subscription_theta_kv = self.create_subscription(
            Float32,
            'theta_kv',
            self.theta_kv_callback,
            10)
        self.subscription_theta_ka = self.create_subscription(
            Float32,
            'theta_ka',
            self.theta_ka_callback,
            10)
        self.subscription_theta_deadband = self.create_subscription(
            Float32,
            'theta_deadband',
            self.theta_deadband_callback,
            10)
        
        #Publishers, for monitoring
        self.publisher_theta_speed = self.create_publisher(Float32, 'theta_speed', 10)
        self.publisher_theta_command = self.create_publisher(Float32, 'theta_command', 10)


        # PID parameters
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05
        self.deadband = 0
        self.last_speed = 0
        self.last_setpoint = 0

        # Feedforward parameters
        self.ks = 0
        self.kv = 0
        self.ka = 0
        
        # PID state
        self.theta_setpoint = 0
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.last_output = 0
        self.speed = 0

        # Motor controller initialization
        self.motor_controller = Board(1,0x10)
        self.motor_controller.begin()
        self.motor_controller.set_moter_pwm_frequency(12750)
        self.motor_controller.set_encoder_enable(self.motor_controller.ALL)
        self.motor_controller.set_encoder_reduction_ratio(self.motor_controller.ALL, 1)

        # Control loop, run every 1 ms
        self.timer = self.create_timer(0.001, self.control_loop)

    def theta_setpoint_callback(self,msg):
        self.get_logger().debug(f"Received setpoint: {msg.data}")
        self.theta_setpoint = msg.data

    def theta_P_callback(self,msg):
        self.kp=msg.data

    def theta_I_callback(self,msg):
        self.ki=msg.data

    def theta_D_callback(self,msg):
        self.kd=msg.data

    def theta_stiction_callback(self,msg):
        self.stiction=msg.data

    def theta_deadband_callback(self,msg):
        self.deadband=msg.data

    def theta_ks_callback(self,msg):
        self.ks=msg.data

    def theta_kv_callback(self,msg):
        self.kv=msg.data
    
    def theta_ka_callback(self,msg):
        self.ka=msg.data

    def control_loop(self):
        """Controls the PID for the motor"""
        now = time.time()
        dt  = now - self.last_time # Calculate dt because it can be variable
        self.last_time = now

        # If the speed setpoint is zero, just turn off the motor, this prevents strange behavior
        if self.theta_setpoint == 0:
            self.last_error = 0
            self.integral   = 0
            self.motor_controller.motor_movement([self.motor_controller.M2], self.motor_controller.CW,0)
            return

        # Find and save current speed
        self.speed = -self.motor_controller.get_encoder_speed([self.motor_controller.M2])[0]/43

        # Sometimes the motor controller reports the speed as ~700 erroneously so just ignore this and assume no change
        if self.speed>600:
            self.speed=self.last_speed
        
        ## If the motor controller has not updated the speed, simply return
        #if self.last_speed == self.speed and self.speed != 0:
        #    return
        self.last_speed = self.speed

        self.get_logger().debug(f"Speed: {self.speed}")
        msg = Float32()
        msg.data = float(self.speed)
        self.publisher_theta_speed.publish(msg)

        # Compute error
        error = self.theta_setpoint - self.speed
        if abs(error)<self.deadband:
            error = 0

        # PID terms
        self.integral += error * dt
        derivative    = (error - self.last_error) / dt

        # Commanded acceleration
        acceleration = (self.theta_setpoint - self.last_setpoint)/dt
        self.last_setpoint = self.theta_setpoint

        # PID
        output = self.kp*error + self.ki*self.integral + self.kd*derivative

        # Feedforward
        output += self.ks*numpy.sign(self.theta_setpoint)+self.kv*self.theta_setpoint+self.ka*acceleration

        # Stop commands above full (100% duty cycle breaks so instead use 95)
        if output>95:
            output = 95
        if output<-95:
            output = -95
        
        self.last_output = output

        self.get_logger().debug(f"Command: {output}")
        msg.data = float(output)
        self.publisher_theta_command.publish(msg)

        # Determine whether CW or CCW
        if output>0:
            self.motor_controller.motor_movement([self.motor_controller.M2], self.motor_controller.CW,output)
        else:
            self.motor_controller.motor_movement([self.motor_controller.M2], self.motor_controller.CCW,abs(output))
        
        self.last_error = error

def main(args=None):
        try:
            with rclpy.init(args=args):
                theta_motor = ThetaMotor()
                rclpy.spin(theta_motor)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass