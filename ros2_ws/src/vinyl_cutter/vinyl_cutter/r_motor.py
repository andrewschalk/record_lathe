import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import time
from .DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

class RMotor(Node):
    def __init__(self):
        super().__init__('r_motor')
        
        # Subscriptions
        self.subscription_r_setpoint = self.create_subscription(
            Float32,
            'r_setpoint',
            self.r_setpoint_callback,
            10)
        self.subscription_r_pos_set = self.create_subscription(
            Float32,
            'r_pos_set',
            self.r_pos_set_callback,
            10)
        self.subscription_limit_switch = self.create_subscription(
            Bool,
            'limit_switch',
            self.limit_switch_callback,
            10)
        
        # PID subscriptions. Allows for the changing of pid constants while testing
        self.subscription_r_P = self.create_subscription(
            Float32,
            'r_P',
            self.r_P_callback,
            10)
        self.subscription_r_I = self.create_subscription(
            Float32,
            'r_I',
            self.r_I_callback,
            10)
        self.subscription_r_D = self.create_subscription(
            Float32,
            'r_D',
            self.r_D_callback,
            10)
        self.subscription_r_pos_P = self.create_subscription(
            Float32,
            'r_pos_P',
            self.r_pos_P_callback,
            10)
        self.subscription_r_pos_I = self.create_subscription(
            Float32,
            'r_pos_I',
            self.r_pos_I_callback,
            10)
        self.subscription_r_pos_D = self.create_subscription(
            Float32,
            'r_pos_D',
            self.r_pos_D_callback,
            10)
        
        # Speed PID parameters
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05
        
        # Speed PID state
        self.r_setpoint = 0
        self.last_error = 0
        self.integral   = 0
        self.last_time  = time.time()
        self.speed      = 0

        # Position PID parameters
        self.pos_kp = 1.0
        self.pos_ki = 0.1
        self.pos_kd = 0.05
        
        # Position PID state
        self.pos_last_error = 0
        self.pos_integral   = 0
        self.position       = 0
        self.pos_settle     = 0
        self.pos_set        = 0

        # Motor controller initialization
        self.motor_controller = Board(1,0x10) # create a board object with the raspberry PI interface
        self.motor_controller.begin()
        self.motor_controller.set_moter_pwm_frequency(20000)# PWM frequency of 20kHz is selected because this frequency is above the human hearing range.
        self.motor_controller.set_encoder_enable(self.motor_controller.ALL)

        # PID Control loop, run every 0.01 s
        self.timer = self.create_timer(0.01, self.control_loop)

    def r_setpoint_callback(self,msg):
        """Saves the speed setpoint when it is published to the r_setpoint topic"""
        self.pos_set=0 # If a position was previously set, we no longer want to follow it
        self.setpoint=msg.data

    def r_pos_set_callback(self,msg):
        """Saves the position setpoint when it is published to the r_pos_set topic"""
        self.setpoint=0 # If a speed was previously set, we no longer want to follow it
        self.pos_set=msg.data

    def limit_switch_callback(self,msg):
        """Sets the position to 0 and sets the speed to 0 when the limit switch is hit"""
        self.position = 0
        self.setpoint = 0

    def r_P_callback(self,msg):
        self.kp=msg.data

    def r_I_callback(self,msg):
        self.ki=msg.data

    def r_D_callback(self,msg):
        self.kd=msg.data

    def r_pos_P_callback(self,msg):
        self.pos_kp=msg.data

    def r_pos_I_callback(self,msg):
        self.pos_ki=msg.data

    def r_pos_D_callback(self,msg):
        self.pos_kd=msg.data

    def control_loop(self):
        """Where all of the motor PID action happens"""
        now = time.time()
        dt = now - self.last_time # Calculate dt because it can be variable
        self.last_time = now

        # Integrate the speed to find the position. Position needs to be kept track of in both speed and position PID modes
        self.position += self.motor_controller.get_encoder_speed(self.motor_controller.M1)*dt

        # Check whether we are setting the speed or position of the motor
        if self.setpoint == 0: # if the position is set to zero then obey the speed PID
            output = self.speed_pid(dt)
        else: # Otherwise obey the position PID
            output = self.pos_pid(dt)
        
        # Stop commands above full
        if output>100:
            output = 100
        if output<-100:
            output = -100

        # Determine whether CW or CCW
        if output<0: # TODO Is CCW positive or CW
            self.motor_controller.motor_movement([self.motor_controller.M1], self.motor_controller.CW,output)
        else:
            self.motor_controller.motor_movement([self.motor_controller.M1], self.motor_controller.CCW,abs(output))

    def speed_pid(self,dt):
        """PID loop for the speed of the motor"""
        # If the speed setpoint is zero, just turn off the motor, this prevents strange behavior
        if self.r_setpoint == 0:
            self.last_error = 0
            self.integral   = 0
            return 0

        # Find and save current speed
        self.speed = self.motor_controller.get_encoder_speed(self.motor_controller.M1)

        # Compute error
        error = self.r_setpoint - self.speed

        # PID terms
        self.integral   += error * dt
        derivative      = (error - self.last_error) / dt
        output          = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.last_error = error
        return output
        
    def pos_pid(self,dt):
        """PID loop for the position of the motor"""
        # Compute error
        error = self.pos_set - self.position

        # PID terms
        self.pos_integral   += error * dt
        derivative          = (error - self.pos_last_error) / dt
        output              = self.pos_kp*error + self.pos_ki*self.pos_integral + self.pos_kd*derivative
        self.pos_last_error = error

        # If the real-time position is close to the setpoint
        if abs(self.position - self.pos_set)< 1:# TODO find what the tolerance should actually be here
            self.pos_settle += dt

            # If the real-time position has settled at the setpoint
            if self.pos_settle > 0.25: #TODO is .25 seconds acceptable?
                # Reset all position variables
                self.pos_last_error = 0
                self.pos_integral   = 0
                self.pos_settle     = 0
                self.pos_set        = 0
                output              = 0 # Turn off the motor, we've reached the goal
        return output

def main(args=None):
        try:
            with rclpy.init(args=args):
                r_motor = RMotor()
                rclpy.spin(r_motor)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass