import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# GPIO Pin Configuration
LEFT_IN1 = 32  # Direction pin 1 for left motor
LEFT_IN2 = 33  # Direction pin 2 for left motor
RIGHT_IN1 = 29  # Direction pin 1 for right motor
RIGHT_IN2 = 31  # Direction pin 2 for right motor

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # ROS 2 Subscriber to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2], GPIO.OUT)

        # PWM Setup
        self.left_pwm_forward = GPIO.PWM(LEFT_IN1, 100)  # 100 Hz PWM for forward motion
        self.left_pwm_backward = GPIO.PWM(LEFT_IN2, 100)  # 100 Hz PWM for backward motion
        self.right_pwm_forward = GPIO.PWM(RIGHT_IN1, 100)  # 100 Hz PWM for forward motion
        self.right_pwm_backward = GPIO.PWM(RIGHT_IN2, 100)  # 100 Hz PWM for backward motion

        self.left_pwm_forward.start(0)
        self.left_pwm_backward.start(0)
        self.right_pwm_forward.start(0)
        self.right_pwm_backward.start(0)

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # Calculate wheel speeds (simplified differential drive model)
        left_speed = linear - angular
        right_speed = linear + angular

        # Scale speeds to PWM values (0-100)
        left_pwm_value = max(min(abs(left_speed * 100), 100), 0)
        right_pwm_value = max(min(abs(right_speed * 100), 100), 0)

        # Set PWM for left motor
        if left_speed > 0:  # Forward
            self.left_pwm_forward.ChangeDutyCycle(left_pwm_value)
            self.left_pwm_backward.ChangeDutyCycle(0)
        else:  # Backward
            self.left_pwm_forward.ChangeDutyCycle(0)
            self.left_pwm_backward.ChangeDutyCycle(left_pwm_value)

        # Set PWM for right motor
        if right_speed > 0:  # Forward
            self.right_pwm_forward.ChangeDutyCycle(right_pwm_value)
            self.right_pwm_backward.ChangeDutyCycle(0)
        else:  # Backward
            self.right_pwm_forward.ChangeDutyCycle(0)
            self.right_pwm_backward.ChangeDutyCycle(right_pwm_value)

    def destroy_node(self):
        # Cleanup GPIO and PWM
        self.left_pwm_forward.stop()
        self.left_pwm_backward.stop()
        self.right_pwm_forward.stop()
        self.right_pwm_backward.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass

    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
