import rclpy
import RPi.GPIO as GPIO
import time
from rclpy.node import Node
from MotorModule import Motor
from sensor_msgs.msg import MagneticField

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.motor = Motor(17, 27, 22, 23, 24, 25)

    # def get_key(self, key_name):
    #     """Check if a specific key is pressed."""
    #     ans = False
    #     for event in pygame.event.get():
    #         pass  # Process events to avoid freezing
    #     key_input = pygame.key.get_pressed()
    #     my_key = getattr(pygame, f'K_{key_name}')
    #     if key_input[my_key]:
    #         ans = True
    #     pygame.display.update()
    #     return ans

    def move_forward(self):
        self.get_logger().info("Moving forward...")
        GPIO.output(self.right_wheel_backward, False)
        GPIO.output(self.left_wheel_backward, False)
        self.right_wheel_pwm.ChangeDutyCycle(80)
        self.left_wheel_pwm.ChangeDutyCycle(80)

    def move_backward(self):
        self.get_logger().info("Moving backward...")
        GPIO.output(self.right_wheel_forward, False)
        GPIO.output(self.left_wheel_forward, False)
        GPIO.output(self.right_wheel_backward, True)
        GPIO.output(self.left_wheel_backward, True)


    def turn_left(self):
        self.get_logger().info("Turning left...")
        GPIO.output(self.right_wheel_backward, False)
        GPIO.output(self.left_wheel_backward, False)
        GPIO.output(self.right_wheel_forward, True)
        GPIO.output(self.left_wheel_backward, True)


    def turn_right(self):
        self.get_logger().info("Turning right...")
        GPIO.output(self.right_wheel_backward, False)
        GPIO.output(self.left_wheel_backward, False)
        GPIO.output(self.right_wheel_backward, True)
        GPIO.output(self.left_wheel_forward, True)


    def stop_motors(self):
        self.get_logger().info("Stopping motors...")
        GPIO.output(self.right_wheel_forward, False)
        GPIO.output(self.right_wheel_backward, False)
        GPIO.output(self.left_wheel_forward, False)
        GPIO.output(self.left_wheel_backward, False)
        self.right_wheel_pwm.ChangeDutyCycle(0)
        self.left_wheel_pwm.ChangeDutyCycle(0)

    def run(self):
        try:
            self.motor.move(0.4, 0.2, 0.5)
            # while rclpy.ok():
            #     if self.get_key('UP'):
            #         self.move_forward()
            #     elif self.get_key('DOWN'):
            #         self.move_backward()
            #     elif self.get_key('LEFT'):
            #         self.turn_left()
            #     elif self.get_key('RIGHT'):
            #         self.turn_right()
            #     else:
            #         self.stop_motors()
            #     time.sleep(0.1)
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down keyboard controller...")
        finally:
            self.stop_motors()
            GPIO.cleanup()
            # pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    controller = KeyboardController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

