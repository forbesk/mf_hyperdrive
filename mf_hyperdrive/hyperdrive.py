from typing import List
import rclpy
from rclpy.node import Node, Parameter
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature

from std_srvs.srv import SetBool

from mf_hyperdrive.pca9685 import PCA9685
from mf_hyperdrive.powercore import PowerCore, PowerCoreMsg


def bound(value, low, high):
    """Bound a value between a min and max."""
    return min(high, max(low, value))


# Default parameters (name, default value, description)
DEFAULT_PARAMS = [
    ("pwm_freq_hz", 200, "PWM and LED frequency in Hz"),
    ("pwm_value_center", 1228, "PWM duty cycle (12-bit) for center value (1500us)"),
    (
        "pwm_value_forward",
        1556,
        "PWM duty cycle (12-bit) for full-forward value (1900us)",
    ),
    (
        "pwm_value_reverse",
        901,
        "PWM duty cycle (12-bit) for full-reverse value (1100us)",
    ),
    ("led_max_duty", 2047, "LED duty cycle (12-bit) for full on"),
    (
        "gimbal_pitch_center",
        1228,
        "PWM duty cycle (12-bit) for gimbal pitch center (0deg)",
    ),
    ("gimbal_pitch_pos45", 1392, "PWM duty cycle (12-bit) for gimbal pitch pos 45deg"),
    ("gimbal_pitch_neg45", 1064, "PWM duty cycle (12-bit) for gimbal pitch neg 45deg"),
    (
        "gimbal_roll_center",
        1228,
        "PWM duty cycle (12-bit) for gimbal roll center (0deg)",
    ),
    ("gimbal_roll_pos45", 1392, "PWM duty cycle (12-bit) for gimbal roll pos 45deg"),
    ("gimbal_roll_neg45", 1064, "PWM duty cycle (12-bit) for gimbal roll neg 45deg"),
]

LED_CHANNEL = 12
SERVO_PITCH_CHANNEL = 10  # Servo1 output
SERVO_ROLL_CHANNEL = 11  # Servo2 output


class Hyperdrive(Node):
    def __init__(self):
        super().__init__("hyperdrive")

        self.depth_m: float = 0.0
        self.temp_c: float = 0.0
        self.led_power: int = 0
        self.motors: List[float] = [0.0] * 9
        self.servos: List[float] = [0.0] * 2
        self.mission: bool = False
        self.voltage: float = 0.0
        self.currents: List[float] = [0.0] * 12

        # Initialize PWM generator
        self.pca9685 = PCA9685()

        # Initialize topics
        self.depth_pub = self.create_publisher(Float64, "depth", 10)
        self.temp_pub = self.create_publisher(Temperature, "temp", 10)
        self.voltage_pub = self.create_publisher(Float64, "battery_voltage", 10)
        self.mission_pub = self.create_publisher(Bool, "mission_switch", 10)
        self.currents_pub = self.create_publisher(Float64MultiArray, "currents", 10)

        self.led_power_sub = self.create_subscription(
            UInt8, "led", self.led_callback, 10
        )
        self.motors_sub = self.create_subscription(
            Float64MultiArray, "motors", self.motors_callback, 10
        )
        self.servo1_sub = self.create_subscription(
            Float64, "servo1", self.servo1_callback, 10
        )
        self.servo2_sub = self.create_subscription(
            Float64, "servo2", self.servo2_callback, 10
        )

        # Initialize parameters
        self.declare_parameters(
            "",
            [
                (name, default, ParameterDescriptor(description=desc))
                for (name, default, desc) in DEFAULT_PARAMS
            ],
        )

        # Initialize services
        self.create_service(SetBool, "enable_pwm", self.enable_pwm_callback)

        # Create publish timer
        pub_timer_period = 0.02
        self.pub_timer = self.create_timer(pub_timer_period, self.pub_timer_callback)

        # Instantiate PWM generator
        self.get_logger().info("Instantiating PCA9685")
        self.pca9685 = PCA9685()

        # TODO: instantiate depth sensor
        self.get_logger().info("Instantiating depth sensor")

        # Instantiate power core
        self.get_logger().info("Instantiating power core")
        self.pc = PowerCore(callback=self.power_callback)
        self.pc.connect()

    def enable_pwm_callback(self, request, response):
        """ROS service to enable or disable the PCA9685 PWM generation."""
        self.pca9685.enabled = request.data
        response.success = True
        return response

    def remap_motor(self, percent):
        """
        Remaps a desired bidirectional ESC output (-100% to 100%) to a 12-bit
        PWM duty cycle (0-4095).

        Remapping is performed via linear interpolation between the center
        duty cycle and the forward or reverse maximum.
        """
        center = self.get_parameter("pwm_value_center")

        if percent >= 0:
            forward = self.get_parameter("pwm_value_forward")
            res = (forward - center) / 100.0 * percent + center
        else:
            reverse = self.get_parameter("pwm_value_reverse")
            res = (reverse - center) / 100.0 * percent + center

        return res

    def pub_timer_callback(self):
        """Publish depth and temperature data at a fixed interval."""
        depth_msg = Float64()
        depth_msg.data = self.depth_m
        self.depth_pub.publish(depth_msg)

        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.temperature = self.temp_c
        self.temp_pub.publish(temp_msg)

    def led_callback(self, msg):
        """Update LED power level on new message."""
        self.led_power = bound(msg.data, 0, 100)
        max_output = self.get_parameter("led_max_duty")
        output = int(self.led_power * max_output / 4095.0)

        self.get_logger().debug(
            f"Setting LED power level to {self.led_power} (duty: {output})"
        )
        self.pca9685.set_pwm(LED_CHANNEL, output)

    def motors_callback(self, msg):
        """Update thruster/motor ESC outputs on new message."""
        self.motors = map(lambda x: self.remap_motor(bound(x, -100.0, 100.0)), msg.data)

        for i in range(10):
            self.pca9685.set_pwm(self.motors[i])

    def servo_callback(self, msg, servo_id):
        """Update gimbal servo outputs on new message."""
        self.servos[servo_id] = bound(msg.data, -100.0, 100.0)
        self.get_logger().debug(
            f"Setting servo {servo_id+1} to {self.servos[servo_id]}"
        )
        # TODO: re-write in terms of angles
        self.pca9685.set_pwm(
            servo_id + SERVO_PITCH_CHANNEL, self.remap_motor(self.servos[servo_id])
        )

    def servo1_callback(self, msg):
        self.servo_callback(msg, 0)

    def servo2_callback(self, msg):
        self.servo_callback(msg, 1)

    def power_callback(self, msg: PowerCoreMsg):
        self.mission_pub.publish(Bool(data=msg.mission))
        self.voltage_pub.publish(Float64(data=msg.voltage))
        self.currents_pub.publish(Float64MultiArray(data=msg.currents))


def main(args=None):
    rclpy.init(args=args)

    hyperdrive = Hyperdrive()

    rclpy.spin(hyperdrive)

    hyperdrive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
