import rclpy
from rclpy.node import Node, Parameter
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature

from std_srvs.srv import SetBool

from mf_hyperdrive.pca9685 import PCA9685
from mf_hyperdrive.ms5837 import ms5837


def bound(value, low, high):
    """Bound a value between a min and max."""
    return min(high, max(low, value))


# Default parameters (name, default value, description)
DEFAULT_PARAMS = [
    ("pwm_freq_hz", 200, "PWM and LED frequency in Hz"),
    ("pwm_value_center", 1275, "PWM duty cycle (12-bit) for center value (1500us)"),
    (
        "pwm_value_forward",
        1575,
        "PWM duty cycle (12-bit) for full-forward value (1900us)",
    ),
    (
        "pwm_value_reverse",
        925,
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
SERVO_PITCH_CHANNEL = 11  # Servo1 output
SERVO_ROLL_CHANNEL = 10  # Servo2 output


class Hyperdrive(Node):
    def __init__(self):
        super().__init__("hyperdrive")

        self.depth_m: float = 0.0
        self.temp_c: float = 0.0
        self.led_power: int = 0
        self.motors: list[float] = [0.0] * 9
        self.servos: list[float] = [0.0] * 2

        # Initialize topics
        self.depth_pub = self.create_publisher(Float64, "depth", 10)
        self.temp_pub = self.create_publisher(Temperature, "temp", 10)
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
        self.pca9685.frequency = 200

        # Instantiate depth sensor
        self.get_logger().info("Instantiating depth sensor")
        self.ms5837 = ms5837.MS5837(ms5837.MODEL_02BA, 0)
        if not self.ms5837.init():
            self.get_logger().fatal("Depth sensor not found!")
            rclpy.shutdown()
        self.ms5837.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        self.ms5837.read(ms5837.OSR_256)
        self.initial_depth = self.ms5837.depth()
        self.get_logger().info(f"Initial depth: {self.initial_depth:0.02f}") 

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
        center = self.get_parameter("pwm_value_center").value

        if percent >= 0:
            forward = self.get_parameter("pwm_value_forward").value
            res = (forward - center) / 100.0 * percent + center
        else:
            reverse = self.get_parameter("pwm_value_reverse").value
            res = (center - reverse) / 100.0 * percent + center

        return res

    def pub_timer_callback(self):
        """Publish depth and temperature data at a fixed interval."""
        depth_msg = Float64()
        self.ms5837.read(ms5837.OSR_256)
        self.depth_m = self.ms5837.depth() - self.initial_depth

        depth_msg.data = self.depth_m
        self.depth_pub.publish(depth_msg)

        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        self.temp_c = self.ms5837.temperature()
        temp_msg.temperature = self.temp_c
        self.temp_pub.publish(temp_msg)

    def led_callback(self, msg):
        """Update LED power level on new message."""
        self.led_power = bound(msg.data, 0, 100)
        max_output = self.get_parameter("led_max_duty").value

        output = int(self.led_power * max_output / 100.0)

        self.get_logger().info(
            f"Setting LED power level to {self.led_power} (duty: {output})"
        )
        self.pca9685.set_pwm(LED_CHANNEL, output)

    def motors_callback(self, msg):
        """Update thruster/motor ESC outputs on new message."""
        self.motors = list(map(lambda x: self.remap_motor(bound(x, -100.0, 100.0)), msg.data))

        self.get_logger().info(str(self.motors))

        for i in range(9):
            self.pca9685.set_pwm(i, int(self.motors[i]))

    def servo_callback(self, msg, servo_id):
        """Update gimbal servo outputs on new message."""
        self.servos[servo_id] = bound(msg.data, -100.0, 100.0)
        self.get_logger().info(
            f"Setting servo {servo_id+1} to {self.servos[servo_id]}"
        )
        # TODO: re-write in terms of angles
        duty = int(self.remap_motor(self.servos[servo_id]))
        self.get_logger().info(f"Calculated duty: {duty}")
        self.pca9685.set_pwm(
            servo_id + SERVO_PITCH_CHANNEL, duty
        )

    def servo1_callback(self, msg):
        self.servo_callback(msg, 0)

    def servo2_callback(self, msg):
        self.servo_callback(msg, 1)


def main(args=None):
    rclpy.init(args=args)

    hyperdrive = Hyperdrive()

    rclpy.spin(hyperdrive)

    hyperdrive.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
