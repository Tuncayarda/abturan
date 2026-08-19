#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Wrench
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Joy


def _desc(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


def _desc_int(text: str, lo: int, hi: int) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=text,
        integer_range=[IntegerRange(from_value=lo, to_value=hi, step=0)],
    )


def _desc_float(text: str, lo: float, hi: float) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=text,
        floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=0.0)],
    )


class JoyToWrenchNode(Node):
    def __init__(self):
        super().__init__('joy_to_wrench_node')

        self.declare_parameter('joy_topic', '/ui/joy_cmd_vel',
            _desc('Joystick input topic name'))
        self.declare_parameter('wrench_topic', '/control/wrench',
            _desc('Wrench output topic name'))
        self.declare_parameter('deadzone', 0.1,
            _desc_float('Joystick deadzone threshold', 0.0, 1.0))

        # Axis index mapping
        self.declare_parameter('axis_surge', 3,
            _desc_int('Surge (forward/backward) axis index', 0, 15))
        self.declare_parameter('axis_sway', 0,
            _desc_int('Sway (left/right) axis index', 0, 15))
        self.declare_parameter('axis_yaw', 1,
            _desc_int('Yaw (rotation) axis index', 0, 15))
        self.declare_parameter('axis_r2', 5,
            _desc_int('R2 trigger axis index (heave up)', 0, 15))
        self.declare_parameter('axis_l2', 4,
            _desc_int('L2 trigger axis index (heave down)', 0, 15))

        # Scale mapping from joystick [-1, 1] to normalized wrench request
        self.declare_parameter('scale_fx', 1.0,
            _desc_float('X force scale factor (surge)', 0.0, 10.0))
        self.declare_parameter('scale_fy', 1.0,
            _desc_float('Y force scale factor (sway)', 0.0, 10.0))
        self.declare_parameter('scale_fz', 1.0,
            _desc_float('Z force scale factor (heave)', 0.0, 10.0))
        self.declare_parameter('scale_tz', 1.0,
            _desc_float('Z torque scale factor (yaw)', 0.0, 10.0))

        joy_topic = self.get_parameter('joy_topic').value
        wrench_topic = self.get_parameter('wrench_topic').value

        self.subscription = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10,
        )
        self.publisher = self.create_publisher(Wrench, wrench_topic, 10)

        self.get_logger().info(
            f'Listening Joy: {joy_topic} | Publishing Wrench: {wrench_topic}'
        )

    def _deadzone(self, value: float) -> float:
        deadzone = float(self.get_parameter('deadzone').value)
        return 0.0 if abs(value) < deadzone else value

    @staticmethod
    def _axis(axes, idx: int) -> float:
        if idx < 0 or idx >= len(axes):
            return 0.0
        return float(axes[idx])

    def joy_callback(self, msg: Joy) -> None:
        axis_surge = int(self.get_parameter('axis_surge').value)
        axis_sway = int(self.get_parameter('axis_sway').value)
        axis_yaw = int(self.get_parameter('axis_yaw').value)
        axis_r2 = int(self.get_parameter('axis_r2').value)
        axis_l2 = int(self.get_parameter('axis_l2').value)

        scale_fx = float(self.get_parameter('scale_fx').value)
        scale_fy = float(self.get_parameter('scale_fy').value)
        scale_fz = float(self.get_parameter('scale_fz').value)
        scale_tz = float(self.get_parameter('scale_tz').value)

        surge = self._deadzone(-self._axis(msg.axes, axis_surge))
        sway = self._deadzone(self._axis(msg.axes, axis_sway))
        yaw = self._deadzone(self._axis(msg.axes, axis_yaw))

        # Triggerlar ile heave: R2 - L2
        r2 = self._deadzone(self._axis(msg.axes, axis_r2))
        l2 = self._deadzone(self._axis(msg.axes, axis_l2))
        heave = self._deadzone((r2 - l2) * 0.5)

        wrench = Wrench()
        wrench.force.x = surge * scale_fx
        wrench.force.y = sway * scale_fy
        wrench.force.z = heave * scale_fz
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = yaw * scale_tz

        self.publisher.publish(wrench)


def main() -> None:
    rclpy.init()
    node = JoyToWrenchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
