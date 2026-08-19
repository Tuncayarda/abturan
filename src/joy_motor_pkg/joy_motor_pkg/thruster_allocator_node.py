#!/usr/bin/env python3

from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


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


class ThrusterAllocatorNode(Node):
	def __init__(self):
		super().__init__('thruster_allocator_node')

		self.declare_parameter('wrench_topic', '/control/wrench',
			_desc('Input wrench topic name'))
		self.declare_parameter('pwm_topic', '/control/pwm_cmds',
			_desc('Output PWM command topic name'))
		self.declare_parameter('smoothing_alpha', 0.6,
			_desc_float('PWM exponential smoothing factor', 0.0, 0.99))

		# 6 thrusters: the STM32 board carries exactly six ESC PWM outputs
		# (PB0, PB1, PB6, PB7, PB8, PB9 -> TIM3_CH3/4, TIM4_CH1..4).
		self.declare_parameter('thruster_count', 6,
			_desc_int('Number of thrusters (STM32 has 6 ESC PWM outputs)', 1, 16))

		self.declare_parameter(
			'allocation_matrix',
			[
				# Fx (surge)      ESC1  ESC2  ESC3  ESC4  ESC5  ESC6
				1.0, 1.0, 0.0, 0.0, 0.0, 0.0,

				# Fy (sway) - no authority with this layout
				0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

				# Fz (heave)
				0.0, 0.0, 1.0, 1.0, 1.0, 1.0,

				# Tx (roll)
				0.0, 0.0, 1.0, -1.0, 1.0, -1.0,

				# Ty (pitch)
				0.0, 0.0, 1.0, 1.0, -1.0, -1.0,

				# Tz (yaw)
				-1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
			],
			_desc('6xN allocation matrix (Fx,Fy,Fz,Tx,Ty,Tz x thruster count), row-major flat array'),
		)

		self.declare_parameter('pwm_min', 0,
			_desc_int('Minimum PWM value', 0, 2000))
		self.declare_parameter('pwm_neutral', 500,
			_desc_int('Neutral (zero thrust) PWM value', 0, 2000))
		self.declare_parameter('pwm_max', 1000,
			_desc_int('Maximum PWM value', 0, 2000))

		self.thruster_count = int(self.get_parameter('thruster_count').value)
		self.prev_pwm = [int(self.get_parameter('pwm_neutral').value)] * self.thruster_count

		wrench_topic = self.get_parameter('wrench_topic').value
		pwm_topic = self.get_parameter('pwm_topic').value

		self.subscription = self.create_subscription(
			Wrench,
			wrench_topic,
			self.wrench_callback,
			10,
		)
		# Consumer: stm32_bridge_node, which maps pwm_min..pwm_max onto the
		# 1000-2000 us ESC pulse and forwards it over UART to the STM32.
		self.publisher = self.create_publisher(Int32MultiArray, pwm_topic, 10)

		self.get_logger().info(
			f'Listening Wrench: {wrench_topic} | Publishing PWM: {pwm_topic}'
		)

	def _load_allocation_matrix(self) -> np.ndarray:
		values = self.get_parameter('allocation_matrix').value
		expected_len = 6 * self.thruster_count
		if len(values) != expected_len:
			self.get_logger().warn(
				f'allocation_matrix length must be {expected_len}, got {len(values)}. '
				'Using identity-like fallback.'
			)
			mat = np.zeros((6, self.thruster_count), dtype=float)
			for i in range(min(6, self.thruster_count)):
				mat[i, i] = 1.0
			return mat
		return np.array(values, dtype=float).reshape((6, self.thruster_count))

	@staticmethod
	def _clamp(value: float, low: float, high: float) -> float:
		return max(low, min(high, value))

	def _to_pwm(self, cmd: float) -> int:
		pwm_min = int(self.get_parameter('pwm_min').value)
		pwm_neutral = int(self.get_parameter('pwm_neutral').value)
		pwm_max = int(self.get_parameter('pwm_max').value)

		pos_span = float(pwm_max - pwm_neutral)
		neg_span = float(pwm_neutral - pwm_min)
		cmd = self._clamp(cmd, -1.0, 1.0)

		if cmd >= 0.0:
			pwm = pwm_neutral + cmd * pos_span
		else:
			pwm = pwm_neutral + cmd * neg_span

		return int(round(self._clamp(pwm, pwm_min, pwm_max)))

	def _smooth_pwm(self, idx: int, new_pwm: int) -> int:
		alpha = float(self.get_parameter('smoothing_alpha').value)
		alpha = self._clamp(alpha, 0.0, 0.99)

		old_pwm = self.prev_pwm[idx]
		filtered = int(round(alpha * old_pwm + (1.0 - alpha) * new_pwm))
		self.prev_pwm[idx] = filtered
		return filtered

	def _normalize(self, thruster_cmd: np.ndarray) -> np.ndarray:
		max_abs = float(np.max(np.abs(thruster_cmd))) if thruster_cmd.size else 1.0
		if max_abs > 1.0:
			return thruster_cmd / max_abs
		return thruster_cmd

	def wrench_callback(self, msg: Wrench) -> None:
		wrench_vec = np.array(
			[
				msg.force.x,
				msg.force.y,
				msg.force.z,
				msg.torque.x,
				msg.torque.y,
				msg.torque.z,
			],
			dtype=float,
		)

		b_matrix = self._load_allocation_matrix()
		pinv = np.linalg.pinv(b_matrix)
		thruster_cmd = pinv @ wrench_vec
		thruster_cmd = self._normalize(thruster_cmd)

		pwm_values: List[int] = []
		for i in range(self.thruster_count):
			raw_pwm = self._to_pwm(float(thruster_cmd[i]))
			pwm_values.append(self._smooth_pwm(i, raw_pwm))

		out = Int32MultiArray()
		out.data = pwm_values
		self.publisher.publish(out)


def main() -> None:
	rclpy.init()
	node = ThrusterAllocatorNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
