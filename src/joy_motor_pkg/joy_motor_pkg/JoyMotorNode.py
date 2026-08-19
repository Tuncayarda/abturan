#!/usr/bin/env python3
"""
ESKI (legacy) node — hicbir launch dosyasinda kullanilmiyor.

Jetson kurulumundan gelen, 8 itici icin joystick eksenlerini dogrudan PWM'e
ceviren ilk surum. Bugunku zincir bunun yerine iki adima ayrildi ve itici
sayisi 6'ya dustu (STM32'de 6 ESC cikisi var):

    /ui/joy_cmd_vel -> joy_to_wrench -> /control/wrench
                    -> thruster_allocator -> /control/pwm_cmds
                    -> stm32_bridge -> UART -> STM32

Buradaki 8 kanal ve /motor_power konusu artik kimse tarafindan tuketilmiyor;
referans olarak duruyor.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy 
from std_msgs.msg import Int32MultiArray


class JoyMotorNode(Node):

    def __init__(self):
        super().__init__('joy_motor_node')

        self.DEADZONE = 0.1

        self.subscription = self.create_subscription(
            Joy,
            '/ui/joy_cmd_vel',
            self.joy_callback,
            10
        )

        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/motor_power',
            10
        )

        self.prev_motor = [500] * 8

    # ===============================
    # Utility Functions 
    # ===============================

    def apply_deadzone(self, value):
        if abs(value) < self.DEADZONE:
            return 0.0
        return value

    def clamp(self, value, min_v=-1.0, max_v=1.0):
        return max(min_v, min(max_v, value))

    def map_pwm(self, value):
        value = self.clamp(value)
        return int(500 + value * 500)

    def smooth(self, index, new_value, alpha=0.7):
        old = self.prev_motor[index]
        filtered = int(alpha * old + (1 - alpha) * new_value)

        self.prev_motor[index] = filtered
        return filtered

    # ===============================
    # Main Control Callback
    # ===============================

    def joy_callback(self, msg: Joy):

        motor = [500] * 8

        # ======================================================
        # Movement Control (Axis[3] → Drive speed)
        # ======================================================

        drive = self.apply_deadzone(
            -msg.axes[3] if len(msg.axes) > 3 else 0
        )

        drive_pwm = self.map_pwm(drive)

        motor[0] = self.smooth(0, 1000 - drive_pwm)
        motor[1] = self.smooth(1, 1000 - drive_pwm)

        motor[6] = self.smooth(6, drive_pwm)
        motor[7] = self.smooth(7, drive_pwm)
        
		# ======================================================
        # Axis[0] Differential Control (Your Requested Logic)
        # ======================================================

        axis0 = self.apply_deadzone(
            msg.axes[0] if len(msg.axes) > 0 else 0
        )

        pwm = self.map_pwm(abs(axis0))

        # axis0 < 0 → 2,4 negatif | 3,5 pozitif
        # axis0 > 0 → 2,4 pozitif | 3,5 negatif

        if axis0 < 0:
            motor[2] = self.smooth(2, 500 - pwm)
            motor[4] = self.smooth(4, 500 - pwm)

            motor[3] = self.smooth(3, 500 + pwm)
            motor[5] = self.smooth(5, 500 + pwm)

        elif axis0 > 0:
            motor[2] = self.smooth(2, 500 + pwm)
            motor[4] = self.smooth(4, 500 + pwm)

            motor[3] = self.smooth(3, 500 - pwm)
            motor[5] = self.smooth(5, 500 - pwm)



        # ======================================================
        # Axis[1] Differential Mixing Control
        # ======================================================

        axis1 = self.apply_deadzone(
            msg.axes[1] if len(msg.axes) > 1 else 0
        )

        mix_pwm = self.map_pwm(axis1)

        # Group mixing
        motor[2] = self.smooth(2, mix_pwm)
        motor[3] = self.smooth(3, mix_pwm)
        motor[4] = self.smooth(4, 1000 - mix_pwm)
        motor[5] = self.smooth(5, 1000 - mix_pwm)


        # ======================================================
        # Button Control
        # ======================================================

        R1 = msg.buttons[10] if len(msg.buttons) > 10 else 0
        L1 = msg.buttons[9] if len(msg.buttons) > 9 else 0

        R2 = msg.axes[5] if len(msg.axes) > 5 else 0
        L2 = msg.axes[4] if len(msg.axes) > 4 else 0

        if L1:
            motor[1] = 0
            motor[6] = 0
 

        if R1:
            motor[6] = 1000
            motor[1] = 1000

        if abs(R2) > 0.1:
            pwm = self.map_pwm(R2)

            motor[2] = pwm
            motor[3] = pwm
            motor[4] = pwm
            motor[5] = pwm

        if abs(L2) > 0.1:
            pwm = self.map_pwm(-L2)

            motor[2] = pwm
            motor[3] = pwm
            motor[4] = pwm
            motor[5] = pwm

        # ======================================================
        # Publish
        # ======================================================

        out_msg = Int32MultiArray()
        out_msg.data = motor

        self.publisher.publish(out_msg)


# ===============================
# Main
# ===============================

def main():
    rclpy.init()

    node = JoyMotorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()