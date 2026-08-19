#!/usr/bin/env python3
"""
IMU node'u — Raspberry Pi'nin I2C hattina bagli BNO08x'i (BNO085/BNO086) okur.

IMU bilerek Pi tarafinda: I2C okumasi zamanlama acisindan toleransli
(eyleyiciler gibi mikro saniye hassasiyeti istemiyor) ve veriyi tuketen
her sey (kinematik, arayuz, kayit) zaten Pi'de calisiyor. STM32 sadece
eyleyicileri suruyor.

MPU6050 kurulumundan farki: yonelim fuzyonu artik yongada. Burada tumleyici
filtre yok; rotation vector dogrudan quaternion olarak geliyor ve manyetometre
de fuzyona dahil oldugu icin **yaw kaymiyor**.

Kablolama:
    GPIO2 / SDA (pin 3) -> BNO SDA
    GPIO3 / SCL (pin 5) -> BNO SCL
    3.3 V       (pin 1) -> BNO VIN
    GND         (pin 9) -> BNO GND

Yayinladigi konular
-------------------
/imu/data_raw      sensor_msgs/Imu            ham ivme + acisal hiz
/imu/data          sensor_msgs/Imu            + yonganin fuzyon yonelimi
/imu/mag           sensor_msgs/MagneticField  manyetik alan (istege bagli)

Not: BNO08x sicaklik raporu sunmuyor, bu yuzden /imu/temperature yok.
"""

import rclpy
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange,
                                ParameterDescriptor)
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from imu_bridge.bno08x import (ACCELEROMETER, ACCURACY_NAMES, Bno08x,
                               GYROSCOPE, MAGNETIC_FIELD, ROTATION_REPORTS)
from imu_bridge.i2c import I2CError


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


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.i2c_bus = self.declare_parameter(
            'i2c_bus', 1,
            _desc_int('I2C veri yolu numarasi (GPIO2/3 = i2c-1)', 0, 20),
        ).value
        self.i2c_address = self.declare_parameter(
            'i2c_address', 0x4A,
            _desc_int('BNO08x I2C adresi (ADR=GND -> 0x4A, ADR=VCC -> 0x4B)',
                      0x03, 0x77),
        ).value
        self.frame_id = self.declare_parameter(
            'frame_id', 'imu_link', _desc('Imu mesajlarinin frame_id alani'),
        ).value
        self.rate_hz = self.declare_parameter(
            'rate_hz', 100.0,
            _desc_float('Yonganin rapor akis frekansi (Hz)', 1.0, 400.0),
        ).value
        self.orientation_mode = self.declare_parameter(
            'orientation_mode', 'rotation_vector',
            _desc('Yonelim kaynagi: rotation_vector (9 eksen, manyetometreli, '
                  'yaw kaymaz) | game_rotation_vector (manyetometresiz, manyetik '
                  'gurultuye bagisik ama yaw kayar) | '
                  'geomagnetic_rotation_vector (dusuk guc) | none'),
        ).value
        self.publish_mag = self.declare_parameter(
            'publish_mag', False,
            _desc('/imu/mag uzerinde manyetik alan yayinla'),
        ).value
        self.soft_reset_on_start = self.declare_parameter(
            'soft_reset_on_start', True,
            _desc('Aciliste yongaya yumusak reset at (bilinen durumdan basla)'),
        ).value
        self.use_reported_accuracy = self.declare_parameter(
            'use_reported_accuracy', True,
            _desc('Yonelim kovaryansi icin yonganin bildirdigi dogruluk '
                  'kestirimini kullan; kapaliysa orientation_stddev kullanilir'),
        ).value
        self.accel_stddev = self.declare_parameter(
            'accel_stddev', 0.03,
            _desc_float('Ivme olcum std sapmasi (m/s^2), kovaryans icin', 0.0, 10.0),
        ).value
        self.gyro_stddev = self.declare_parameter(
            'gyro_stddev', 0.003,
            _desc_float('Acisal hiz std sapmasi (rad/s), kovaryans icin', 0.0, 10.0),
        ).value
        self.orientation_stddev = self.declare_parameter(
            'orientation_stddev', 0.02,
            _desc_float('Yonelim std sapmasi (rad), kovaryans icin', 0.0, 10.0),
        ).value
        self.mag_stddev = self.declare_parameter(
            'mag_stddev', 1.0,
            _desc_float('Manyetik alan std sapmasi (uT), kovaryans icin', 0.0, 100.0),
        ).value

        if self.orientation_mode not in ROTATION_REPORTS and \
                self.orientation_mode != 'none':
            raise ValueError(
                f'orientation_mode "{self.orientation_mode}" gecersiz, '
                f'secenekler: {sorted(ROTATION_REPORTS)} veya "none"')

        self.imu = Bno08x(
            bus=self.i2c_bus,
            address=self.i2c_address,
            soft_reset=self.soft_reset_on_start,
        )
        self.get_logger().info(
            f'BNO08x bulundu: {self.imu.version_string} @ '
            f'i2c-{self.i2c_bus} 0x{self.i2c_address:02X}')

        self._interval_us = int(1e6 / self.rate_hz)
        self._enable_reports()

        self._raw_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self._data_pub = self.create_publisher(Imu, '/imu/data', 10)
        self._mag_pub = None
        if self.publish_mag:
            self._mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        self._error_count = 0
        self._last_quat_status = None

        # Kuyrugu rapor hizinin iki kati sikligda bosaltiyoruz: yonga veriyi
        # kendiliginden akittigi icin biz sadece geride kalmamaya bakiyoruz.
        self._timer = self.create_timer(0.5 / self.rate_hz, self._on_tick)

    def _enable_reports(self):
        self.imu.enable_report(ACCELEROMETER, self._interval_us)
        self.imu.enable_report(GYROSCOPE, self._interval_us)
        if self.publish_mag:
            # Manyetometre 100 Hz'in uzerine cikmiyor; ivme/jiro kadar hizli
            # istemeye de gerek yok.
            self.imu.enable_report(MAGNETIC_FIELD, max(self._interval_us, 20000))
        if self.orientation_mode != 'none':
            self.imu.enable_report(ROTATION_REPORTS[self.orientation_mode],
                                   self._interval_us)
        self.get_logger().info(
            f'Raporlar acildi: {self.rate_hz:.0f} Hz, yonelim = '
            f'{self.orientation_mode}'
            + (', manyetometre acik' if self.publish_mag else ''))

    # ------------------------------------------------------------------
    def _on_tick(self):
        try:
            self.imu.service()
        except I2CError as error:
            self._error_count += 1
            # Her okumada log basmak dongu frekansinda spam olur; seyrek bas.
            if self._error_count % 100 == 1:
                self.get_logger().warn(
                    f'IMU okunamadi ({self._error_count}. hata): {error}')
            return

        if self.imu.reset_detected:
            # Yonga kendini resetlemis (besleme dalgalanmasi, WDT...);
            # raporlar sifirlandi, geri acmazsak veri akisi durur.
            self.get_logger().warn('BNO08x resetlendi, raporlar yeniden aciliyor')
            try:
                self.imu.reenable_reports()
            except I2CError as error:
                self.get_logger().warn(f'Raporlar geri acilamadi: {error}')
            return

        stamp = self.get_clock().now().to_msg()

        if self.imu.new_motion and self.imu.accel and self.imu.gyro:
            self.imu.new_motion = False
            self._publish_imu(stamp)

        if self.imu.new_mag and self._mag_pub is not None and self.imu.mag:
            self.imu.new_mag = False
            self._publish_mag(stamp)

    def _publish_imu(self, stamp):
        acc_var = self.accel_stddev ** 2
        gyro_var = self.gyro_stddev ** 2

        raw = Imu()
        raw.header.stamp = stamp
        raw.header.frame_id = self.frame_id
        raw.linear_acceleration.x = self.imu.accel[0]
        raw.linear_acceleration.y = self.imu.accel[1]
        raw.linear_acceleration.z = self.imu.accel[2]
        raw.angular_velocity.x = self.imu.gyro[0]
        raw.angular_velocity.y = self.imu.gyro[1]
        raw.angular_velocity.z = self.imu.gyro[2]
        raw.linear_acceleration_covariance = [acc_var, 0.0, 0.0,
                                              0.0, acc_var, 0.0,
                                              0.0, 0.0, acc_var]
        raw.angular_velocity_covariance = [gyro_var, 0.0, 0.0,
                                           0.0, gyro_var, 0.0,
                                           0.0, 0.0, gyro_var]
        # data_raw'da yonelim yok: REP-145'e gore ilk elemani -1 yapiyoruz.
        raw.orientation_covariance = [-1.0] + [0.0] * 8
        self._raw_pub.publish(raw)

        if self.orientation_mode == 'none' or self.imu.quaternion is None:
            return

        msg = Imu()
        msg.header = raw.header
        msg.linear_acceleration = raw.linear_acceleration
        msg.angular_velocity = raw.angular_velocity
        msg.linear_acceleration_covariance = raw.linear_acceleration_covariance
        msg.angular_velocity_covariance = raw.angular_velocity_covariance
        msg.orientation.x = self.imu.quaternion[0]
        msg.orientation.y = self.imu.quaternion[1]
        msg.orientation.z = self.imu.quaternion[2]
        msg.orientation.w = self.imu.quaternion[3]

        ori_var = self._orientation_variance()
        msg.orientation_covariance = [ori_var, 0.0, 0.0,
                                      0.0, ori_var, 0.0,
                                      0.0, 0.0, ori_var]
        # Bayragi burada dusuruyoruz: yonelim tuketildi. (Yayin motion
        # raporuyla birlikte yapiliyor, cunku sensor_msgs/Imu ivme + acisal
        # hiz + yonelimi tek mesajda tasiyor.)
        self.imu.new_orientation = False
        self._data_pub.publish(msg)
        self._log_accuracy_change()

    def _orientation_variance(self):
        """Yonelim kovaryansi: mumkunse yonganin kendi kestirimi.

        Game rotation vector'de bu alan yok; o modda parametreye dusuyoruz.
        """
        accuracy = self.imu.quat_accuracy_rad
        if self.use_reported_accuracy and accuracy is not None and accuracy > 0.0:
            return accuracy ** 2
        return self.orientation_stddev ** 2

    def _log_accuracy_change(self):
        """Fuzyon dogrulugu degistikce bir kez logla.

        Manyetometre kalibre olana kadar 'unreliable/low' gorunur; kullanici
        yayindaki yonelime ne kadar guvenecegini boylece biliyor. Kalibrasyon
        icin araci havada 8 cizer gibi birkac saniye cevirmek yeterli.
        """
        status = self.imu.quat_status
        if status == self._last_quat_status:
            return
        self._last_quat_status = status
        name = ACCURACY_NAMES.get(status, str(status))
        if status >= 2:
            self.get_logger().info(f'Yonelim dogrulugu: {name}')
        else:
            self.get_logger().warn(
                f'Yonelim dogrulugu: {name} — manyetometre kalibrasyonu icin '
                'araci yavasca birkac eksende cevirin')

    def _publish_mag(self, stamp):
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        # sensor_msgs/MagneticField Tesla ister, BNO08x uT veriyor.
        msg.magnetic_field.x = self.imu.mag[0] * 1e-6
        msg.magnetic_field.y = self.imu.mag[1] * 1e-6
        msg.magnetic_field.z = self.imu.mag[2] * 1e-6
        var = (self.mag_stddev * 1e-6) ** 2
        msg.magnetic_field_covariance = [var, 0.0, 0.0,
                                         0.0, var, 0.0,
                                         0.0, 0.0, var]
        self._mag_pub.publish(msg)

    def destroy_node(self):
        try:
            self.imu.close()
        except OSError:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    try:
        node = ImuNode()
    except (I2CError, ValueError) as error:
        # Kamera/eyleyici zinciri IMU olmadan da calisabilsin diye
        # burada cikiyoruz; launch dosyasinda required degil.
        print(f'[imu_node] baslatilamadi: {error}')
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
