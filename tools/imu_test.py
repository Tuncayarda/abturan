#!/usr/bin/env python3
"""
IMU'yu (BNO08x) ROS'suz oku.

I2C kablolamasinin ve adresin dogru oldugunu gormek icin. Ayni surucuyu
(src/imu_bridge/.../bno08x.py) kullanir.

Kullanim:
    python3 tools/imu_test.py
    python3 tools/imu_test.py --address 0x4B
    python3 tools/imu_test.py --mode game_rotation_vector
    python3 tools/imu_test.py --scan
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'imu_bridge'))

from imu_bridge.bno08x import (ACCELEROMETER, ACCURACY_NAMES,  # noqa: E402
                               Bno08x, GYROSCOPE, MAGNETIC_FIELD,
                               ROTATION_REPORTS)
from imu_bridge.i2c import I2CDevice, I2CError  # noqa: E402


def scan(bus):
    """Hattaki cihazlari tara (i2cdetect'in kucuk hali)."""
    print(f'i2c-{bus} taraniyor...')
    found = []
    for addr in range(0x03, 0x78):
        try:
            dev = I2CDevice(bus, addr)
        except I2CError as error:
            print(f'  {error}')
            return []
        try:
            dev.read_raw(1)
            found.append(addr)
        except I2CError:
            pass
        finally:
            dev.close()
    if found:
        print('  bulunan adresler: ' + ', '.join(f'0x{a:02X}' for a in found))
        for addr in found:
            if addr in (0x4A, 0x4B):
                print(f'  0x{addr:02X} BNO08x olabilir (--address 0x{addr:02X})')
    else:
        print('  hicbir cihaz yok. SDA/SCL/VIN/GND ve 3.3 V beslemeyi kontrol et.')
    return found


def quat_to_euler_deg(x, y, z, w):
    """(x, y, z, w) -> (roll, pitch, yaw) derece. Sadece gozle dogrulama icin."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    # Tepe noktalarinda asin tanim disina cikmasin.
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    parser = argparse.ArgumentParser(description='BNO08x ham okuma testi')
    parser.add_argument('--bus', type=int, default=1,
                        help='I2C veri yolu (GPIO2/3 = 1)')
    parser.add_argument('--address', default='0x4A',
                        help='BNO08x adresi (ADR=GND -> 0x4A, ADR=VCC -> 0x4B)')
    parser.add_argument('--mode', default='rotation_vector',
                        choices=sorted(ROTATION_REPORTS),
                        help='yonelim kaynagi')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='ekrana yazma frekansi (Hz)')
    parser.add_argument('--report-hz', type=float, default=100.0,
                        help='yonganin rapor akis frekansi')
    parser.add_argument('--mag', action='store_true',
                        help='manyetometreyi de ac ve goster')
    parser.add_argument('--no-reset', action='store_true',
                        help='aciliste yumusak reset atma')
    parser.add_argument('--scan', action='store_true',
                        help='sadece hatti tara ve cik')
    args = parser.parse_args()

    if not os.path.exists(f'/dev/i2c-{args.bus}'):
        print(f'/dev/i2c-{args.bus} yok. config.txt icinde "dtparam=i2c_arm=on" '
              'gerekiyor — ./setup_hardware.sh calistir, sonra reboot.')
        return 1

    if args.scan:
        scan(args.bus)
        return 0

    address = int(args.address, 0)
    try:
        imu = Bno08x(bus=args.bus, address=address, soft_reset=not args.no_reset)
    except I2CError as error:
        print(error)
        scan(args.bus)
        return 1

    print(f'BNO08x: {imu.version_string}  i2c-{args.bus} 0x{address:02X}')

    interval_us = int(1e6 / args.report_hz)
    imu.enable_report(ACCELEROMETER, interval_us)
    imu.enable_report(GYROSCOPE, interval_us)
    imu.enable_report(ROTATION_REPORTS[args.mode], interval_us)
    if args.mag:
        imu.enable_report(MAGNETIC_FIELD, max(interval_us, 20000))
    print(f'raporlar acildi: {args.report_hz:.0f} Hz, yonelim = {args.mode}')

    print('\nCikis: Ctrl+C')
    header = (f'{"ax":>8}{"ay":>8}{"az":>8} | {"gx":>8}{"gy":>8}{"gz":>8} | '
              f'{"roll":>7}{"pitch":>7}{"yaw":>7} | {"dogruluk":>9}')
    if args.mag:
        header += f' | {"mx":>7}{"my":>7}{"mz":>7}'
    print(header)

    period = 1.0 / args.rate
    next_print = time.monotonic()
    errors = 0
    try:
        while True:
            try:
                imu.service()
            except I2CError as error:
                errors += 1
                if errors % 50 == 1:
                    print(f'\nI2C hatasi ({errors}): {error}')
            if imu.reset_detected:
                print('\nyonga resetlendi, raporlar geri aciliyor')
                imu.reenable_reports()

            now = time.monotonic()
            if now < next_print:
                time.sleep(0.002)
                continue
            next_print = now + period

            if imu.accel is None or imu.gyro is None:
                print('veri bekleniyor...', end='\r')
                continue

            ax, ay, az = imu.accel
            norm = math.sqrt(ax * ax + ay * ay + az * az)
            if imu.quaternion is None:
                roll = pitch = yaw = float('nan')
            else:
                roll, pitch, yaw = quat_to_euler_deg(*imu.quaternion)
            accuracy = ACCURACY_NAMES.get(imu.quat_status, '?')

            line = (f'{ax:8.2f}{ay:8.2f}{az:8.2f} | '
                    f'{imu.gyro[0]:8.3f}{imu.gyro[1]:8.3f}{imu.gyro[2]:8.3f} | '
                    f'{roll:7.1f}{pitch:7.1f}{yaw:7.1f} | {accuracy:>9}')
            if args.mag and imu.mag is not None:
                line += (f' | {imu.mag[0]:7.1f}{imu.mag[1]:7.1f}'
                         f'{imu.mag[2]:7.1f}')
            print(f'{line}   |a|={norm:5.2f}', end='\r')
    except KeyboardInterrupt:
        print('\n')
        print('Kontrol: durgunken |a| ~ 9.81 olmali, gyro degerleri ~0.')
        print('Yaw sabit durmuyorsa veya dogruluk "low/unreliable" ise')
        print('araci havada 8 cizer gibi birkac saniye cevirin (manyetometre')
        print('kalibrasyonu), ya da --mode game_rotation_vector ile deneyin.')
    finally:
        imu.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
