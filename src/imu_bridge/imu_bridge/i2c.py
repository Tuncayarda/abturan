#!/usr/bin/env python3
"""
Cok kucuk bir I2C erisim katmani (/dev/i2c-N uzerinden, ioctl ile).

Neden smbus2/python3-smbus yok: ROS konteynerine ek paket kurmamak icin.
Ihtiyacimiz olan tek sey "register yaz / register oku" ve bunu Linux i2c-dev
arayuzu zaten dosya islemleriyle veriyor:

    ioctl(fd, I2C_SLAVE, addr)   -> hedef cihazi sec
    write(fd, [reg])             -> okunacak register'i isaretle
    read(fd, n)                  -> n bayt oku

Iki kullanim sekli var:

* register tabanli cihazlar (MPU ailesi): write_byte / read_block
* SHTP gibi paket tabanli cihazlar (BNO08x): write_raw / read_raw — arada
  register bayti yok, ham bayt dizisi gidip geliyor.

Not: bu yontem yazma ile okuma arasinda STOP birakiyor (repeated start yok).
Hem MPU ailesi hem BNO08x bunu sorunsuz kabul ediyor.
"""

import fcntl
import os

I2C_SLAVE = 0x0703


class I2CError(OSError):
    """I2C hattinda okuma/yazma hatasi."""


class I2CDevice:

    def __init__(self, bus=1, address=0x4A):
        self.bus = int(bus)
        self.address = int(address)
        self.path = f'/dev/i2c-{self.bus}'
        try:
            self._fd = os.open(self.path, os.O_RDWR)
            fcntl.ioctl(self._fd, I2C_SLAVE, self.address)
        except OSError as error:
            raise I2CError(
                f'{self.path} adres 0x{self.address:02X} acilamadi: {error}. '
                'I2C acik mi? /boot/firmware/config.txt icinde '
                'Pi 5 config.txt icinde "dtoverlay=i2c1-pi5,pins_2_3" '
                'olmali (setup_hardware.sh bunu yapar).'
            ) from error

    # ------------------------------------------------------------------
    # Ham erisim (BNO08x/SHTP): register bayti yok.
    def write_raw(self, data):
        try:
            written = os.write(self._fd, bytes(data))
        except OSError as error:
            raise I2CError(f'{len(data)} bayt yazilamadi: {error}') from error
        if written != len(data):
            raise I2CError(f'{len(data)} bayt istendi, {written} bayt yazildi')

    def read_raw(self, length):
        try:
            data = os.read(self._fd, length)
        except OSError as error:
            raise I2CError(f'{length} bayt okunamadi: {error}') from error
        if len(data) != length:
            raise I2CError(f'{length} bayt istendi, {len(data)} geldi')
        return data

    # ------------------------------------------------------------------
    # Register tabanli erisim (MPU ailesi).
    def write_byte(self, reg, value):
        try:
            os.write(self._fd, bytes([reg & 0xFF, value & 0xFF]))
        except OSError as error:
            raise I2CError(f'0x{reg:02X} register yazilamadi: {error}') from error

    def read_block(self, reg, length):
        try:
            os.write(self._fd, bytes([reg & 0xFF]))
            data = os.read(self._fd, length)
        except OSError as error:
            raise I2CError(f'0x{reg:02X} register okunamadi: {error}') from error
        if len(data) != length:
            raise I2CError(f'0x{reg:02X}: {length} bayt istendi, {len(data)} geldi')
        return data

    def read_byte(self, reg):
        return self.read_block(reg, 1)[0]

    def close(self):
        if getattr(self, '_fd', None) is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
