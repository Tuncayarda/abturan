"""BNO08x SHTP surucusu icin donanimsiz testler.

Yonganin yerine sahte bir I2C cihazi koyuyoruz: paket cercevelemesini, Q
noktasi olceklerini ve reset kurtarmasini boylece kart olmadan dogruluyoruz.

    pytest src/imu_bridge/test/test_bno08x.py
"""

import struct

import pytest

from imu_bridge import bno08x
from imu_bridge.bno08x import Bno08x
from imu_bridge.i2c import I2CError

PRODUCT_ID_RESPONSE = (bytes([0xF8, 0x00, 3, 2])
                       + struct.pack('<II', 12345, 678)
                       + struct.pack('<H', 9) + b'\x00\x00')


class FakeI2C:
    """BNO08x'i taklit eder: yazilanlari kaydeder, kuyruktakini okutur."""

    def __init__(self, bus=1, address=0x4A):
        self.bus = bus
        self.address = address
        self.writes = []
        self.packets = []
        self._pending = None
        self.closed = False

    def queue(self, channel, payload, seq=0):
        length = len(payload) + 4
        self.packets.append(bytes([length & 0xFF, (length >> 8) & 0x7F,
                                   channel, seq]) + bytes(payload))

    def write_raw(self, data):
        data = bytes(data)
        self.writes.append(data)
        # Gercek yonga gibi Product ID istegine cevap ver.
        if len(data) >= 5 and data[2] == bno08x.CH_CONTROL and data[4] == 0xF9:
            self.queue(bno08x.CH_CONTROL, PRODUCT_ID_RESPONSE)

    def read_raw(self, n):
        if self._pending is None:
            self._pending = self.packets.pop(0) if self.packets \
                else b'\x00\x00\x00\x00'
        if n == 4:
            header = self._pending[:4]
            if header[0] == 0 and header[1] == 0:
                # Bos baslik: yonga her okumada yeniden verir.
                self._pending = None
            return header
        data = self._pending[:n]
        self._pending = None
        return data + b'\x00' * (n - len(data))

    def close(self):
        self.closed = True


@pytest.fixture
def imu(monkeypatch):
    fake = FakeI2C()
    monkeypatch.setattr(bno08x, 'I2CDevice', lambda bus, address: fake)
    device = Bno08x(soft_reset=False)
    device.fake = fake
    return device


def motion_packet():
    """Tek pakette zaman damgasi + accel + gyro + rotation vector."""
    payload = bytearray()
    payload += bytes([bno08x.REPORT_BASE_TIMESTAMP]) + struct.pack('<I', 1000)
    # Q8: 2510 / 256 = 9.805 m/s^2 (1 g)
    payload += bytes([bno08x.ACCELEROMETER, 0, 3, 0]) + struct.pack('<3h', 0, 0, 2510)
    # Q9: 256 / 512 = 0.5 rad/s
    payload += bytes([bno08x.GYROSCOPE, 0, 3, 0]) + struct.pack('<3h', 256, 0, 0)
    # Q14 birim quaternion + Q12 dogruluk (205 / 4096 = 0.05 rad)
    payload += (bytes([bno08x.ROTATION_VECTOR, 0, 2, 0])
                + struct.pack('<4h', 0, 0, 0, 16384) + struct.pack('<h', 205))
    return bytes(payload)


def test_product_id(imu):
    assert imu.product_id == (3, 2, 9, 12345, 678)
    assert 'SH-2 3.2.9' in imu.version_string
    assert imu.reset_detected is False


def test_set_feature_frame(imu):
    imu.enable_report(bno08x.ROTATION_VECTOR, 10000)
    write = imu.fake.writes[-1]
    assert len(write) == 21                       # 4 bayt basli + 17 bayt yuk
    assert write[2] == bno08x.CH_CONTROL
    assert write[4] == bno08x.CMD_SET_FEATURE
    assert write[5] == bno08x.ROTATION_VECTOR
    assert struct.unpack_from('<I', write, 9)[0] == 10000


def test_multiple_reports_in_one_packet(imu):
    imu.fake.queue(bno08x.CH_INPUT_REPORTS, motion_packet())
    assert imu.service() == 1
    assert imu.accel[2] == pytest.approx(9.8046875)
    assert imu.gyro[0] == pytest.approx(0.5)
    assert imu.quaternion == (0.0, 0.0, 0.0, 1.0)
    assert imu.quat_accuracy_rad == pytest.approx(205 / 4096.0)
    assert imu.accel_status == 3
    assert imu.quat_status == 2
    assert imu.new_motion and imu.new_orientation


def test_magnetometer_scale(imu):
    imu.fake.queue(bno08x.CH_INPUT_REPORTS,
                   bytes([bno08x.MAGNETIC_FIELD, 0, 3, 0])
                   + struct.pack('<3h', 16 * 25, 0, 0))   # Q4 -> 25 uT
    imu.service()
    assert imu.mag[0] == pytest.approx(25.0)
    assert imu.new_mag


def test_game_rotation_vector_has_no_accuracy(imu):
    """Game RV 12 bayt: dogruluk alani yok, eskisini bozmamali."""
    imu.fake.queue(bno08x.CH_INPUT_REPORTS, motion_packet())
    imu.service()
    previous_accuracy = imu.quat_accuracy_rad
    imu.fake.queue(bno08x.CH_INPUT_REPORTS,
                   bytes([bno08x.GAME_ROTATION_VECTOR, 0, 3, 0])
                   + struct.pack('<4h', 0, 0, 8192, 14189))
    imu.service()
    assert imu.quaternion[2] == pytest.approx(0.5, abs=1e-4)
    assert imu.quat_accuracy_rad == previous_accuracy


def test_reset_detection_and_recovery(imu):
    imu.enable_report(bno08x.ACCELEROMETER, 10000)
    imu.enable_report(bno08x.GYROSCOPE, 10000)
    imu.fake.queue(bno08x.CH_EXECUTABLE, bytes([0x01]))
    imu.service()
    assert imu.reset_detected

    before = len(imu.fake.writes)
    imu.reenable_reports()
    assert len(imu.fake.writes) - before == 2
    assert imu.reset_detected is False


def test_unsolicited_product_id_means_reset(imu):
    imu.fake.queue(bno08x.CH_CONTROL, PRODUCT_ID_RESPONSE)
    imu.service()
    assert imu.reset_detected


def test_empty_queue_is_not_an_error(imu):
    assert imu.service() == 0


def test_oversized_packet_is_consumed_in_one_read(imu):
    """MAX_PACKET ustu paket (pratikte acilis reklami) tek okumada yutulmali.

    Kismi okuma ise yaramaz: yongada her I2C okuma islemi paketin basindan
    basliyor, yarim okunan paket kuyruktan dusmuyor. Yutulmazsa surucu ayni
    basligi sonsuza kadar geri okur ve veri akisi hic baslamaz.
    """
    imu.fake.queue(bno08x.CH_COMMAND, b'\xab' * 600)
    imu.fake.queue(bno08x.CH_INPUT_REPORTS,
                   bytes([bno08x.ACCELEROMETER, 0, 3, 0])
                   + struct.pack('<3h', 256, 0, 0))
    imu.service()
    # Buyuk paket atlandi ve ARDINDAKI gercek rapor okunabildi.
    assert imu.accel[0] == pytest.approx(1.0)


def test_bogus_length_raises(imu):
    """Senkron kaybinda (baslik 0xFF 0xFF) sessizce None donmemeli.

    Sessiz None, "veri yok" ile "hat bozuk"u ayirt edilemez hale getirir ve
    node hicbir sey loglamadan sonsuza kadar bos yayin yapar.
    """
    imu.fake.packets.append(b'\xff\xff\xff\xff')
    with pytest.raises(I2CError):
        imu.service()


def test_unknown_and_truncated_reports_are_ignored(imu):
    # Uzunlugunu bilmedigimiz rapor: paketin kalanini ayiramayiz, atlamaliyiz.
    imu.fake.queue(bno08x.CH_INPUT_REPORTS, bytes([0x7F, 0, 0, 0, 1, 2, 3, 4]))
    imu.service()
    # Kirpik rapor: struct.unpack tasmasi olmamali.
    imu.fake.queue(bno08x.CH_INPUT_REPORTS,
                   bytes([bno08x.ACCELEROMETER, 0, 3, 0, 1, 2]))
    imu.service()
    assert imu.accel is None
