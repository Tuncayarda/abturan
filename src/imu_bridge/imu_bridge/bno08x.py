#!/usr/bin/env python3
"""
BNO08x (BNO085 / BNO086) surucusu — I2C uzerinden SHTP + SH-2.

MPU ailesinden temel farki: bu yonga **register haritasi sunmuyor**. Uzerinde
kendi ARM cekirdegi ve fuzyon yazilimi (SH-2) var; haberlesme paket tabanli:

    SHTP paketi = 4 bayt basli + yuk
        bayt 0  uzunluk LSB
        bayt 1  uzunluk MSB (bit 15 = devam bayragi, maskeliyoruz)
        bayt 2  kanal
        bayt 3  sira numarasi
    (uzunluk 4 baytlik basligi da kapsar; 0 gelirse "soyleyecek sey yok")

Akis: aciliste yongaya hangi raporu kac mikrosaniyede bir istedigimizi
soyluyoruz (Set Feature Command), sonrasinda veri kendiliginden akiyor ve biz
sadece paket kuyrugunu bosaltiyoruz.

Kazanc: yonelim fuzyonu yongada. Rotation vector dogrudan quaternion veriyor,
manyetometre de isin icinde oldugu icin **yaw kaymiyor** — MPU6050 kurulumunda
tumleyici filtreyle yasadigimiz sorun ortadan kalkiyor.

Kablolama (MPU ile ayni hat):
    Pi GPIO2 / SDA  (fiziksel pin 3)  -> BNO SDA
    Pi GPIO3 / SCL  (fiziksel pin 5)  -> BNO SCL
    Pi 3.3 V        (fiziksel pin 1)  -> BNO VIN (3V3)
    Pi GND          (fiziksel pin 9)  -> BNO GND

Adres ADR/PS0 pini dusukken 0x4A, yuksekken 0x4B.

DIKKAT — saat germe (clock stretching): BNO08x I2C'de saati uzun sure
gerebiliyor, Raspberry Pi'nin donanim I2C'si (BSC) bunu bilinen sekilde kotu
yonetiyor. 400 kHz'te bozuk paket/EREMOTEIO gorulurse hiz dusurulmeli:
/boot/firmware/config.txt icinde `dtparam=i2c_arm_baudrate=50000`.
"""

import struct
import time

from imu_bridge.i2c import I2CDevice, I2CError

# --- SHTP kanallari ---
CH_COMMAND = 0          # SHTP'nin kendi komutlari + acilis reklam paketi
CH_EXECUTABLE = 1       # reset / uyku
CH_CONTROL = 2          # SH-2 kontrol (feature ac/kapa, sorgular)
CH_INPUT_REPORTS = 3    # sensor raporlari
CH_WAKE_REPORTS = 4
CH_GYRO_RV = 5

# --- SH-2 kontrol raporlari ---
CMD_PRODUCT_ID_REQUEST = 0xF9
RSP_PRODUCT_ID = 0xF8
CMD_SET_FEATURE = 0xFD
RSP_GET_FEATURE = 0xFC
RSP_COMMAND = 0xF1
REPORT_BASE_TIMESTAMP = 0xFB
REPORT_TIMESTAMP_REBASE = 0xFA

# --- Sensor rapor kimlikleri ---
ACCELEROMETER = 0x01
GYROSCOPE = 0x02
MAGNETIC_FIELD = 0x03
LINEAR_ACCELERATION = 0x04
ROTATION_VECTOR = 0x05
GRAVITY = 0x06
GAME_ROTATION_VECTOR = 0x08
GEOMAGNETIC_ROTATION_VECTOR = 0x09

# Rapor uzunluklari (4 baytlik rapor onekiyle birlikte). Tek SHTP paketinde
# arka arkaya birden fazla rapor gelebiliyor; ayirmak icin bunlar sart.
REPORT_LENGTHS = {
    REPORT_BASE_TIMESTAMP: 5,
    REPORT_TIMESTAMP_REBASE: 5,
    ACCELEROMETER: 10,
    GYROSCOPE: 10,
    MAGNETIC_FIELD: 10,
    LINEAR_ACCELERATION: 10,
    ROTATION_VECTOR: 14,
    GRAVITY: 10,
    GAME_ROTATION_VECTOR: 12,
    GEOMAGNETIC_ROTATION_VECTOR: 14,
}

# Sabit noktali cikislarin Q noktalari: gercek deger = ham * 2^-Q
Q_ACCEL = 8       # m/s^2
Q_GYRO = 9        # rad/s
Q_MAG = 4         # uT
Q_QUAT = 14       # birimsiz
Q_ACCURACY = 12   # rad

SCALE_ACCEL = 2.0 ** -Q_ACCEL
SCALE_GYRO = 2.0 ** -Q_GYRO
SCALE_MAG = 2.0 ** -Q_MAG
SCALE_QUAT = 2.0 ** -Q_QUAT
SCALE_ACCURACY = 2.0 ** -Q_ACCURACY

# Acilis reklam paketi ~270-380 bayt; 512 rahat yetiyor. Isimize yarayan
# paketler (sensor raporlari, kontrol cevaplari) zaten 64 baytin altinda.
MAX_PACKET = 512

# Bunun uzerindeki "uzunluk" gercek bir paket degil, senkron kaybi demek.
# (i2c-dev tek okumada 8192 bayttan fazlasini kirpiyor.)
MAX_DISCARD = 8192

# Tek okuma cagrisinda en fazla bu kadar buyuk paket atlanir; sonrasi hata.
MAX_SKIPS = 4

# _read_one'in "bu paketi attim, sen devam et" isareti. None (= veri yok) ile
# karismasin diye ayri bir nesne.
_SKIPPED = object()

ROTATION_REPORTS = {
    'rotation_vector': ROTATION_VECTOR,
    'game_rotation_vector': GAME_ROTATION_VECTOR,
    'geomagnetic_rotation_vector': GEOMAGNETIC_ROTATION_VECTOR,
}

ACCURACY_NAMES = {0: 'unreliable', 1: 'low', 2: 'medium', 3: 'high'}


class Bno08x:
    """BNO08x SHTP surucusu. Bloke etmez: service() kuyrugu bosaltir."""

    def __init__(self, bus=1, address=0x4A, soft_reset=True):
        self.dev = I2CDevice(bus, address)
        self._seq = [0] * 6          # kanal basina giden paket sira numarasi
        self._enabled = {}           # report_id -> araligi (us), reset sonrasi geri kurmak icin

        # Son gorulen degerler. None = o rapordan henuz veri gelmedi.
        self.accel = None            # (x, y, z) m/s^2
        self.gyro = None             # (x, y, z) rad/s
        self.mag = None              # (x, y, z) uT
        self.quaternion = None       # (x, y, z, w)
        self.quat_accuracy_rad = None
        self.accel_status = 0        # 0..3 (SH-2 dogruluk seviyesi)
        self.gyro_status = 0
        self.mag_status = 0
        self.quat_status = 0

        # Yeni veri bayraklari — node bunlara bakip yayin yapiyor.
        self.new_motion = False      # accel veya gyro guncellendi
        self.new_mag = False
        self.new_orientation = False

        self.reset_detected = False  # yonga kendini resetlediyse True
        self.product_id = None       # (sw_major, sw_minor, sw_patch, part_no, build)

        if soft_reset:
            self.soft_reset()
        else:
            self._drain(0.2)

        self.product_id = self.read_product_id()
        # Acilista yutulan paketler arasinda "reset tamamlandi" bildirimi de
        # olabilir; bu bizim kendi resetimiz, node'a yeni bir reset gibi
        # gostermeyelim.
        self.reset_detected = False

    # ------------------------------------------------------------------
    # SHTP tasima katmani
    def _write_packet(self, channel, payload):
        length = len(payload) + 4
        header = bytes([length & 0xFF, (length >> 8) & 0x7F,
                        channel, self._seq[channel]])
        self._seq[channel] = (self._seq[channel] + 1) & 0xFF
        self.dev.write_raw(header + bytes(payload))

    def _read_packet(self):
        """Isimize yarayan ilk SHTP paketini oku. Veri yoksa None doner.

        Ilgilenmedigimiz buyuk paketleri (acilis reklami) burada yutup devam
        ediyoruz: "atladim" ile "veri yok"u ayni sekilde donmek, cagiran
        taraflarin (service/_drain/_wait_for) kuyruk bosaldi sanip bir tur
        beklemesine yol aciyordu.
        """
        for _ in range(MAX_SKIPS):
            packet = self._read_one()
            if packet is not _SKIPPED:
                return packet
        # Ust uste bu kadar atlanacak paket normal degil.
        raise I2CError('SHTP kuyrugu bosaltilamadi: surekli buyuk paket geliyor')

    def _read_one(self):
        """Tek bir okuma denemesi. Atlanan paket icin _SKIPPED doner."""
        header = self.dev.read_raw(4)
        length = ((header[1] << 8) | header[0]) & 0x7FFF
        if length == 0:
            return None
        if length < 4:
            # Bozuk basligi paket sanip surunmeyelim.
            raise I2CError(f'gecersiz SHTP uzunlugu: {length}')

        if length > MAX_PACKET:
            # Ilgilenmedigimiz kadar buyuk paket (pratikte sadece acilis reklam
            # paketi). KISMI OKUMA ISE YARAMAZ: BNO08x'te her I2C okuma islemi
            # bekleyen paketin BASINDAN basliyor, yani yarim okunan paket
            # kuyruktan dusmuyor ve ayni basligi sonsuza kadar geri okurduk.
            # Tek transaction'da tamamini okuyup atiyoruz.
            if length > MAX_DISCARD:
                # i2c-dev tek okumada 8 KB'dan fazlasini kirpiyor; bu boyda bir
                # "paket" gercek degil, hat senkronu kaybolmus (tipik olarak
                # baslik 0xFF 0xFF okunmus). Sessizce None donmek veri akisinin
                # durdugunu gizler, o yuzden hata veriyoruz.
                raise I2CError(
                    f'SHTP senkronu kayip gorunuyor (uzunluk {length}). '
                    'I2C hizini dusurmeyi deneyin: '
                    'dtparam=i2c_arm_baudrate=50000')
            self.dev.read_raw(length)
            return _SKIPPED

        # Not: ikinci okuma basligi bastan verir, uzunluk onu da kapsiyor.
        data = self.dev.read_raw(length)
        return data[2], data[4:]

    def _drain(self, seconds):
        """Verilen sure boyunca gelen paketleri oku (acilis gurultusu icin)."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                packet = self._read_packet()
            except I2CError:
                packet = None
            if packet is None:
                time.sleep(0.005)
                continue
            self._handle_packet(*packet)

    def _wait_for(self, channel, report_id, timeout=1.0):
        """Belirli bir kontrol cevabini bekle; gelen digerlerini isle."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                packet = self._read_packet()
            except I2CError:
                packet = None
            if packet is None:
                time.sleep(0.002)
                continue
            ch, payload = packet
            if ch == channel and payload and payload[0] == report_id:
                return payload
            self._handle_packet(ch, payload)
        return None

    # ------------------------------------------------------------------
    # SH-2 komutlari
    def soft_reset(self):
        """Yumusak reset. Bilinen bir durumdan baslamak icin.

        Reset sonrasi yonga once SHTP reklam paketini, sonra 'reset tamam'
        bildirimini yolluyor; ikisini de yutuyoruz.
        """
        try:
            self._write_packet(CH_EXECUTABLE, [0x01])
        except I2CError:
            # Reset komutunun ACK'ini yonga bazen kesiyor; onemli degil.
            pass
        time.sleep(0.6)
        self._seq = [0] * 6
        self._drain(0.5)
        self.reset_detected = False

    def read_product_id(self):
        self._write_packet(CH_CONTROL, [CMD_PRODUCT_ID_REQUEST, 0x00])
        payload = self._wait_for(CH_CONTROL, RSP_PRODUCT_ID, timeout=1.0)
        if payload is None or len(payload) < 16:
            raise I2CError(
                'BNO08x cevap vermedi (Product ID). Adres dogru mu (0x4A/0x4B), '
                'kablolama ve 3V3 besleme tamam mi? I2C hizi 400 kHz ise '
                'dtparam=i2c_arm_baudrate=50000 ile dusurmeyi deneyin.')
        sw_major = payload[2]
        sw_minor = payload[3]
        part_no, build = struct.unpack_from('<II', payload, 4)
        sw_patch, = struct.unpack_from('<H', payload, 12)
        return (sw_major, sw_minor, sw_patch, part_no, build)

    def enable_report(self, report_id, interval_us):
        """Set Feature Command: raporu belirtilen periyotla akit."""
        payload = bytearray(17)
        payload[0] = CMD_SET_FEATURE
        payload[1] = report_id
        # [2] ozellik bayraklari, [3:5] degisim duyarliligi -> 0 (periyodik akis)
        struct.pack_into('<I', payload, 5, int(interval_us))
        # [9:13] toplu (batch) araligi, [13:17] sensore ozel yapilandirma -> 0
        self._write_packet(CH_CONTROL, payload)
        self._enabled[report_id] = int(interval_us)
        time.sleep(0.01)

    def reenable_reports(self):
        """Reset sonrasi daha once acilmis raporlari geri ac."""
        for report_id, interval_us in list(self._enabled.items()):
            self.enable_report(report_id, interval_us)
        self.reset_detected = False

    # ------------------------------------------------------------------
    # Paket isleme
    def service(self, max_packets=32):
        """Bekleyen paketleri isle. Islenen paket sayisini doner.

        I2C hatasini yukari birakir; cagiran taraf seyrek loglayabilsin diye.
        """
        count = 0
        while count < max_packets:
            packet = self._read_packet()
            if packet is None:
                break
            self._handle_packet(*packet)
            count += 1
        return count

    def _handle_packet(self, channel, payload):
        if not payload:
            return
        if channel in (CH_INPUT_REPORTS, CH_WAKE_REPORTS):
            self._parse_reports(payload)
        elif channel == CH_EXECUTABLE:
            if payload[0] == 0x01:      # "reset tamamlandi"
                self.reset_detected = True
        elif channel == CH_CONTROL:
            if payload[0] == RSP_PRODUCT_ID:
                # Istemeden gelen Product ID = yonga resetlenmis demek.
                self.reset_detected = True
        # CH_COMMAND (reklam paketi) ve digerleri bizi ilgilendirmiyor.

    def _parse_reports(self, payload):
        offset = 0
        end = len(payload)
        while offset < end:
            report_id = payload[offset]
            size = REPORT_LENGTHS.get(report_id)
            if size is None:
                # Tanimadigimiz raporun uzunlugunu bilemeyiz, dolayisiyla
                # paketin kalanini da ayiramayiz — burada kesiyoruz.
                return
            if offset + size > end:
                return
            self._parse_one(report_id, payload, offset)
            offset += size

    def _parse_one(self, report_id, payload, offset):
        if report_id in (REPORT_BASE_TIMESTAMP, REPORT_TIMESTAMP_REBASE):
            return

        # Sensor raporlarinin ortak oneki: [id, sira, durum, gecikme]
        status = payload[offset + 2] & 0x03

        if report_id == ACCELEROMETER:
            x, y, z = struct.unpack_from('<3h', payload, offset + 4)
            self.accel = (x * SCALE_ACCEL, y * SCALE_ACCEL, z * SCALE_ACCEL)
            self.accel_status = status
            self.new_motion = True
        elif report_id == GYROSCOPE:
            x, y, z = struct.unpack_from('<3h', payload, offset + 4)
            self.gyro = (x * SCALE_GYRO, y * SCALE_GYRO, z * SCALE_GYRO)
            self.gyro_status = status
            self.new_motion = True
        elif report_id == MAGNETIC_FIELD:
            x, y, z = struct.unpack_from('<3h', payload, offset + 4)
            self.mag = (x * SCALE_MAG, y * SCALE_MAG, z * SCALE_MAG)
            self.mag_status = status
            self.new_mag = True
        elif report_id in (ROTATION_VECTOR, GAME_ROTATION_VECTOR,
                           GEOMAGNETIC_ROTATION_VECTOR):
            i, j, k, real = struct.unpack_from('<4h', payload, offset + 4)
            self.quaternion = (i * SCALE_QUAT, j * SCALE_QUAT,
                               k * SCALE_QUAT, real * SCALE_QUAT)
            if report_id != GAME_ROTATION_VECTOR:
                # Game RV'de dogruluk kestirimi alani yok.
                accuracy, = struct.unpack_from('<h', payload, offset + 12)
                self.quat_accuracy_rad = accuracy * SCALE_ACCURACY
            self.quat_status = status
            self.new_orientation = True

    # ------------------------------------------------------------------
    def close(self):
        self.dev.close()

    @property
    def version_string(self):
        if self.product_id is None:
            return 'bilinmiyor'
        major, minor, patch, part_no, build = self.product_id
        return f'SH-2 {major}.{minor}.{patch} (part {part_no}, build {build})'
