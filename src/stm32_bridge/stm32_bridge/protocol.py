#!/usr/bin/env python3
"""
Raspberry Pi <-> STM32 UART cercevesi (wire protocol).

Bu dosya protokolun Python tarafi; C tarafi `stm32_fw/Inc/up_protocol.h` ve
`stm32_fw/Src/up_protocol.c`. IKISI BIRLIKTE DEGISMELI — mesaj id'leri,
payload sirasi ve CRC ayni.

Cerceve:

    +------+------+--------+-----+-----------------+---------+
    | 0xAA | 0x55 | MSG_ID | LEN |  payload (LEN)  | CRC16   |
    +------+------+--------+-----+-----------------+---------+
       0      1       2       3      4 .. 4+LEN-1    2 bayt LE

CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF), MSG_ID + LEN + payload uzerinden.
SOF baytlari CRC'ye girmiyor; senkron kaybinda parser yeni SOF arayarak
kendini toparliyor.

Neden metin degil ikili: 50 Hz'de 6 ESC + step + LED gonderiyoruz ve
115200 baud'da her bayt 87 us. Cerceve 18 bayt (~1.6 ms) — ASCII ile ayni
veri 3-4 kat uzun ve parse'i STM32 tarafinda float parsing gerektiriyor.
"""

import struct

SOF1 = 0xAA
SOF2 = 0x55

HEADER_LEN = 4          # SOF1 SOF2 ID LEN
CRC_LEN = 2
MAX_PAYLOAD = 64
MAX_FRAME = HEADER_LEN + MAX_PAYLOAD + CRC_LEN

# -- Pi -> STM32 ----------------------------------------------------------
MSG_ESC = 0x01          # 6 x uint16 LE, us cinsinden darbe genisligi
MSG_STEPPER = 0x02      # uint8 mode, int16 speed_sps, int32 target
MSG_LED = 0x03          # uint8 0/1
MSG_HEARTBEAT = 0x04    # uint32 seq

# -- STM32 -> Pi ----------------------------------------------------------
MSG_STATUS = 0x81       # asagidaki STATUS_STRUCT
MSG_LOG = 0x82          # ASCII metin

# Step motor modlari (up_protocol.h ile ayni)
STEP_IDLE = 0           # bobinler serbest, isinma yok
STEP_HOLD = 1           # mevcut fazda kilitli bekle
STEP_VELOCITY = 2       # speed_sps hizinda surekli don
STEP_POSITION = 3       # target adimina git, varinca HOLD'a gec

STEP_MODE_NAMES = {
    STEP_IDLE: 'idle',
    STEP_HOLD: 'hold',
    STEP_VELOCITY: 'velocity',
    STEP_POSITION: 'position',
}
STEP_MODE_IDS = {name: mode for mode, name in STEP_MODE_NAMES.items()}

# ESC darbe sinirlari (standart hobi ESC / BlueRobotics Basic ESC)
ESC_MIN_US = 1000
ESC_NEUTRAL_US = 1500
ESC_MAX_US = 2000
ESC_COUNT = 6

# STATUS payload'u: uptime, flags, led, step_pos, step_speed, rx_ok, rx_err
STATUS_STRUCT = struct.Struct('<IBBihHH')
STEPPER_STRUCT = struct.Struct('<Bhi')

# STATUS flags bitleri
FLAG_FAILSAFE = 1 << 0      # komut zaman asimi -> ESC'ler notr, step serbest
FLAG_ESC_ARMED = 1 << 1     # arm sekansi bitti, ESC'ler komut aliyor
FLAG_STEP_ENERGIZED = 1 << 2  # bobinlerde akim var


def crc16(data, crc=0xFFFF):
    """CRC16-CCITT-FALSE. C tarafiyla birebir ayni tabloyu uretir."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode(msg_id, payload=b''):
    """Tek cerceve uret."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload {len(payload)} > {MAX_PAYLOAD}')
    body = bytes([msg_id, len(payload)]) + bytes(payload)
    return bytes([SOF1, SOF2]) + body + struct.pack('<H', crc16(body))


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


# -- Pi -> STM32 kodlayicilar --------------------------------------------

def encode_esc(pulses_us):
    """6 ESC icin darbe genisligi. Eksik verilen kanallar notr kabul edilir."""
    values = list(pulses_us[:ESC_COUNT])
    values += [ESC_NEUTRAL_US] * (ESC_COUNT - len(values))
    clamped = [int(clamp(round(v), ESC_MIN_US, ESC_MAX_US)) for v in values]
    return encode(MSG_ESC, struct.pack('<6H', *clamped))


def encode_stepper(mode, speed_sps=0, target=0):
    """Step motor komutu. speed her modda hiz sinirini belirler."""
    speed = int(clamp(round(speed_sps), -32768, 32767))
    target = int(clamp(round(target), -2**31, 2**31 - 1))
    return encode(MSG_STEPPER,
                  STEPPER_STRUCT.pack(int(mode) & 0xFF, speed, target))


def encode_led(state):
    return encode(MSG_LED, struct.pack('<B', 1 if state else 0))


def encode_heartbeat(seq):
    return encode(MSG_HEARTBEAT, struct.pack('<I', int(seq) & 0xFFFFFFFF))


# -- STM32 -> Pi kodlayicilar --------------------------------------------
# Uretimde bu yonu STM32 uretiyor; buradaki kodlayicilar sanal STM32
# (tools/fake_stm32.py) ve testler icin var — C tarafindaki
# up_encode_status / up_encode_log ile ayni cerceveyi verir.

def encode_status(uptime_ms, flags=0, led=False, stepper_position=0,
                  stepper_speed_sps=0, rx_ok=0, rx_err=0):
    payload = STATUS_STRUCT.pack(
        int(uptime_ms) & 0xFFFFFFFF,
        int(flags) & 0xFF,
        1 if led else 0,
        int(stepper_position),
        int(stepper_speed_sps),
        int(rx_ok) & 0xFFFF,
        int(rx_err) & 0xFFFF,
    )
    return encode(MSG_STATUS, payload)


def encode_log(text):
    data = str(text).encode('ascii', errors='replace')[:MAX_PAYLOAD]
    return encode(MSG_LOG, data)


# -- STM32 -> Pi cozuculer ----------------------------------------------

def decode_status(payload):
    """STATUS payload'unu dict'e cevir."""
    if len(payload) < STATUS_STRUCT.size:
        raise ValueError(f'status payload {len(payload)} < {STATUS_STRUCT.size}')
    (uptime_ms, flags, led, step_pos, step_speed,
     rx_ok, rx_err) = STATUS_STRUCT.unpack_from(payload)
    return {
        'uptime_ms': uptime_ms,
        'failsafe': bool(flags & FLAG_FAILSAFE),
        'esc_armed': bool(flags & FLAG_ESC_ARMED),
        'stepper_energized': bool(flags & FLAG_STEP_ENERGIZED),
        'led': bool(led),
        'stepper_position': step_pos,
        'stepper_speed_sps': step_speed,
        'rx_ok': rx_ok,
        'rx_err': rx_err,
    }


class FrameParser:
    """Akis halindeki bayttan cerceve cikaran durum makinesi.

    Seri porttan gelen veri parca parca dusuyor; her cagrida elde ne
    varsa isliyoruz ve tamamlanan cerceveleri donuyoruz. Bozuk CRC veya
    kacan bayt durumunda tampondan tek bayt atip yeni SOF ariyoruz —
    boylece hat bir kez bozulsa bile kalici kilitlenme olmuyor.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.ok_count = 0
        self.error_count = 0

    def feed(self, data):
        """Gelen baytlari isle, (msg_id, payload) listesi don."""
        self.buffer.extend(data)
        frames = []

        while True:
            # SOF'a kadar ilerle
            start = self.buffer.find(bytes([SOF1, SOF2]))
            if start < 0:
                # Yarim kalmis SOF1'i sakla, gerisini at
                keep = 1 if self.buffer[-1:] == bytes([SOF1]) else 0
                if len(self.buffer) > keep:
                    self.buffer = self.buffer[len(self.buffer) - keep:]
                break
            if start > 0:
                self.error_count += 1
                del self.buffer[:start]

            if len(self.buffer) < HEADER_LEN:
                break

            msg_id = self.buffer[2]
            length = self.buffer[3]
            if length > MAX_PAYLOAD:
                self.error_count += 1
                del self.buffer[:2]
                continue

            total = HEADER_LEN + length + CRC_LEN
            if len(self.buffer) < total:
                break

            body = bytes(self.buffer[2:HEADER_LEN + length])
            received = struct.unpack_from('<H', self.buffer, HEADER_LEN + length)[0]
            if received == crc16(body):
                frames.append((msg_id, bytes(self.buffer[HEADER_LEN:HEADER_LEN + length])))
                self.ok_count += 1
                del self.buffer[:total]
            else:
                # CRC tutmadi: bu SOF muhtemelen veri icindeki rastlanti.
                self.error_count += 1
                del self.buffer[:2]

        return frames
