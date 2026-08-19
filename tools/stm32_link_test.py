#!/usr/bin/env python3
"""
STM32 hattini ROS'suz test et.

ROS/Docker devreye girmeden once kablonun ve firmware'in dogru oldugunu
gormek icin. Ayni protokolu (src/stm32_bridge/.../protocol.py) kullanir,
yani burada calisan sey koprude de calisir.

Arka planda 50 Hz ESC cercevesi + heartbeat gonderiyor — STM32'nin
failsafe'i tam da bunun kesilmesine bakiyor, o yuzden gercek kosullari
taklit etmek icin sart.

Kullanim:
    python3 tools/stm32_link_test.py                     # /dev/ttyAMA0
    python3 tools/stm32_link_test.py --port /dev/ttyUSB0 --baud 115200

Komutlar (interaktif):
    esc <1-6> <1000-2000>   tek kanala darbe ver
    esc all <1000-2000>     hepsine ayni darbe
    stop                    hepsini notre (1500) al
    step v <adim/s>         surekli don (isaretli, 0 = dur+kilitle)
    step p <adim>           mutlak hedefe git
    step hold               bobinleri kilitli tut
    step idle               bobinleri birak (isinma yok)
    led on | led off
    status                  son STATUS cercevesini yaz
    mon [saniye]            STATUS akisini izle (varsayilan 5 s)
    q                       cik (notre alip bobinleri birakir)
"""

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'stm32_bridge'))

from stm32_bridge import protocol as proto  # noqa: E402

try:
    import serial
except ImportError:
    sys.exit('pyserial yok: sudo apt install -y python3-serial')


class Link:
    """Seri hat + arka plan gonderici/okuyucu."""

    def __init__(self, port, baud, rate_hz=50.0):
        self.serial = serial.Serial(port, baud, timeout=0.1, write_timeout=0.2)
        self.rate_hz = rate_hz
        self.parser = proto.FrameParser()
        self.lock = threading.Lock()
        self.running = True

        self.esc_us = [proto.ESC_NEUTRAL_US] * proto.ESC_COUNT
        self.status = None
        self.status_stamp = 0.0
        self.logs = []
        self.seq = 0

        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread.start()

    def send(self, frame):
        with self.lock:
            try:
                self.serial.write(frame)
            except Exception as error:  # noqa: B902
                print(f'  ! yazma hatasi: {error}')

    def _tx_loop(self):
        period = 1.0 / self.rate_hz
        deadline = time.perf_counter()
        while self.running:
            self.send(proto.encode_esc(self.esc_us))
            self.seq += 1
            if self.seq % 10 == 0:
                self.send(proto.encode_heartbeat(self.seq))
            deadline += period
            slack = deadline - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                deadline = time.perf_counter()

    def _rx_loop(self):
        while self.running:
            try:
                chunk = self.serial.read(1)
                if self.serial.in_waiting:
                    chunk += self.serial.read(self.serial.in_waiting)
            except Exception:  # noqa: B902
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            for msg_id, payload in self.parser.feed(chunk):
                if msg_id == proto.MSG_STATUS:
                    try:
                        self.status = proto.decode_status(payload)
                        self.status_stamp = time.monotonic()
                    except ValueError:
                        pass
                elif msg_id == proto.MSG_LOG:
                    text = payload.decode('ascii', errors='replace').strip()
                    self.logs.append(text)
                    print(f'\n  [stm32] {text}')

    def close(self):
        self.esc_us = [proto.ESC_NEUTRAL_US] * proto.ESC_COUNT
        self.send(proto.encode_esc(self.esc_us))
        self.send(proto.encode_stepper(proto.STEP_IDLE, 0, 0))
        self.send(proto.encode_led(False))
        time.sleep(0.1)
        self.running = False
        time.sleep(0.2)
        self.serial.close()


def print_status(link):
    if link.status is None:
        print('  STATUS gelmedi. RX kablosu (STM32 PA2 -> Pi GPIO15/pin 10) '
              've baud hizini kontrol et.')
        return
    age = time.monotonic() - link.status_stamp
    s = link.status
    print(f'  yas={age * 1000:.0f} ms  uptime={s["uptime_ms"] / 1000:.1f} s')
    print(f'  failsafe={s["failsafe"]}  esc_armed={s["esc_armed"]}  '
          f'led={s["led"]}  bobin_enerjili={s["stepper_energized"]}')
    print(f'  step: konum={s["stepper_position"]} hiz={s["stepper_speed_sps"]} adim/s')
    print(f'  cerceve: ok={s["rx_ok"]} hata={s["rx_err"]}')


def handle(link, text):
    """Tek komut isle. False donerse cikiliyor."""
    parts = text.split()
    if not parts:
        return True
    cmd = parts[0].lower()

    if cmd in ('q', 'quit', 'exit'):
        return False

    if cmd == 'stop':
        link.esc_us = [proto.ESC_NEUTRAL_US] * proto.ESC_COUNT
        print('  hepsi notr (1500 us)')
        return True

    if cmd == 'esc':
        if len(parts) != 3:
            print('  kullanim: esc <1-6|all> <1000-2000>')
            return True
        try:
            us = int(parts[2])
        except ValueError:
            print('  darbe sayi olmali')
            return True
        if not proto.ESC_MIN_US <= us <= proto.ESC_MAX_US:
            print(f'  darbe {proto.ESC_MIN_US}-{proto.ESC_MAX_US} arasinda olmali')
            return True
        if parts[1].lower() == 'all':
            link.esc_us = [us] * proto.ESC_COUNT
            print(f'  ESC1-6 = {us} us')
        else:
            try:
                ch = int(parts[1])
            except ValueError:
                print('  kanal 1-6 veya "all"')
                return True
            if not 1 <= ch <= proto.ESC_COUNT:
                print(f'  kanal 1-{proto.ESC_COUNT} olmali')
                return True
            link.esc_us[ch - 1] = us
            print(f'  ESC{ch} = {us} us')
        return True

    if cmd == 'step':
        if len(parts) < 2:
            print('  kullanim: step v <adim/s> | step p <adim> | step hold | step idle')
            return True
        sub = parts[1].lower()
        if sub in ('hold', 'idle'):
            mode = proto.STEP_HOLD if sub == 'hold' else proto.STEP_IDLE
            link.send(proto.encode_stepper(mode, 0, 0))
            print(f'  step {sub}')
            return True
        if len(parts) != 3:
            print('  kullanim: step v <adim/s> | step p <adim>')
            return True
        try:
            value = int(parts[2])
        except ValueError:
            print('  sayi olmali')
            return True
        if sub == 'v':
            mode = proto.STEP_VELOCITY if value else proto.STEP_HOLD
            link.send(proto.encode_stepper(mode, value, 0))
            print(f'  step hiz = {value} adim/s')
        elif sub == 'p':
            link.send(proto.encode_stepper(proto.STEP_POSITION, 400, value))
            print(f'  step hedef = {value} adim (400 adim/s)')
        else:
            print('  bilinmeyen alt komut')
        return True

    if cmd == 'led':
        if len(parts) != 2 or parts[1].lower() not in ('on', 'off'):
            print('  kullanim: led on | led off')
            return True
        state = parts[1].lower() == 'on'
        link.send(proto.encode_led(state))
        print(f'  LED {"acik" if state else "kapali"}')
        return True

    if cmd == 'status':
        print_status(link)
        return True

    if cmd == 'mon':
        duration = 5.0
        if len(parts) == 2:
            try:
                duration = float(parts[1])
            except ValueError:
                pass
        end = time.monotonic() + duration
        while time.monotonic() < end:
            print_status(link)
            print()
            time.sleep(0.5)
        return True

    print(f'  bilinmeyen komut: {cmd}  (yardim icin dosyanin basini oku)')
    return True


def main():
    parser = argparse.ArgumentParser(
        description='STM32 UART hattini ROS olmadan test et')
    parser.add_argument('--port', default='/dev/ttyAMA0',
                        help='seri aygit (Pi 5 GPIO14/15 = /dev/ttyAMA0)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--rate', type=float, default=50.0,
                        help='ESC cercevesi gonderim frekansi (Hz)')
    args = parser.parse_args()

    if not os.path.exists(args.port):
        print(f'{args.port} yok. Pi 5te GPIO14/15 UARTi icin config.txt icinde '
              '"dtparam=uart0=on" gerekiyor — ./setup_hardware.sh calistir.')
        return 1

    try:
        link = Link(args.port, args.baud, args.rate)
    except Exception as error:  # noqa: B902
        print(f'{args.port} acilamadi: {error}')
        return 1

    print(f'{args.port} @ {args.baud} baud, {args.rate:.0f} Hz ESC cercevesi.')
    print('STM32 arm sekansi ~2 s suruyor; oncesinde komutlar uygulanmaz.')
    print('Komut listesi icin dosyanin basindaki docstring. Cikis: q\n')
    time.sleep(1.0)
    print_status(link)

    try:
        while True:
            try:
                text = input('stm32> ')
            except EOFError:
                break
            if not handle(link, text):
                break
    except KeyboardInterrupt:
        print()
    finally:
        print('Notre alinip bobinler birakiliyor...')
        link.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
