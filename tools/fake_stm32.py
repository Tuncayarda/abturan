#!/usr/bin/env python3
"""
Sanal STM32 — kart olmadan tum ROS zincirini test etmek icin.

Bir sanal seri port (pty) acar ve gercek firmware gibi davranir:
arm sekansi, 500 ms failsafe, step motor konum entegrasyonu ve 10 Hz
STATUS cercevesi. stm32_bridge'i buna baglayip joystick -> allocator ->
koprü yolunun tamamini donanimsiz dogrulayabilirsin.

Kullanim:
    python3 tools/fake_stm32.py --link /tmp/ttySTM32
    # baska bir terminalde:
    ros2 run stm32_bridge stm32_bridge --ros-args -p serial_port:=/tmp/ttySTM32

Sadece standart kutuphane kullanir (pyserial gerekmez).
"""

import argparse
import os
import pty
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'stm32_bridge'))

from stm32_bridge import protocol as proto  # noqa: E402

ARM_MS = 2000
FAILSAFE_MS = 500
STATUS_PERIOD = 0.1
TICK = 0.01          # 100 Hz simulasyon adimi


class FakeStm32:

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.parser = proto.FrameParser()
        self.boot = time.monotonic()
        self.last_cmd = self.boot
        self.last_status = 0.0

        self.esc_us = [proto.ESC_NEUTRAL_US] * proto.ESC_COUNT
        self.armed = False
        self.failsafe = True
        self.led = False

        self.step_mode = proto.STEP_IDLE
        self.step_speed = 0
        self.step_target = 0
        self.step_position = 0.0
        self.step_energized = False

    # ------------------------------------------------------------------
    def uptime_ms(self):
        return int((time.monotonic() - self.boot) * 1000)

    def handle(self, msg_id, payload):
        import struct

        if msg_id == proto.MSG_ESC:
            self.esc_us = list(struct.unpack('<6H', payload))
            self.last_cmd = time.monotonic()
        elif msg_id == proto.MSG_STEPPER:
            mode, speed, target = proto.STEPPER_STRUCT.unpack(payload)
            self.step_mode = mode
            self.step_speed = speed
            self.step_target = target
            self.last_cmd = time.monotonic()
            if self.verbose:
                print(f'  step -> {proto.STEP_MODE_NAMES.get(mode, "?")} '
                      f'hiz={speed} hedef={target}')
        elif msg_id == proto.MSG_LED:
            self.led = bool(payload[0])
            self.last_cmd = time.monotonic()
            if self.verbose:
                print(f'  led -> {"acik" if self.led else "kapali"}')
        elif msg_id == proto.MSG_HEARTBEAT:
            self.last_cmd = time.monotonic()

    def tick(self, dt):
        now = time.monotonic()

        if not self.armed and (now - self.boot) * 1000 >= ARM_MS:
            self.armed = True
            return proto.encode_log('ESC arm tamam')

        timed_out = (now - self.last_cmd) * 1000 > FAILSAFE_MS
        log = None
        if self.armed and timed_out != self.failsafe:
            self.failsafe = timed_out
            log = proto.encode_log('FAILSAFE: komut yok' if timed_out
                                   else 'komut geri geldi')
        if self.failsafe:
            self.step_mode = proto.STEP_IDLE

        # Step motor konumu
        if self.step_mode == proto.STEP_IDLE:
            self.step_energized = False
        else:
            self.step_energized = True
            if self.step_mode == proto.STEP_VELOCITY:
                self.step_position += self.step_speed * dt
            elif self.step_mode == proto.STEP_POSITION:
                remaining = self.step_target - self.step_position
                move = abs(self.step_speed) * dt
                if abs(remaining) <= move:
                    self.step_position = float(self.step_target)
                    self.step_mode = proto.STEP_HOLD
                else:
                    self.step_position += move * (1 if remaining > 0 else -1)
        return log

    def status_frame(self):
        flags = 0
        if self.failsafe:
            flags |= proto.FLAG_FAILSAFE
        if self.armed:
            flags |= proto.FLAG_ESC_ARMED
        if self.step_energized:
            flags |= proto.FLAG_STEP_ENERGIZED
        return proto.encode_status(
            uptime_ms=self.uptime_ms(),
            flags=flags,
            led=self.led,
            stepper_position=int(self.step_position),
            stepper_speed_sps=int(self.step_speed),
            rx_ok=self.parser.ok_count,
            rx_err=self.parser.error_count,
        )


def main():
    parser = argparse.ArgumentParser(description='Sanal STM32 (pty)')
    parser.add_argument('--link', help='bu yola sembolik bag kur (or. /tmp/ttySTM32)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='gelen step/led komutlarini yaz')
    parser.add_argument('--print-esc', action='store_true',
                        help='ESC darbelerini saniyede bir yaz')
    args = parser.parse_args()

    master, slave = pty.openpty()
    # Ham (raw) kip sart: pty varsayilan olarak satir tamponlu ve ECHO acik
    # geliyor; ikili cerceveler icinde 0x0A/0x0D gecerken satir isleme
    # devreye girip baytlari bozuyor, echo da hatti kendi verisiyle
    # tikiyor. pyserial actigi portta bunu kendisi yapar, ama duz
    # os.open ile baglanan istemciler icin burada garantiye aliyoruz.
    tty.setraw(slave)
    termios.tcflush(slave, termios.TCIOFLUSH)
    os.set_blocking(master, False)
    tty_path = os.ttyname(slave)

    if args.link:
        if os.path.islink(args.link) or os.path.exists(args.link):
            os.remove(args.link)
        os.symlink(tty_path, args.link)
        print(f'Sanal STM32 hazir: {args.link} -> {tty_path}')
    else:
        print(f'Sanal STM32 hazir: {tty_path}')
    print('Baglanmak icin:')
    port = args.link or tty_path
    print(f'  ros2 run stm32_bridge stm32_bridge --ros-args -p serial_port:={port}')
    print(f'  python3 tools/stm32_link_test.py --port {port}')
    print('Cikis: Ctrl+C\n')

    sim = FakeStm32(verbose=args.verbose)
    last_print = 0.0
    last = time.monotonic()

    try:
        while True:
            try:
                chunk = os.read(master, 4096)
            except BlockingIOError:
                chunk = b''
            except OSError:
                break

            if chunk:
                for msg_id, payload in sim.parser.feed(chunk):
                    sim.handle(msg_id, payload)

            now = time.monotonic()
            dt = now - last
            last = now
            log = sim.tick(dt)
            if log:
                os.write(master, log)

            if now - sim.last_status >= STATUS_PERIOD:
                sim.last_status = now
                os.write(master, sim.status_frame())

            if args.print_esc and now - last_print >= 1.0:
                last_print = now
                print(f'  ESC us: {sim.esc_us}  failsafe={sim.failsafe}  '
                      f'step={int(sim.step_position)}')

            time.sleep(TICK)
    except KeyboardInterrupt:
        print('\nkapatiliyor.')
    finally:
        os.close(master)
        os.close(slave)
        if args.link and os.path.islink(args.link):
            os.remove(args.link)
    return 0


if __name__ == '__main__':
    sys.exit(main())
