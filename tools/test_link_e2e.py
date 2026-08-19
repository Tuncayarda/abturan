#!/usr/bin/env python3
"""
Protokolun uctan uca testi — donanim gerekmez.

tools/fake_stm32.py'yi gercek bir sanal seri port (pty) uzerinde calistirip
Pi tarafinin gonderdigi cerceveleri, STM32 tarafinin verdigi STATUS'u,
arm sekansini ve failsafe'i dogruluyor. Kart takmadan once ve protokolde
degisiklik yaptiktan sonra kosturulacak test bu.

C tarafiyla bayt uyumu ayrica stm32_fw/test_protocol.c ile test ediliyor:
    cd stm32_fw && gcc -Wall -Wextra -Werror -std=c11 -IInc \
        Src/up_protocol.c test_protocol.c -o /tmp/up_test && /tmp/up_test

Kullanim:
    python3 tools/test_link_e2e.py
"""

import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'stm32_bridge'))

from stm32_bridge import protocol as proto  # noqa: E402


class Harness:
    """Sanal STM32'yi baslatir ve pty'ye baglanir (pyserial gerekmez)."""

    def __init__(self):
        self.link = os.path.join(tempfile.mkdtemp(prefix='up_e2e_'), 'ttyFAKE')
        self.proc = subprocess.Popen(
            [sys.executable, os.path.join(HERE, 'fake_stm32.py'),
             '--link', self.link],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for _ in range(50):
            if os.path.exists(self.link):
                break
            time.sleep(0.1)
        else:
            self.fail('sanal port olusmadi')
        self.fd = os.open(self.link, os.O_RDWR | os.O_NOCTTY)
        os.set_blocking(self.fd, False)
        self.parser = proto.FrameParser()
        self.statuses = []
        self.logs = []

    def fail(self, message):
        out = ''
        if self.proc.poll() is not None and self.proc.stdout:
            out = self.proc.stdout.read()
        self.close()
        raise SystemExit(f'HATA: {message}\n{out}')

    def pump(self, seconds, frames=(), send_esc=True):
        """Belirtilen sure boyunca cerceve alisverisi yap."""
        for frame in frames:
            os.write(self.fd, frame)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                data = b''
            for msg_id, payload in self.parser.feed(data):
                if msg_id == proto.MSG_STATUS:
                    self.statuses.append(proto.decode_status(payload))
                elif msg_id == proto.MSG_LOG:
                    text = payload.decode('ascii', 'replace')
                    self.logs.append(text)
                    print(f'    [stm32] {text}')
            if send_esc:
                os.write(self.fd, proto.encode_esc([1600] * proto.ESC_COUNT))
            time.sleep(0.02)
        if self.proc.poll() is not None:
            self.fail('sanal STM32 coktu')
        return self.statuses[-1] if self.statuses else None

    def close(self):
        if getattr(self, 'fd', None) is not None:
            os.close(self.fd)
            self.fd = None
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()


def check(cond, what):
    print(f'  {what:<48} {"OK" if cond else "FAIL"}')
    return 0 if cond else 1


def main():
    h = Harness()
    failures = 0
    try:
        print('1) arm sekansi (2 s notr) ve hat kurulumu')
        st = h.pump(2.6)
        failures += check(st is not None, 'STATUS cerceveleri geliyor')
        failures += check(bool([s for s in h.statuses if s['esc_armed']]),
                          'ESC arm tamamlandi')
        failures += check(st is not None and not st['failsafe'],
                          'komut akarken failsafe kapali')

        print('2) step motor: VELOCITY 400 adim/s, 1 s')
        h.statuses.clear()
        st = h.pump(1.0, [proto.encode_stepper(proto.STEP_VELOCITY, 400, 0)])
        pos = st['stepper_position']
        failures += check(300 < pos < 500, f'konum ~400 adim (olculen {pos})')
        failures += check(st['stepper_energized'], 'bobinler enerjili')

        print('3) LED')
        h.statuses.clear()
        st = h.pump(0.4, [proto.encode_led(True)])
        failures += check(st['led'], 'LED acildi')

        print('4) step motor: POSITION hedef 0')
        h.statuses.clear()
        st = h.pump(1.5, [proto.encode_stepper(proto.STEP_POSITION, 800, 0)])
        failures += check(abs(st['stepper_position']) <= 2,
                          f'hedefe varildi (konum {st["stepper_position"]})')

        print('5) komut kesilince failsafe')
        h.statuses.clear()
        st = h.pump(1.2, send_esc=False)
        failures += check(st['failsafe'], 'failsafe devreye girdi')
        failures += check(not st['stepper_energized'],
                          'failsafe bobinleri birakti')

        print('6) cerceve sagligi')
        failures += check(st['rx_err'] == 0,
                          f'STM32 tarafinda cerceve hatasi yok ({st["rx_err"]})')
        failures += check(h.parser.error_count == 0,
                          f'Pi tarafinda cerceve hatasi yok ({h.parser.error_count})')
    finally:
        h.close()

    print()
    if failures:
        print(f'BASARISIZ — {failures} kontrol gecmedi')
        return 1
    print('TUM KONTROLLER GECTI')
    return 0


if __name__ == '__main__':
    sys.exit(main())
