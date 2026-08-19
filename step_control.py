#!/usr/bin/env python3
"""
NEMA17 + L298N + Raspberry Pi 5
Terminalden interaktif step motor kontrolu.

Kullanim:
    python3 step_control.py
    python3 step_control.py --reverse
    python3 step_control.py --delay 3 --half
"""

import argparse
import gc
import math
import os
import sys
import time

from gpiozero import DigitalOutputDevice, PWMOutputDevice

# ---------------------------------------------------------------
# Pin tanimlari (BCM numaralari)
# ---------------------------------------------------------------
IN1_PIN = 17
IN2_PIN = 27
IN3_PIN = 22
IN4_PIN = 23

# ENA/ENB varsayilan olarak bagli degil (kartta jumper takili).
# Mikroadim icin jumper'lari cikarip bu pinlere kablo cekmek gerekiyor.
ENA_PIN = None
ENB_PIN = None

# lgpio bu Pi'de 10 kHz'e kadar kabul ediyor; olculdu, adim
# zamanlamasini hic bozmuyor (PWM daemon tarafinda uretiliyor).
PWM_FREQ = 10000

# L298N iki transistor serisi oldugu icin kopru uzerinde kayda deger
# gerilim dusuyor. 1-2 A civarinda tipik toplam deger.
L298N_DROP = 2.0


def power_for_current(amps, ohms, supply_volts):
    """DURAN motorda bobin akimini anma degerinde tutacak duty oranini hesapla.

    Kararli halde I = V_ort / R ve V_ort = duty * (Vbesleme - Vdusum).

    DIKKAT: bu hesap sadece motor dururken gecerli, yani HOLD icin.
    Motor donmeye baslayinca ters-EMK devreye giriyor; duty'yi bu
    degere kismak elde kalan gerilim basligini da kisiyor ve akim hic
    kurulamiyor -> tork cokuyor, motor kalkamiyor. Gercek chopper
    surucu tam gerilimi uygulayip kiyarak akimi ZORLAR; duty ile
    kismak bunu yapamaz. Bu yuzden surus gucu yuksek, hold gucu
    dusuk tutuluyor.
    """
    headroom = supply_volts - L298N_DROP
    if headroom <= 0:
        raise ValueError(f"besleme {supply_volts} V, L298N dusumunu karsilamiyor")
    return amps * ohms / headroom

# ---------------------------------------------------------------
# Zamanlama sabitleri
# ---------------------------------------------------------------

# Bu Pi 5 cekirdeginde time.sleep olcumle ~60 us tepe sapma veriyor;
# 5 ms'lik adimda %1.2, yani sorun degil. Onun altindaki artik sureyi
# sleep zaten cozemedigi icin sadece o kadarlik kismi donerek gecistiriyoruz.
# (Uzun spin denendi ve daha kotu: planlayici donen thread'i kesince
# tepe sapma 2 ms'ye kadar cikiyor.)
SPIN_THRESHOLD = 0.00015

# Rampanin ilk adiminda hiz kac kat yavas olsun.
RAMP_START_FACTOR = 4.0

# Bobinler enerjisizken rotor en yakin detent'e kayar. Harekete
# baslamadan once fazi verip rotorun oturmasini bekliyoruz.
SETTLE_TIME = 0.02

# ---------------------------------------------------------------
# Faz dizileri. Her satir = (IN1, IN2, IN3, IN4)
# ---------------------------------------------------------------

# Full-step: iki bobin de her zaman enerjili. Tork yuksek, cozunurluk 200/tur.
FULL_STEP = [
    (1, 0, 1, 0),   # A+ B+
    (0, 1, 1, 0),   # A- B+
    (0, 1, 0, 1),   # A- B-
    (1, 0, 0, 1),   # A+ B-
]

# Half-step: aralara tek bobinli durumlar giriyor.
# Daha yumusak ve sessiz, cozunurluk 400/tur, tork biraz dusuk.
HALF_STEP = [
    (1, 0, 1, 0),   # A+ B+
    (0, 0, 1, 0),   #    B+
    (0, 1, 1, 0),   # A- B+
    (0, 1, 0, 0),   # A-
    (0, 1, 0, 1),   # A- B-
    (0, 0, 0, 1),   #    B-
    (1, 0, 0, 1),   # A+ B-
    (1, 0, 0, 0),   # A+
]


def build_microstep_table(micro):
    """Sinuzoidal mikroadim tablosu. Satir = (in1, in2, in3, in4, duty_a, duty_b).

    Full/half-step'te bobin akimi kare dalga; her adimda tork vektoru
    hem yon hem BUYUKLUK degistiriyor. Titremenin kaynagi bu.
    Burada akim vektorunun buyuklugu sabit tutulup sadece acisi
    donduruluyor (a=cos, b=sin) -> pratikte suruklenmesiz, sessiz hareket.

    Yarim ornek kaydirmasi (i + 0.5) sayesinde micro=1 tam olarak
    full-step konumlarina, micro=2 half-step konumlarina denk geliyor;
    ama half-step'in tek/cift bobin tork ucurumu olmadan.
    """
    total = 4 * micro
    table = []
    for i in range(total):
        theta = 2.0 * math.pi * (i + 0.5) / total
        a = math.cos(theta)
        b = math.sin(theta)
        table.append((
            1 if a > 0 else 0,   # IN1
            1 if a < 0 else 0,   # IN2
            1 if b > 0 else 0,   # IN3
            1 if b < 0 else 0,   # IN4
            abs(a),              # ENA duty
            abs(b),              # ENB duty
        ))
    return table


def sleep_until(deadline):
    """Adim araligini bekle. Uzun kismi uyuyarak, artigi donerek."""
    slack = deadline - time.perf_counter()
    if slack > SPIN_THRESHOLD:
        time.sleep(slack - SPIN_THRESHOLD)
    while time.perf_counter() < deadline:
        pass


class Stepper:
    """L298N uzerinden bipolar step motor surucusu."""

    def __init__(self, pins, half=False, step_delay=0.005,
                 reverse=False, hold=False, ramp_steps=60,
                 enable_pins=None, micro=1, power=1.0, hold_power=0.35):
        self.outputs = [DigitalOutputDevice(p, initial_value=False) for p in pins]

        if enable_pins:
            self.enables = [PWMOutputDevice(p, frequency=PWM_FREQ, initial_value=0.0)
                            for p in enable_pins]
            self.micro = micro
            self.sequence = build_microstep_table(micro)
        else:
            self.enables = None
            self.micro = 2 if half else 1
            self.sequence = HALF_STEP if half else FULL_STEP

        self.power = power              # surus akimi olcegi (0-1)
        self.hold_power = hold_power    # kilitli beklerken akim olcegi
        self.phase = 0                  # kaldigi yeri hatirlar
        self.energized = False          # bobinlerde su an akim var mi
        self.step_delay = step_delay
        self.reverse = reverse
        self.ramp_steps = ramp_steps
        self.position = 0               # baslangictan beri net adim
        self._hold = False
        self.hold = hold                # setter: acikken bobinleri hemen kilitler

    @property
    def microstepping(self):
        return self.enables is not None

    # -- dusuk seviye ---------------------------------------------------

    def _apply_phase(self, index, scale=None):
        """Fazi glitch'siz uygula.

        Once 0'a dusecek pinleri, sonra 1'e cikacaklari yaziyoruz.
        Tersi olursa (0,1) -> (1,0) gecisinde ayni H-koprunun iki girisi
        bir an ikisi de 1 kaliyor; L298N'de bu fren + akim sicramasi demek.
        """
        row = self.sequence[index]
        target = row[:4]
        for output, value in zip(self.outputs, target):
            if not value:
                output.value = 0
        for output, value in zip(self.outputs, target):
            if value:
                output.value = 1

        if self.enables is not None:
            if scale is None:
                scale = self.power
            self.enables[0].value = min(1.0, row[4] * scale)
            self.enables[1].value = min(1.0, row[5] * scale)

        self.energized = True

    def release(self):
        """Bobinlerin enerjisini kes. Motor serbest kalir, isinma durur."""
        if self.enables is not None:
            for enable in self.enables:
                enable.value = 0.0
        for output in self.outputs:
            output.off()
        self.energized = False

    # -- hold -----------------------------------------------------------

    @property
    def hold(self):
        return self._hold

    @hold.setter
    def hold(self, value):
        """Hold acilinca bobinler ANINDA enerjilenir.

        Eskiden sadece bir bayrak set ediliyordu ve etkisi ancak bir
        sonraki hareketin sonunda goruluyordu; arada motor serbest
        kaldigi icin 'hold calismiyor' gibi duruyordu.
        """
        self._hold = bool(value)
        if self._hold:
            # Kilitli beklerken dusuk akim yetiyor. ENA/ENB bagliysa
            # bu, L298N'de chopper olmadigi icin sart: tam akimda
            # NEMA17 dakikalar icinde kizariyor.
            self._apply_phase(self.phase, scale=self.hold_power)
        else:
            self.release()

    # -- hareket --------------------------------------------------------

    def _delay_for(self, index, total):
        """Basta ve sonda yavas, ortada tam hizli. Trapez rampa."""
        if self.ramp_steps <= 0:
            return self.step_delay
        ramp = min(self.ramp_steps, total // 2)
        if ramp == 0:
            return self.step_delay
        distance = min(index, total - 1 - index)
        if distance >= ramp:
            return self.step_delay
        # Geometrik rampa: distance=0'da RAMP_START_FACTOR kat yavas,
        # rampa sonunda tam olarak 1.0'a oturur. Eski dogrusal formul
        # rampa bitiminde hizda sicrama birakiyordu.
        factor = RAMP_START_FACTOR ** (1.0 - distance / ramp)
        return self.step_delay * factor

    def move(self, steps):
        """steps pozitifse ileri, negatifse geri. --reverse bunu ters cevirir."""
        if steps == 0:
            return

        direction = 1 if steps > 0 else -1
        if self.reverse:
            direction = -direction

        # Bobinler bostaysa rotor en yakin detent'e kaymis olabilir.
        # Once mevcut fazi ver, otursun; yoksa ilk adim tekliyor.
        if not self.energized:
            self._apply_phase(self.phase)
            time.sleep(SETTLE_TIME)

        count = abs(steps)
        gc_was_on = gc.isenabled()
        gc.disable()          # GC duraklamasi da jitter uretiyor
        try:
            deadline = time.perf_counter()
            for i in range(count):
                self.phase = (self.phase + direction) % len(self.sequence)
                self._apply_phase(self.phase)
                self.position += direction

                deadline += self._delay_for(i, count)
                if deadline < time.perf_counter():
                    # Python geride kaldi, birikmesin diye saati sifirla
                    deadline = time.perf_counter()
                else:
                    sleep_until(deadline)
        finally:
            if gc_was_on:
                gc.enable()
            if self._hold:
                self._apply_phase(self.phase, scale=self.hold_power)
            else:
                self.release()

    def close(self):
        self.release()
        for output in self.outputs:
            output.close()
        if self.enables is not None:
            for enable in self.enables:
                enable.close()


def try_realtime():
    """Adim dongusunu SCHED_FIFO'ya al. Root degilsek sessizce vazgec."""
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
        return True
    except (AttributeError, OSError):
        return False


# ---------------------------------------------------------------
# Terminal arayuzu
# ---------------------------------------------------------------

HELP_TEXT = """
Komutlar:
  200          200 adim ileri
  -200         200 adim geri
  2t           2 tam tur (adim sayisi otomatik hesaplanir)
  rpm 120      hizi devir/dakika olarak ayarla
  sweep        rezonans taramasi: her hizda 1 tur gider gelir, dinle
  r            yon cevir (reverse ac/kapa)
  d            gecerli gecikmeyi goster
  d 3          adimlar arasi gecikmeyi 3 ms yap
  p 0.6        surus gucu / akim olcegi (sadece ENA/ENB bagliysa)
  hold         bobinleri enerjili tut ac/kapa (aninda etki eder)
  free         bobinleri hemen birak (hold'u da kapatir)
  pos          baslangictan beri net adim sayisini goster
  zero         konum sayacini sifirla
  ?            bu yardim
  q            cikis
""".strip()

MIN_DELAY_MS = 0.3

# Rezonans taramasinda denenecek hizlar (devir/dakika).
SWEEP_RPMS = [15, 30, 45, 60, 80, 100, 130, 160, 200, 250, 300, 400]


def rpm_to_delay(rpm, steps_per_rev):
    return 60.0 / (rpm * steps_per_rev)


def delay_to_rpm(delay, steps_per_rev):
    return 60.0 / (delay * steps_per_rev)


def run_sweep(motor, steps_per_rev):
    """Her hizda bir tur gidip gelir. Amac rezonans bolgesini KULAKLA bulmak.

    NEMA17'nin full-step mid-band rezonansi tipik olarak 60-120 rpm
    araligina dusuyor ve orada motor donmek yerine zirildiyor. Hangi
    hizda sessiz oldugunu gormeden dogru --delay secilemez.
    """
    original = motor.step_delay
    print("  Her hizda 1 tur ileri + 1 tur geri. En sessiz olani not al.")
    print("  Durdurmak icin Ctrl-C.\n")
    try:
        for rpm in SWEEP_RPMS:
            delay = rpm_to_delay(rpm, steps_per_rev)
            if delay < MIN_DELAY_MS / 1000.0:
                print(f"  {rpm:4d} rpm -> atlandi (gecikme {delay*1000:.2f} ms, cok kisa)")
                continue
            motor.step_delay = delay
            print(f"  {rpm:4d} rpm ({delay*1000:5.2f} ms/adim) ...", end="", flush=True)
            motor.move(steps_per_rev)
            motor.move(-steps_per_rev)
            print(" ok")
            time.sleep(0.4)
    except KeyboardInterrupt:
        if not motor.hold:
            motor.release()
        print("\n  tarama durduruldu.")
    finally:
        motor.step_delay = original
    print(f"  gecikme geri alindi: {original*1000:.2f} ms")


def parse_command(text, steps_per_rev):
    """Kullanici girdisini adim sayisina cevirir. Anlamazsa None doner."""
    text = text.strip().lower().replace(",", ".")
    if not text:
        return None

    multiplier = 1.0
    if text.endswith("tur"):
        text, multiplier = text[:-3].strip(), steps_per_rev
    elif text.endswith("t"):
        text, multiplier = text[:-1].strip(), steps_per_rev

    if text in ("", "+"):
        text = "1"
    elif text == "-":
        text = "-1"

    try:
        return int(round(float(text) * multiplier))
    except ValueError:
        return None


def interactive_loop(motor, steps_per_rev):
    print(HELP_TEXT)
    print()
    if motor.microstepping:
        mode = f"mikroadim 1/{motor.micro} (guc {motor.power:.2f})"
    else:
        mode = ("half-step" if motor.micro == 2 else "full-step") + \
               " -- ENA/ENB bagli degil, akim sinirlamasi YOK"
    print(f"Hazir. {mode} | {steps_per_rev} adim/tur")
    print(f"Yon: {'ters' if motor.reverse else 'normal'} | "
          f"gecikme: {motor.step_delay * 1000:.2f} ms "
          f"({delay_to_rpm(motor.step_delay, steps_per_rev):.0f} rpm) | "
          f"hold: {'acik' if motor.hold else 'kapali'}")

    while True:
        try:
            raw = input("adim> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        if not raw:
            continue

        command = raw.lower()

        if command in ("q", "quit", "exit", "cikis"):
            return

        if command in ("?", "h", "help", "yardim"):
            print(HELP_TEXT)
            continue

        if command == "r":
            motor.reverse = not motor.reverse
            print(f"  yon: {'ters' if motor.reverse else 'normal'}")
            continue

        if command == "hold":
            motor.hold = not motor.hold
            print(f"  hold: {'acik (motor kilitli, isinir)' if motor.hold else 'kapali'}")
            continue

        if command in ("free", "bos", "serbest"):
            motor.hold = False
            print("  bobinler serbest")
            continue

        if command == "pos":
            turns = motor.position / steps_per_rev
            print(f"  konum: {motor.position} adim ({turns:+.2f} tur)")
            continue

        if command == "zero":
            motor.position = 0
            print("  konum sifirlandi")
            continue

        if command == "sweep":
            run_sweep(motor, steps_per_rev)
            continue

        parts = command.split()
        if parts[0] == "d" and len(parts) <= 2:
            if len(parts) == 1:
                print(f"  gecikme: {motor.step_delay * 1000:.2f} ms "
                      f"({delay_to_rpm(motor.step_delay, steps_per_rev):.0f} rpm)")
                continue
            try:
                ms = float(parts[1].replace(",", "."))
            except ValueError:
                print("  ornek: d 3")
                continue
            if ms < MIN_DELAY_MS:
                print(f"  {MIN_DELAY_MS} ms altinda motor zaten adim atamaz.")
                continue
            motor.step_delay = ms / 1000.0
            print(f"  gecikme: {ms:.2f} ms "
                  f"({delay_to_rpm(motor.step_delay, steps_per_rev):.0f} rpm)")
            continue

        if parts[0] == "rpm" and len(parts) == 2:
            try:
                rpm = float(parts[1].replace(",", "."))
            except ValueError:
                print("  ornek: rpm 120")
                continue
            if rpm <= 0:
                print("  rpm pozitif olmali.")
                continue
            delay = rpm_to_delay(rpm, steps_per_rev)
            if delay < MIN_DELAY_MS / 1000.0:
                print(f"  {rpm:.0f} rpm bu cozunurlukte {delay*1000:.2f} ms/adim "
                      f"gerektiriyor, cok kisa.")
                continue
            motor.step_delay = delay
            print(f"  hiz: {rpm:.0f} rpm ({delay*1000:.2f} ms/adim)")
            continue

        if parts[0] == "p" and len(parts) <= 2:
            if not motor.microstepping:
                print("  guc ayari icin ENA/ENB pinleri bagli olmali "
                      "(--ena / --enb).")
                continue
            if len(parts) == 1:
                print(f"  guc: {motor.power:.2f} | hold gucu: {motor.hold_power:.2f}")
                continue
            try:
                value = float(parts[1].replace(",", "."))
            except ValueError:
                print("  ornek: p 0.6")
                continue
            if not 0.05 <= value <= 1.0:
                print("  guc 0.05 - 1.0 arasinda olmali.")
                continue
            motor.power = value
            if motor.hold:
                motor.hold = True     # yeni gucle yeniden uygula
            print(f"  guc: {value:.2f}")
            continue

        steps = parse_command(command, steps_per_rev)
        if steps is None:
            print("  anlamadim, '?' yaz.")
            continue

        if steps == 0:
            continue

        effective = -steps if motor.reverse else steps
        label = "ileri" if effective > 0 else "geri"
        print(f"  {abs(steps)} adim {label}...", end="", flush=True)

        started = time.perf_counter()
        try:
            motor.move(steps)
        except KeyboardInterrupt:
            # hold acikken durdurmak bobinleri bosaltmamali
            if not motor.hold:
                motor.release()
            print(" durduruldu.")
            continue
        print(f" bitti ({time.perf_counter() - started:.2f} s)")


def main():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi 5 + L298N ile NEMA17 step motor kontrolu")
    parser.add_argument("--reverse", action="store_true",
                        help="tum yonleri ters cevir")
    parser.add_argument("--half", action="store_true",
                        help="half-step modu (daha yumusak, 400 adim/tur)")
    parser.add_argument("--delay", type=float, default=None,
                        help="adimlar arasi gecikme, milisaniye "
                             "(verilmezse hiz 60 rpm olacak sekilde secilir)")
    parser.add_argument("--rpm", type=float, default=None,
                        help="hizi devir/dakika olarak ver (--delay yerine)")
    parser.add_argument("--ena", type=int, default=ENA_PIN,
                        help="L298N ENA pini (BCM). Mikroadim icin gerekli.")
    parser.add_argument("--enb", type=int, default=ENB_PIN,
                        help="L298N ENB pini (BCM). Mikroadim icin gerekli.")
    parser.add_argument("--micro", type=int, default=8,
                        choices=(1, 2, 4, 8, 16, 32),
                        help="tam adim basina mikroadim (ENA/ENB bagliysa)")
    parser.add_argument("--power", type=float, default=None,
                        help="surus akimi olcegi 0-1. Verilmezse "
                             "--coil-amps/--coil-ohms'tan hesaplanir, "
                             "o da yoksa 0.30")
    parser.add_argument("--hold-power", type=float, default=None,
                        help="kilitli beklerken akim olcegi 0-1 "
                             "(varsayilan: surus gucunun yarisi)")
    parser.add_argument("--coil-amps", type=float, default=None,
                        help="motorun faz basina anma akimi, amper "
                             "(17HS4401 icin 1.7)")
    parser.add_argument("--coil-ohms", type=float, default=None,
                        help="faz direnci, ohm (17HS4401 icin 1.5)")
    parser.add_argument("--supply-volts", type=float, default=12.0,
                        help="L298N motor besleme gerilimi (varsayilan 12)")
    parser.add_argument("--hold", action="store_true",
                        help="bobinleri enerjili tut (baslangicta da kilitler)")
    parser.add_argument("--no-ramp", action="store_true",
                        help="hizlanma/yavaslama rampasini kapat")
    parser.add_argument("--ramp-steps", type=int, default=60,
                        help="rampa uzunlugu, TAM adim cinsinden "
                             "(varsayilan 60; mikroadimla otomatik olceklenir)")
    parser.add_argument("--steps-per-rev", type=int, default=200,
                        help="motorun tam adim sayisi (1.8 derece icin 200)")
    parser.add_argument("--no-rt", action="store_true",
                        help="gercek zamanli oncelik denemesini atla")
    parser.add_argument("--pins", type=int, nargs=4,
                        default=[IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN],
                        metavar=("IN1", "IN2", "IN3", "IN4"),
                        help="BCM pin numaralari")
    args = parser.parse_args()

    if (args.ena is None) != (args.enb is None):
        parser.error("--ena ve --enb birlikte verilmeli.")

    enable_pins = None
    if args.ena is not None:
        enable_pins = [args.ena, args.enb]
        substeps = args.micro
    else:
        substeps = 2 if args.half else 1
        if args.micro != 8:
            print("Not: --micro ancak --ena/--enb bagliyken ise yarar, "
                  "yok sayiliyor.", file=sys.stderr)

    steps_per_rev = args.steps_per_rev * substeps

    if args.delay is not None and args.rpm is not None:
        parser.error("--delay ve --rpm birlikte verilemez.")
    if args.rpm is not None:
        if args.rpm <= 0:
            parser.error("--rpm pozitif olmali.")
        delay_ms = rpm_to_delay(args.rpm, steps_per_rev) * 1000.0
    elif args.delay is not None:
        delay_ms = args.delay
    else:
        # Cozunurluk degisince ayni RPM'i korumak icin gecikmeyi olcekle;
        # yoksa --micro 8 motoru 8 kat yavaslatiyor.
        delay_ms = rpm_to_delay(60.0, steps_per_rev) * 1000.0

    if delay_ms < MIN_DELAY_MS:
        parser.error(f"Istenen hiz {delay_ms:.2f} ms/adim gerektiriyor, "
                     f"alt sinir {MIN_DELAY_MS} ms. Daha dusuk --micro dene.")

    # -- surus ve kilit akimi ---------------------------------------
    # Surus gucu varsayilan olarak TAM: ters-EMK'yi yenecek gerilim
    # basligi lazim, yoksa motor kalkmiyor. Isinmayi hold tarafinda
    # kisiyoruz, cunku motor dururken akimi sinirlayan tek sey duty.
    power = 1.0 if args.power is None else args.power
    if not 0.05 <= power <= 1.0:
        parser.error("--power 0.05 - 1.0 arasinda olmali.")

    if args.hold_power is not None:
        hold_power = args.hold_power
    elif args.coil_amps and args.coil_ohms:
        try:
            hold_power = power_for_current(args.coil_amps, args.coil_ohms,
                                           args.supply_volts)
        except ValueError as error:
            parser.error(str(error))
        hold_power = min(1.0, hold_power)
        if enable_pins:
            print(f"Kilit gucu: {hold_power:.2f} "
                  f"({args.coil_amps} A x {args.coil_ohms} ohm / "
                  f"{args.supply_volts - L298N_DROP:.1f} V etkin)")
    else:
        hold_power = 0.30

    hold_power = max(0.05, min(power, hold_power))

    if enable_pins is None and (args.power is not None or args.coil_amps):
        print("Uyari: guc ayari ancak --ena/--enb bagliyken uygulanabilir; "
              "ENA/ENB jumper'liyken bobinler tam gerilimde suruluyor.",
              file=sys.stderr)

    if not args.no_rt and not try_realtime():
        print("Not: gercek zamanli oncelik alinamadi (root degilsin). "
              "Daha duzgun adim icin: sudo python3 step_control.py")

    try:
        motor = Stepper(
            pins=args.pins,
            half=args.half,
            step_delay=delay_ms / 1000.0,
            reverse=args.reverse,
            hold=args.hold,
            # Rampa TAM adim cinsinden veriliyor. Mikroadimda cevirmezsek
            # --micro 8'de rampa 8 kat kisaliyor ve motor kalkamadan
            # tam hiza firliyor.
            ramp_steps=0 if args.no_ramp else args.ramp_steps * substeps,
            enable_pins=enable_pins,
            micro=args.micro,
            power=power,
            hold_power=hold_power,
        )
    except Exception as error:
        print(f"GPIO acilamadi: {error}", file=sys.stderr)
        print("Pi 5 kullaniyorsan lgpio kurulu olmali: "
              "sudo apt install -y python3-lgpio python3-gpiozero", file=sys.stderr)
        return 1

    try:
        interactive_loop(motor, steps_per_rev)
    finally:
        motor.close()
        print("Motor kapatildi.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
