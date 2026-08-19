#!/usr/bin/env python3
"""ENA/ENB + mikroadim tani testi.

Amac: motorun neden donmedigini kademeli olarak daraltmak.
Kablolama mi yanlis, tork mu yetmiyor, hiz mi fazla?

Kullanim:
    python3 micro_test.py --ena 13 --enb 19
"""

import argparse
import time

from gpiozero import DigitalOutputDevice, PWMOutputDevice

from step_control import build_microstep_table, PWM_FREQ, sleep_until


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pins", type=int, nargs=4, default=[17, 27, 22, 23])
    parser.add_argument("--ena", type=int, required=True)
    parser.add_argument("--enb", type=int, required=True)
    parser.add_argument("--steps-per-rev", type=int, default=200)
    args = parser.parse_args()

    ins = [DigitalOutputDevice(p, initial_value=False) for p in args.pins]
    ena = PWMOutputDevice(args.ena, frequency=PWM_FREQ, initial_value=0.0)
    enb = PWMOutputDevice(args.enb, frequency=PWM_FREQ, initial_value=0.0)
    in1, in2, in3, in4 = ins

    def bosalt():
        ena.value = enb.value = 0.0
        for pin in ins:
            pin.off()

    def adimla(table, tur_sayisi, rpm, power):
        """table uzerinden verilen hizda don."""
        n = len(table)
        toplam = int(args.steps_per_rev * (n / 4) * tur_sayisi)
        period = 60.0 / (rpm * args.steps_per_rev * (n / 4))
        # rampa: ilk ve son %20, 4 kat yavastan baslar
        ramp = max(1, toplam // 5)
        deadline = time.perf_counter()
        for i in range(toplam):
            row = table[i % n]
            for pin, value in zip(ins, row[:4]):
                if not value:
                    pin.value = 0
            for pin, value in zip(ins, row[:4]):
                if value:
                    pin.value = 1
            ena.value = min(1.0, row[4] * power)
            enb.value = min(1.0, row[5] * power)

            d = min(i, toplam - 1 - i)
            factor = 4.0 ** (1.0 - min(d, ramp) / ramp)
            deadline += period * factor
            sleep_until(deadline)

    try:
        print("=" * 60)
        print("TEST 1 - ENA gercekten A bobinini kontrol ediyor mu?")
        print("=" * 60)
        in1.on(); in2.off(); in3.off(); in4.off()
        for duty in (1.0, 0.5, 0.0):
            ena.value = duty
            print(f"  ENA duty = {duty:.1f} -> mili ELINLE cevirmeye calis. "
                  f"{'SERT olmali' if duty else 'SERBEST olmali'}")
            time.sleep(5)
        bosalt()
        print("  Uc kademede de ayni his geldiyse ENA kablosu YANLIS.\n")

        print("=" * 60)
        print("TEST 2 - ENB gercekten B bobinini kontrol ediyor mu?")
        print("=" * 60)
        in1.off(); in2.off(); in3.on(); in4.off()
        for duty in (1.0, 0.5, 0.0):
            enb.value = duty
            print(f"  ENB duty = {duty:.1f} -> mili elinle cevir. "
                  f"{'SERT olmali' if duty else 'SERBEST olmali'}")
            time.sleep(5)
        bosalt()
        print()

        input("TEST 3 - tam gucte, cok yavas full-step (10 rpm). Enter...")
        adimla(build_microstep_table(1), 1, rpm=10, power=1.0)
        bosalt()
        print("  Bir tam tur dondu mu? DONMEDIYSE sorun kablolama/besleme.\n")

        input("TEST 4 - tam gucte, yavas 1/8 mikroadim (10 rpm). Enter...")
        adimla(build_microstep_table(8), 1, rpm=10, power=1.0)
        bosalt()
        print("  Test 3 dondu ama bu donmediyse sorun mikroadim tablosu.\n")

        input("TEST 5 - 1/8 mikroadim, artan hiz. Enter...")
        for rpm in (10, 30, 60, 120, 200):
            print(f"  {rpm} rpm ...", end="", flush=True)
            adimla(build_microstep_table(8), 1, rpm=rpm, power=1.0)
            bosalt()
            print(" ok - dondu mu, sesi nasil?")
            time.sleep(1)
        print("  Hangi hizda kekelemeye basladiysa ust sinirin o.\n")

        input("TEST 6 - 1/8 mikroadim 30 rpm, azalan guc. Enter...")
        for power in (1.0, 0.8, 0.6, 0.4, 0.26):
            print(f"  guc {power:.2f} ...", end="", flush=True)
            adimla(build_microstep_table(8), 1, rpm=30, power=power)
            bosalt()
            print(" ok - hala doniyor mu?")
            time.sleep(1)
        print("  Durdugu deger, kullanabilecegin alt guc sinirinin altidir.")

    except KeyboardInterrupt:
        print("\nkesildi.")
    finally:
        bosalt()
        for pin in ins:
            pin.close()
        ena.close()
        enb.close()
        print("\nPinler serbest.")


if __name__ == "__main__":
    main()
