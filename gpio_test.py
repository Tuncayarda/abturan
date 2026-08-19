#!/usr/bin/env python3
"""L298N tani testi. Sirayla pinleri enerjilendirir, arada elle kontrol istenir."""

import time
from gpiozero import DigitalOutputDevice

PINS = {"IN1": 17, "IN2": 27, "IN3": 22, "IN4": 23}

outs = {name: DigitalOutputDevice(pin, initial_value=False)
        for name, pin in PINS.items()}

try:
    print("TEST 1: her pin tek tek 3 saniye HIGH olacak.")
    print("Multimetren varsa ilgili OUT pini ile GND arasini olc.\n")
    for name, out in outs.items():
        print(f"  {name} (GPIO{PINS[name]}) HIGH ...", end="", flush=True)
        out.on()
        time.sleep(3)
        out.off()
        print(" LOW")

    input("\nTEST 2: bir bobin surekli enerjili tutulacak. Enter'a bas...")
    print("IN1=HIGH, IN2=LOW -> A bobini enerjili. 8 saniye.")
    print(">>> Motor milini ELINLE cevirmeye calis. Kilitli/sert mi?")
    outs["IN1"].on()
    outs["IN2"].off()
    time.sleep(8)
    outs["IN1"].off()
    print("bobin birakildi.")

    input("\nTEST 3: ikinci bobin. Enter'a bas...")
    print("IN3=HIGH, IN4=LOW -> B bobini enerjili. 8 saniye.")
    print(">>> Yine mili elinle cevirmeye calis.")
    outs["IN3"].on()
    outs["IN4"].off()
    time.sleep(8)
    outs["IN3"].off()
    print("bobin birakildi.")

    input("\nTEST 4: cok yavas 20 adim (300ms/adim). Enter'a bas...")
    seq = [(1, 0, 1, 0), (0, 1, 1, 0), (0, 1, 0, 1), (1, 0, 0, 1)]
    order = ["IN1", "IN2", "IN3", "IN4"]
    for i in range(20):
        for name, value in zip(order, seq[i % 4]):
            outs[name].value = value
        print(f"  adim {i+1}/20  faz={seq[i % 4]}")
        time.sleep(0.3)

finally:
    for out in outs.values():
        out.off()
        out.close()
    print("\nPinler serbest birakildi.")
