#!/usr/bin/env python3
import os
# Raspberry Pi 5 backend zorunluluğu
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'

from gpiozero import PWMOutputDevice
import time

# -- CONFIGURATION --------------------------------------------------
MOTOR_PIN = 12         # GPIO12 → physical pin 32
FREQ      = 50         # 50 Hz → 20 ms periyot

LOW_THROTTLE    = 0.05   # 1000 µs (Tam Geri)
FULL_THROTTLE   = 0.10   # 2000 µs (Tam İleri)
HALF_THROTTLE   = 0.075  # 1500 µs (Nötr / Durma)

# -------------------------------------------------------------------

def execute_dynamic_arm(motor_device):
    """
    Geliştirilmiş Reset ve Arm Mekanizması.
    Güç kesmeye gerek kalmadan ESC kilidini açmayı dener.
    """
    print("\n" + "="*40)
    print("[ESC SIFIRLANIYOR & ARM SEKANSI BAŞLADI]")

    # 1. ADIM: Donanımsal Reset (Kablo sök-tak simülasyonu)
    print("-> Sinyal tamamen kapatılıyor (0 V). ESC beyni sıfırlanıyor...")
    motor_device.value = 0
    time.sleep(2.5)  # ESC'nin sinyalsizliği anlayıp kendini boşa alması için süre

    # 2. ADIM: Standart Başlangıç Nötrü
    print(f"-> 1) Nötr Sinyali ({int(HALF_THROTTLE*20000)} µs) veriliyor...")
    motor_device.value = HALF_THROTTLE
    time.sleep(2)

    # 3. ADIM: Üst Limit Doğrulama
    print(f"-> 2) Üst Sınır ({int(FULL_THROTTLE*20000)} µs) gönderiliyor...")
    motor_device.value = FULL_THROTTLE
    time.sleep(2)

    # 4. ADIM: Alt Limit Doğrulama
    print(f"-> 3) Alt Sınır ({int(LOW_THROTTLE*20000)} µs) gönderiliyor...")
    motor_device.value = LOW_THROTTLE
    time.sleep(2)

    # 5. ADIM: Kilidi Açma (Unlock/Neutral)
    print(f"-> 4) Güvenli Durma Noktası ({int(HALF_THROTTLE*20000)} µs) kuruluyor...")
    motor_device.value = HALF_THROTTLE
    time.sleep(3.5)

    print("[ARM TAMAMLANDI] ESC'yi dinleyin. Düzenli bip sesleri gelmiş olmalı.")
    print("="*40 + "\n")

# Sinyali başlat
print(f"PWM Sinyali başlatılıyor... Pin: GPIO {MOTOR_PIN}, Frekans: {FREQ}Hz")
motor = PWMOutputDevice(MOTOR_PIN, frequency=FREQ, initial_value=0)

# İlk açılışta arm sekmesini çalıştır
execute_dynamic_arm(motor)

print("--- KONTROL PANELİ ---")
print("Komutlar:")
print("  - 1500 : Motoru Durdurur (Nötr)")
print("  - 2000 : Tam Güç İleri")
print("  - 1000 : Tam Güç Geri")
print("  - arm  : Kablo sökmeden ESC'yi sıfırlayıp baştan arm eder")
print("  - exit : Programı kapatır\n")

try:
    while True:
        user_input = input("Mikrosaniye veya Komut (1000 - 2000): ").strip().lower()

        if user_input == 'exit':
            print("Programdan çıkılıyor...")
            break

        if user_input == 'arm':
            execute_dynamic_arm(motor)
            continue

        try:
            us_val = int(user_input)

            if us_val < 1000 or us_val > 2000:
                print("HATA: Sadece 1000 ile 2000 arasında değer girebilirsiniz!")
                continue

            duty = us_val / 20000.0
            motor.value = duty
            print(f"Sinyal Gönderildi: {us_val} µs (Duty: %{duty*100:.2f})")

        except ValueError:
            print("HATA: Geçerli bir sayı, 'arm' veya 'exit' girin.")

except KeyboardInterrupt:
    print("\nKullanıcı tarafından kesildi.")

finally:
    # Kapanış güvenliği
    print("\nGÜVENLİK: Motor durduruluyor...")
    motor.value = HALF_THROTTLE
    time.sleep(0.5)
    motor.value = 0
    motor.close()
    print("Stopped, GPIO cleaned up.")
