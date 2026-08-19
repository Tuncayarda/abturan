import sys
from time import sleep
from gpiozero import PWMOutputDevice

# --- AYARLAR ---
# Raspberry Pi 5 üzerinde PWM destekleyen bir pin seçildi (GPIO 12)
SERVO_PIN = 12
FREKANS = 333  # Servo çalışma frekansı (Hz)

# 333 Hz için zamanlama hesaplamaları (Milisaniye -> Duty Cycle Oranı)
# Toplam periyot: 1 / 333 = 0.003003 saniye (3.003 ms)
PERIYOT_MS = 1000.0 / FREKANS

MIN_DARBE_MS = 0.5  # 0 derece için darbe genişliği
MAX_DARBE_MS = 2.5  # 180 derece için darbe genişliği

# Duty cycle (aktiflik oranı) hesaplama fonksiyonu
def aci_to_duty(aci):
    # Açıyı milisaniyeye haritala
    darbe_ms = MIN_DARBE_MS + (aci / 180.0) * (MAX_DARBE_MS - MIN_DARBE_MS)
    # Milisaniyeyi duty cycle oranına böl (0.0 ile 1.0 arası)
    return darbe_ms / PERIYOT_MS

# PWM Cihazını Başlat
try:
    servo = PWMOutputDevice(pin=SERVO_PIN, frequency=FREKANS)
except Exception as e:
    print(f"Hata: GPIO başlatılamadı. root yetkisi gerekebilir. Detay: {e}")
    sys.exit(1)

print("--- 333 Hz Servo Kontrol Programı ---")
print("Çıkış yapmak için 'q' yazın.\n")

# İlk konum: 90 derece (Merkez)
servo.value = aci_to_duty(90)

while True:
    try:
        girdi = input("Açı girin (0 - 180): ").strip()

        if girdi.lower() == 'q':
            print("Program kapatılıyor...")
            break

        aci = float(girdi)

        if 0 <= aci <= 180:
            duty = aci_to_duty(aci)
            servo.value = duty
            print(f"Servo {aci} derecesine ayarlandı. (Duty Cycle: {duty:.4f})")
        else:
            print("Hata: Lütfen 0 ile 180 arasında bir değer girin.")

    except ValueError:
        print("Hata: Geçersiz bir sayı veya komut girdiniz.")
    except KeyboardInterrupt:
        print("\nProgram sonlandırıldı.")
        break

# Temizlik
servo.close()
