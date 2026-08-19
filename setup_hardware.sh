#!/usr/bin/env bash
# Host tarafi donanim hazirligi: STM32 UART'i + IMU I2C'sini acar.
#
# Neden gerekli:
#   * IMU GPIO2/GPIO3'e bagli ama /dev/i2c-1 yok — config.txt icinde
#     Pi 5'te "dtoverlay=i2c1-pi5,pins_2_3" gerekli.
#   * STM32 GPIO14/GPIO15'e bagli. Pi 5'te bu UART /dev/ttyAMA0 ve
#     "dtoverlay=uart0-pi5" olmadan gorunmuyor. /dev/serial0 -> ttyAMA10 ise
#     Pi 5'in AYRI hata ayiklama basligidir; STM32 oraya bagli DEGIL.
#
# Kullanim:
#   ./setup_hardware.sh              # ne yapacagini gosterir, onay ister
#   ./setup_hardware.sh --yes        # onay sormaz
#   ./setup_hardware.sh --i2c-slow   # I2C'yi 50 kHz'e dusur (BNO08x saat germe)
#
# Degisiklikten sonra REBOOT gerekiyor.
set -euo pipefail

CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt      # eski Raspberry Pi OS
CMDLINE=/boot/firmware/cmdline.txt
[ -f "$CMDLINE" ] || CMDLINE=/boot/cmdline.txt

ASSUME_YES=0
I2C_SLOW=0
for arg in "$@"; do
  case "$arg" in
    --yes) ASSUME_YES=1 ;;
    --i2c-slow) I2C_SLOW=1 ;;
    *) echo "bilinmeyen secenek: $arg"; echo "kullanim: $0 [--yes] [--i2c-slow]"; exit 1 ;;
  esac
done

MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo bilinmiyor)"
echo "Kart      : $MODEL"
echo "config.txt: $CONFIG"
echo

# -- Hangi satirlar gerekli? ------------------------------------------------
if [[ "$MODEL" == *"Pi 5"* ]]; then
  # Pi 5'in RP1 uzerindeki denetleyicileri model-ozel overlay ister:
  # GPIO2/3 = I2C1, GPIO14/15 = UART0. Genel dtparam satirlari bazi yeni
  # kernel/firmware kombinasyonlarinda aygit dugumunu etkinlestirmiyor.
  WANTED=("dtoverlay=i2c1-pi5,pins_2_3" "dtoverlay=uart0-pi5")
  UART_DEV=/dev/ttyAMA0
else
  # Pi 4 ve oncesi: enable_uart=1 ile GPIO14/15 -> /dev/serial0
  WANTED=("dtparam=i2c_arm=on" "enable_uart=1")
  UART_DEV=/dev/serial0
fi

# BNO08x I2C'de saati uzun sure gerebiliyor ve Pi'nin donanim I2C'si bunu kotu
# yonetiyor. Bozuk paket / EREMOTEIO gorulurse hat hizini dusurmek gerekiyor.
# NOT: 50 kHz'e dusurunce imu.yaml icindeki rate_hz'i de dusurun (50 Hz),
# yoksa hat doluyor (bkz. README, IMU bolumu).
if [ "$I2C_SLOW" -eq 1 ]; then
  WANTED+=("dtparam=i2c_arm_baudrate=50000")
fi

MISSING=()
for line in "${WANTED[@]}"; do
  if grep -qE "^[[:space:]]*${line}[[:space:]]*$" "$CONFIG"; then
    echo "  [var]    $line"
  else
    echo "  [EKLENIR] $line"
    MISSING+=("$line")
  fi
done

# Yorumlanmis hali varsa bunu da soyleyelim; sadece ekleme yapmak yeterli
# (sonraki satir onceki yorumu zaten etkisizlestiriyor) ama kullanici
# dosyada iki yerde gormesin diye yorumlu satiri da acacagiz.
COMMENTED=()
for line in "${WANTED[@]}"; do
  if grep -qE "^[[:space:]]*#[[:space:]]*${line}[[:space:]]*$" "$CONFIG"; then
    COMMENTED+=("$line")
  fi
done

# -- Seri konsol cakismasi --------------------------------------------------
CONSOLE_WARN=""
if grep -q "console=${UART_DEV#/dev/}" "$CMDLINE" 2>/dev/null; then
  CONSOLE_WARN="cmdline.txt icinde 'console=${UART_DEV#/dev/}' var — cekirdek konsolu bu porta yaziyor, veri hatti olarak kullanilamaz."
fi
if systemctl is-enabled "serial-getty@${UART_DEV#/dev/}.service" >/dev/null 2>&1; then
  CONSOLE_WARN="${CONSOLE_WARN} serial-getty@${UART_DEV#/dev/} servisi acik — kapatilacak."
fi
[ -n "$CONSOLE_WARN" ] && { echo; echo "UYARI: $CONSOLE_WARN"; }

CONFIG_CHANGED=0
if [ ${#MISSING[@]} -gt 0 ] || [ ${#COMMENTED[@]} -gt 0 ]; then
  echo
  if [ "$ASSUME_YES" -ne 1 ]; then
    read -r -p "$CONFIG dosyasi degistirilecek (yedegi alinir). Devam? [e/H] " reply
    case "$reply" in
      e|E|y|Y) ;;
      *) echo "Iptal edildi."; exit 1 ;;
    esac
  fi

  BACKUP="${CONFIG}.bak.$(date +%Y%m%d-%H%M%S)"
  sudo cp "$CONFIG" "$BACKUP"
  echo "Yedek: $BACKUP"

  # Yorumlu satirlari ac
  for line in "${COMMENTED[@]}"; do
    escaped="${line//\//\\/}"
    sudo sed -i "s/^[[:space:]]*#[[:space:]]*${escaped}[[:space:]]*$/${escaped}/" "$CONFIG"
    echo "Yorum kaldirildi: $line"
    CONFIG_CHANGED=1
  done

  # Eksik satirlari ekle (yorumlu hali acildiysa artik eksik olmayabilir)
  for line in "${MISSING[@]}"; do
    if ! grep -qE "^[[:space:]]*${line}[[:space:]]*$" "$CONFIG"; then
      printf '\n# up_robot: %s\n%s\n' "STM32 UART / IMU I2C icin eklendi" "$line" \
        | sudo tee -a "$CONFIG" >/dev/null
      echo "Eklendi: $line"
      CONFIG_CHANGED=1
    fi
  done
else
  echo
  echo "config.txt zaten hazir."
fi

# Kernel seri konsolu ayni UART'a yazarsa ikili protokole boot/log baytlari
# karisir. Tam eslesen console=<aygit>,<baud> tokenlarini cmdline'dan kaldir.
if grep -qE "(^|[[:space:]])console=${UART_DEV#/dev/}(,[^[:space:]]+)?([[:space:]]|$)" "$CMDLINE" 2>/dev/null; then
  CMDLINE_BACKUP="${CMDLINE}.bak.$(date +%Y%m%d-%H%M%S)"
  sudo cp "$CMDLINE" "$CMDLINE_BACKUP"
  sudo sed -i -E "s/(^|[[:space:]])console=${UART_DEV#/dev/}(,[^[:space:]]+)?([[:space:]]|$)/\\1\\3/g; s/[[:space:]]+/ /g; s/^ //; s/ $//" "$CMDLINE"
  echo "Kernel seri konsolu kaldirildi (yedek: $CMDLINE_BACKUP)"
  CONFIG_CHANGED=1
fi

# Seri konsolu kapat (varsa)
if systemctl is-enabled "serial-getty@${UART_DEV#/dev/}.service" >/dev/null 2>&1; then
  sudo systemctl disable --now "serial-getty@${UART_DEV#/dev/}.service" || true
  echo "Kapatildi: serial-getty@${UART_DEV#/dev/}"
fi

# Host tarafindaki tools/ scriptleri icin gerekli paketler. (Konteyner
# kendi python3-serial'ini Dockerfile'dan aliyor, bu host icin.)
NEED_PKGS=()
python3 -c 'import serial' 2>/dev/null || NEED_PKGS+=(python3-serial)
command -v i2cdetect >/dev/null 2>&1 || NEED_PKGS+=(i2c-tools)
if [ ${#NEED_PKGS[@]} -gt 0 ]; then
  echo "Kuruluyor: ${NEED_PKGS[*]}"
  sudo apt-get install -y "${NEED_PKGS[@]}" || \
    echo "UYARI: ${NEED_PKGS[*]} kurulamadi — tools/ scriptleri calismaz."
fi

# Kullaniciyi gruplara ekle (konteyner privileged calisiyor ama host
# tarafindaki tools/ scriptleri icin lazim)
for grp in i2c dialout; do
  if getent group "$grp" >/dev/null && ! id -nG "$USER" | tr ' ' '\n' | grep -qx "$grp"; then
    sudo usermod -aG "$grp" "$USER"
    echo "$USER kullanicisi '$grp' grubuna eklendi (yeni oturumda etkin)."
  fi
done

echo
if [ "$CONFIG_CHANGED" -eq 1 ]; then
  echo "Bitti. Yapilandirma degisti; REBOOT gerekiyor:  sudo reboot"
else
  echo "Bitti. Yapilandirma degisikligi yok; reboot gerekmiyor."
fi
echo "Sonrasinda kontrol:"
echo "  ls -l $UART_DEV /dev/i2c-1"
echo "  i2cdetect -y 1                  # IMU (BNO08x) 0x4A veya 0x4B gorunmeli"
echo
echo "BNO08x bozuk paket/EREMOTEIO veriyorsa saat germesi (clock stretching)"
echo "yuzundendir: config.txt icine dtparam=i2c_arm_baudrate=50000 ekleyin."
echo "  python3 tools/imu_test.py"
echo "  python3 tools/stm32_link_test.py --port $UART_DEV"
