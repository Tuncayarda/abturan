# UP Robot — Raspberry Pi 5 + STM32

Raspberry Pi 5 (Raspberry Pi OS, Debian trixie) üzerinde Docker içinde çalışan
ROS 2 Humble kontrol düzlemi ve UART'la bağlı bir STM32 eyleyici kartı.

**Görev paylaşımı:**

| | Raspberry Pi 5 | STM32 |
|---|---|---|
| Ne yapar | kamera (SRT), IMU (I2C), ROS/rosbridge, kinematik | 6 ESC PWM, step motor, LED |
| Neden | libcamera host'a bağlı; ağ ve ROS burada | µs hassasiyetli darbe zamanlaması |

Pi'nin GPIO'suna **hiçbir eyleyici bağlı değil**. Linux userspace'te 1500 µs'lik
ESC darbesini ±10 µs tutmak ya da step motoru titremesiz sürmek garanti
edilemiyor (ölçümler ve gerekçe: `step_control.py` içindeki jitter notları).
Aynı iş STM32'de donanım timer'ına bırakıldı.

---

## Mimari

```
┌──────────────────────── Raspberry Pi 5 (host) ────────────────────────┐
│                                                                         │
│  VERİ DÜZLEMİ (host'ta):                                                │
│    rpicam-vid → H264 → mpegtsmux → srtsink (srt://:9003, LISTENER)      │
│    └─ scripts/rpi_cam_streamer.py                                       │
│         │                                                               │
│         ▼  (tek client SRT caller olarak bağlanır, görüntüyü çeker)     │
│   srt://<raspi_ip>:9003                                                 │
│                                                                         │
│         ▲ params.json (ayarlar)   │ stats.json (throughput)            │
│         │                          ▼                                    │
│   ┌─────┴──── cam_ctrl/ (paylaşılan klasör) ────────┐                  │
│   │                                                   │                  │
│  KONTROL DÜZLEMİ (Docker container'ında):            │                  │
│   ┌───────────────────────────────────────────────────────────────┐   │
│   │ rosbridge_websocket :9090 + rosapi   ← arayüz buraya bağlanır  │   │
│   │ front_cam_node : kamera parametreleri + throughput             │   │
│   │ imu_node       : /dev/i2c-1 → /imu/data_raw, /imu/data         │   │
│   │ joy_to_wrench → thruster_allocator : joystick → 6 ESC PWM      │   │
│   │ stm32_bridge   : /dev/ttyAMA0 → STM32 (ikili çerçeve + CRC)    │   │
│   └───────────────────────────────────────────────────────────────┘   │
└───────────────┬────────────────────────────────┬────────────────────────┘
        I2C     │                        UART    │  115200 8N1
     GPIO2/3    │                    GPIO14/15   │
                ▼                                ▼
          ┌──────────┐                    ┌──────────────┐
          │   IMU    │                    │    STM32     │
          │ BNO08x   │                    │  6×ESC PWM   │
          └──────────┘                    │  step motor  │
                                          │  LED         │
                                          └──────────────┘
```

**Önemli:** Görüntü ROS'tan GEÇMEZ. Video doğrudan host'tan SRT ile paylaşılır;
ROS sadece kamera parametrelerini kontrol eder ve throughput'u raporlar.
Yani video, ROS/container çökse bile akmaya devam eder.

Kontrol zinciri:

```
/ui/joy_cmd_vel  →  joy_to_wrench  →  /control/wrench
                 →  thruster_allocator  →  /control/pwm_cmds  (6 kanal, 0-1000)
                 →  stm32_bridge  →  UART  →  STM32  →  ESC 1-6
```

---

## Paketler

| Paket | Tür | Görevi |
|-------|-----|--------|
| `up_bringup` | ament_cmake | Ana launch dosyası (her şeyi başlatır) |
| `rpi_cam_bridge` | ament_python | `rpi_cam_node` (kamera kontrol + throughput) ve host streamer scripti |
| `joy_motor_pkg` | ament_python | joystick → wrench → **6 ESC** PWM dağıtımı |
| `stm32_bridge` | ament_python | ROS ↔ STM32 UART köprüsü (ESC, step motor, LED) |
| `imu_bridge` | ament_python | BNO08x (SHTP/SH-2) → `sensor_msgs/Imu` |

STM32 firmware'i ayrı: [`stm32_fw/`](stm32_fw/) (CubeMX projesine eklenecek
4 dosya + testler). Protokol iki tarafta da tanımlı ve **birlikte** değişmeli:
`stm32_fw/Inc/up_protocol.h` ↔ `src/stm32_bridge/stm32_bridge/protocol.py`.

---

## İlk kurulum (bir kez)

### 1) Donanım arayüzlerini aç

STM32 GPIO14/15'e, IMU GPIO2/3'e bağlı. İkisi de Raspberry Pi OS'ta
**varsayılan olarak kapalı**:

```bash
./setup_hardware.sh          # ne yapacağını gösterir, onay ister
sudo reboot
```

Script `/boot/firmware/config.txt` dosyasını yedekleyip şunları ekler:

| Satır | Ne için |
|-------|---------|
| `dtoverlay=uart0-pi5` | Pi 5 GPIO14/15 UART'ı → **`/dev/ttyAMA0`** (STM32) |
| `dtoverlay=i2c1-pi5,pins_2_3` | Pi 5 GPIO2/3 I2C'si → `/dev/i2c-1` (IMU) |
| `dtparam=i2c_arm_baudrate=50000` | *(gerekirse)* BNO08x saat germesi için I2C'yi yavaşlat |

> **Pi 5 tuzağı:** `/dev/serial0` bu kartta `ttyAMA10`'a, yani **ayrı hata
> ayıklama başlığına** bakıyor — GPIO14/15'e değil. STM32 için doğru aygıt
> `/dev/ttyAMA0`. Pi 5'te genel `dtparam=uart0=on` yerine model-özel
> `uart0-pi5` overlay'i kullanılır. Eski Pi'lerde durum tersidir
> (`enable_uart=1` + `/dev/serial0`);
> script kart modelini kendisi ayırt ediyor.

Kontrol:

```bash
ls -l /dev/ttyAMA0 /dev/i2c-1
i2cdetect -y 1                  # IMU 0x4A (veya 0x4B) görünmeli
```

### 2) Docker + host paketleri

```bash
# Docker (kurulu değilse)
sudo apt-get install -y docker.io docker-compose
sudo usermod -aG docker pi          # sonra bir kez logout/login (veya reboot)
sudo systemctl enable --now docker

# Host kamera streamer'ı için gerekli (gstreamer + python bindings)
sudo apt-get install -y python3-gi gir1.2-gstreamer-1.0 \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
# rpicam-apps Raspberry Pi OS'ta zaten kurulu olmalı (rpicam-hello --list-cameras)
```

`python3-serial` ve `i2c-tools`'u `setup_hardware.sh` kuruyor (host'taki
`tools/` scriptleri için). Container kendi `python3-serial`'ını Dockerfile'dan
alıyor.

---

## Kullanım

### Yöntem 1 — Otomatik (önerilen): `./start.sh`

```bash
cd ~/ros2_ws

./start.sh                          # port 9003, 640x480@30, 1Mbit/s, /dev/ttyAMA0
./start.sh 9003 1280 720 30 2000000 # port w h fps bitrate
STM32_PORT=/dev/ttyUSB0 ./start.sh  # STM32 USB-TTL ile bağlıysa
```

Çıktı:

```
[start] Video (SRT listener): srt://<raspi_ip>:9003   ← arayüz buraya bağlanır
[start] rosbridge           : ws://<raspi_ip>:9090
[start] STM32 UART          : /dev/ttyAMA0 @ 115200
```

Aygıtlar eksikse uyarır ama sistem yine kalkar (köprüler yeniden dener).
Durdurmak için **Ctrl+C**.

### Yöntem 2 — Manuel (container içinden launch)

```bash
cd ~/ros2_ws

# 1) Host kamera streamer'ını ayrı bir terminalde başlat (host'ta kalmalı!)
python3 src/rpi_cam_bridge/scripts/rpi_cam_streamer.py --ctrl-dir ./cam_ctrl

# 2) Container'a kabuk olarak gir (entrypoint ROS'u source eder + build eder)
docker compose run --rm ros2 bash

# 3) Launch:
ros2 launch up_bringup bringup.launch.py
ros2 launch up_bringup bringup.launch.py stm32_port:=/dev/ttyUSB0 stm32_baud:=115200
```

Çalışan sisteme komut göndermek için `./shell.sh` (veya
`docker exec -it ros2_humble bash`).

---

## Konular (topics)

**Eyleyici — yazılır**

| Konu | Tip | Anlamı |
|------|-----|--------|
| `/ui/joy_cmd_vel` | `sensor_msgs/Joy` | arayüzden joystick |
| `/control/wrench` | `geometry_msgs/Wrench` | istenen kuvvet/tork |
| `/control/pwm_cmds` | `std_msgs/Int32MultiArray` | 6 kanal, 0-1000 (500 = nötr) |
| `/control/led` | `std_msgs/Bool` | LED (STM32 PB12) |
| `/control/stepper/velocity` | `std_msgs/Float32` | adım/s, işaretli |
| `/control/stepper/position` | `std_msgs/Int32` | mutlak hedef adım |
| `/control/stepper/enable` | `std_msgs/Bool` | `false` → bobinleri bırak |

**Okunur**

| Konu | Tip | Anlamı |
|------|-----|--------|
| `/imu/data_raw` | `sensor_msgs/Imu` | ham ivme + açısal hız |
| `/imu/data` | `sensor_msgs/Imu` | + BNO08x füzyon yönelimi (quaternion) |
| `/imu/mag` | `sensor_msgs/MagneticField` | manyetik alan (`publish_mag: true` ise) |
| `/stm32_bridge_node/status` | `std_msgs/String` | STM32 durumu, JSON |
| `/stm32_bridge_node/link_ok` | `std_msgs/Bool` | STATUS taze mi |
| `/stm32_bridge_node/stepper_position` | `std_msgs/Int32` | STM32'nin adım sayacı |
| `/front_cam_node/throughput_kbps` | `std_msgs/Float64` | canlı throughput |
| `/front_cam_node/streaming` | `std_msgs/Bool` | yayın aktif mi |

Elle deneme (container içinden):

```bash
ros2 topic pub --once /control/led std_msgs/Bool "{data: true}"
ros2 topic pub --once /control/stepper/velocity std_msgs/Float32 "{data: 200.0}"
ros2 topic pub --once /control/stepper/enable std_msgs/Bool "{data: false}"
ros2 topic echo /stm32_bridge_node/status
ros2 topic echo /imu/data_raw
```

---

## STM32 bağlantısı

| Raspberry Pi | Fiziksel pin | STM32 |
|--------------|-------------:|-------|
| GPIO14 / TXD | 8 | PA3 / USART2_RX |
| GPIO15 / RXD | 10 | PA2 / USART2_TX |
| GND | 6 | STM32 GND |

TX ↔ RX **çaprazlanır**, GND ortak olmalı. Pin haritası, CubeMX ayarları,
protokol ve **step motor akım uyarısı** için: [`stm32_fw/README.md`](stm32_fw/README.md).

### Güvenlik davranışı

- **Arm:** STM32 açılışta 2 s boyunca ESC'lere nötr verir, sonra komut uygular.
- **Failsafe:** 500 ms komut gelmezse ESC'ler nötre, step motor bobinleri
  serbeste düşer. Kablo çıkması / Pi kapanması / ROS durması aynı yola çıkar.
- Pi tarafı da `/control/pwm_cmds` 0.5 s susarsa nötr göndermeye başlar,
  ayrıca 5 Hz heartbeat yayar.
- Köprü kapanırken (Ctrl+C) nötr + `idle` + LED kapalı gönderir.

### Kart olmadan test

```bash
python3 tools/test_link_e2e.py        # protokol + arm + failsafe (sanal STM32)

# veya elle: sanal bir STM32 açıp ROS köprüsünü ona bağla
python3 tools/fake_stm32.py --link /tmp/ttySTM32 -v
ros2 run stm32_bridge stm32_bridge --ros-args -p serial_port:=/tmp/ttySTM32
```

### Gerçek kartla test (ROS'suz)

```bash
python3 tools/stm32_link_test.py --port /dev/ttyAMA0
```

```
stm32> status            # STATUS geliyor mu, armed mı
stm32> led on
stm32> esc all 1600      # PERVANELERİ ÇIKAR
stm32> stop
stm32> step v 200
stm32> step idle
stm32> q
```

Arka planda 50 Hz ESC çerçevesi + heartbeat gönderir, yani gerçek koşulları
taklit eder (failsafe tam bunun kesilmesine bakıyor).

---

## IMU

| Raspberry Pi | Fiziksel pin | IMU |
|--------------|-------------:|-----|
| GPIO2 / SDA | 3 | SDA |
| GPIO3 / SCL | 5 | SCL |
| 3.3 V | 1 | VCC |
| GND | 9 | GND |

IMU **BNO08x** (BNO085/BNO086). MPU ailesinden farkı: register haritası yok,
üzerinde kendi füzyon yazılımı (SH-2) çalışıyor ve haberleşme paket tabanlı
(**SHTP**). Sürücü `src/imu_bridge/imu_bridge/bno08x.py` içinde, harici
bağımlılık yok — aynı `i2c.py` ioctl sarmalayıcısını kullanıyor.

Adres ADR/PS0 pini GND'de `0x4A`, VCC'de `0x4B`.

> **Kart hiç cevap vermiyorsa ilk buraya bakın — PS0/PS1 (protokol seçimi).**
> BNO08x üç arayüz konuşabiliyor ve hangisini kullanacağını bu iki pin seçiyor:
>
> | PS1 | PS0 | Arayüz |
> |-----|-----|--------|
> | 0 | 0 | **I2C** ← bize gereken |
> | 0 | 1 | UART-RVC |
> | 1 | 0 | UART |
> | 1 | 1 | SPI |
>
> Adafruit/SparkFun breakout'larında ikisi de varsayılan olarak GND'ye çekili
> (I2C), ama ucuz "BNO085 modül" kartlarının bir kısmı **UART-RVC** modunda
> geliyor — o durumda I2C adresini hiç ACK'lemez, `imu_test.py --scan` boş
> döner ve sorunu kablolamada aramakla vakit kaybedilir. Ayrıca bazı kartlarda
> PS0 aynı zamanda adres seçimi olduğu için (0x4A/0x4B) ikisi karışabiliyor —
> kartın şemasına bakın.
>
> `RST` pini varsa Pi'nin bir GPIO'suna bağlamak faydalı: sürücünün yumuşak
> reseti (executable kanalı) yonga zaten konuşuyorken işe yarıyor, tamamen
> kilitlendiğinde donanım reseti gerekiyor.

Yönelim füzyonu yongada: `/imu/data` doğrudan quaternion taşıyor, ROS
tarafında tümleyici filtre yok. Manyetometre füzyona dahil olduğu için
**yaw kaymıyor** (MPU6050 kurulumunun bilinen sorunuydu).

Ayarlar: `src/imu_bridge/config/imu.yaml`.

| Parametre | Anlamı |
|-----------|--------|
| `orientation_mode` | `rotation_vector` (9 eksen, yaw kaymaz) / `game_rotation_vector` (manyetometresiz, gürültüye bağışık ama yaw kayar) / `geomagnetic_rotation_vector` / `none` |
| `rate_hz` | yonganın rapor akış frekansı |
| `publish_mag` | `/imu/mag` yayınla |
| `use_reported_accuracy` | yönelim kovaryansını yonganın kendi doğruluk kestiriminden al |
| `soft_reset_on_start` | açılışta yumuşak reset |

```bash
python3 tools/imu_test.py --scan                       # hattı tara, adresi bul
python3 tools/imu_test.py                              # canlı okuma
python3 tools/imu_test.py --mode game_rotation_vector  # manyetometresiz dene
python3 tools/imu_test.py --mag                        # manyetik alanı da göster
```

**Bilinmesi gerekenler:**

- **Saat germe (clock stretching):** BNO08x I2C'de saati uzun süre gerebiliyor,
  Pi'nin donanım I2C'si (BSC) bunu bilinen şekilde kötü yönetiyor. Bozuk paket
  veya `EREMOTEIO` görülürse `/boot/firmware/config.txt` içine
  `dtparam=i2c_arm_baudrate=50000` ekleyip yeniden başlatın.
- **Manyetometre kalibrasyonu** gerekiyor: yonga doğruluk seviyesini
  (`unreliable`/`low`/`medium`/`high`) her raporda bildiriyor, node bu değer
  değiştikçe logluyor. `low` görürseniz aracı havada 8 çizer gibi birkaç saniye
  çevirin. Motor mıknatısları yüzünden hiç düzelmiyorsa
  `orientation_mode: game_rotation_vector` kullanın — yaw yine kayar ama
  manyetik gürültüden etkilenmez.
- **Sıcaklık konusu yok:** BNO08x sıcaklık raporu sunmuyor, `/imu/temperature`
  kaldırıldı.
- Açılışta gyro bias ölçümü **yok** — kalibrasyonu yonga kendisi yapıyor, araç
  açılışta hareketsiz olmak zorunda değil.
- Yönelim istemiyorsan `/imu/data_raw`'ı kullan (orientation_covariance[0] = -1,
  yani "yönelim yok" demek) veya `orientation_mode: none` ile tamamen kapat.
- Yonga kendini resetlerse (besleme dalgalanması) node bunu algılayıp raporları
  otomatik geri açıyor; logda "BNO08x resetlendi" uyarısı görünür.
- **Eksen yönleri (REP-103) kontrol edilmeli.** Yonga quaternion'u kendi gövde
  eksenlerine göre veriyor; node bunu olduğu gibi `imu_link` altında
  yayınlıyor, herhangi bir eksen çevirme (remap) yapmıyor. ROS'ta beklenen
  düzen **x ileri, y sola, z yukarı**. Kartı buna uyacak şekilde monte edin;
  uymuyorsa ya kart yönünü değiştirin ya da `imu_link` ile gövde çerçevesi
  arasına sabit bir dönüşüm (`static_transform_publisher`) koyun. Doğrulaması
  kolay: `python3 tools/imu_test.py` ile aracı ileri eğin — **pitch** artmalı;
  sağa yatırın — **roll** artmalı. Ters işaret çıkıyorsa montaj yönü yanlış.
- **I2C hızını 50 kHz'e düşürdüyseniz `rate_hz`'i de düşürün.** Kabaca hesap:
  accel + gyro + rotation vector aynı pakette ~43 bayt, sürücü paket başına iki
  okuma yapıyor (başlık + tamamı) → ~440 bit. 100 Hz'de ~44 kbit/s, yani
  50 kHz'lik bir hatta **%88 doluluk** — saat germe payı bile kalmıyor ve
  sürücü geride kalır. 50 kHz'te `rate_hz: 50` (≈%44) güvenli; 400 kHz'te
  100 Hz sorun değil (%11).

---

## Kamera parametrelerini değiştirme (runtime)

Arayüz bunu rosbridge üzerinden yapar; manuel test için container içinden:

```bash
ros2 param set /front_cam_node bitrate 3000000
ros2 param set /front_cam_node width  1280
ros2 param set /front_cam_node height 720
ros2 param set /front_cam_node fps    30
ros2 param set /front_cam_node port   9003
ros2 param set /front_cam_node enabled false   # yayını durdur / true → başlat
```

Her değişiklikte `front_cam_node`, `cam_ctrl/params.json`'a yazar; host streamer
bunu görüp yayını yeni ayarlarla yeniden başlatır.

### Görüntüyü izleme (client tarafı)

```bash
ffplay srt://<raspi_ip>:9003

gst-launch-1.0 srtsrc uri="srt://<raspi_ip>:9003?mode=caller" ! \
    tsdemux ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

Format: **H264 / MPEG-TS over SRT**. Host 9003'te **dinler** (listener), tek
client caller olarak bağlanır — IP girilmez.

---

## Joystick → 6 ESC

```
/ui/joy_cmd_vel → joy_to_wrench → /control/wrench → thruster_allocator
                → /control/pwm_cmds → stm32_bridge → UART → STM32
```

Ayarlar: `src/joy_motor_pkg/config/joy_wrench_allocator.yaml`.

Dağıtım matrisi 6×6; kolonlar **ESC1..ESC6 = PB0, PB1, PB6, PB7, PB8, PB9**:

| | ESC1 | ESC2 | ESC3 | ESC4 | ESC5 | ESC6 |
|---|---|---|---|---|---|---|
| Fx (surge) | 1 | 1 | 0 | 0 | 0 | 0 |
| Fy (sway) | 0 | 0 | 0 | 0 | 0 | 0 |
| Fz (heave) | 0 | 0 | 1 | 1 | 1 | 1 |
| Tx (roll) | 0 | 0 | 1 | −1 | 1 | −1 |
| Ty (pitch) | 0 | 0 | 1 | 1 | −1 | −1 |
| Tz (yaw) | −1 | 1 | 0 | 0 | 0 | 0 |

ESC1-2 yatay (surge + yaw), ESC3-6 dikey (heave + roll + pitch).

> **Fy satırı sıfır:** bu yerleşimde yana kayma (sway) yetkisi yok. Joystick'ten
> sway gelse bile matris onu iticiye dağıtamaz. Yana hareket isteniyorsa yatay
> iticilerin vektörel (45°) yerleştirilmesi ve bu satırın doldurulması gerekir.

`/control/pwm_cmds` aralığı (0-1000, 500 nötr) `stm32_bridge` tarafından
1000-2000 µs darbeye çevrilir. Pervane ters bağlıysa YAML'de
`esc_reverse: [false, true, ...]` ile kanal bazında çevirebilirsin.

---

## Portlar

| Port | Ne | Protokol |
|------|-----|----------|
| 9003 | Kamera video yayını (listener) | SRT / H264 MPEG-TS |
| 9090 | rosbridge websocket (arayüz kontrol) | WebSocket |

---

## Dosya yapısı

```
ros2_ws/
├── start.sh                 # TEK komut: host streamer + container
├── setup_hardware.sh        # UART + I2C'yi aç (bir kez, reboot gerekir)
├── shell.sh                 # çalışan container'a kabuk
├── docker-compose.yml       # container tanımı (host network, privileged, cam_ctrl)
├── Dockerfile               # ros:humble-ros-base + rosbridge + pyserial
├── entrypoint.sh            # source + colcon build + ros2 launch
├── cam_ctrl/                # paylaşılan klasör (params.json / stats.json)
├── src/
│   ├── up_bringup/          # bringup.launch.py (hepsini başlatır)
│   ├── rpi_cam_bridge/
│   │   ├── rpi_cam_bridge/rpi_cam_node.py     # ROS kontrol node'u
│   │   └── scripts/rpi_cam_streamer.py        # HOST kamera streamer'ı
│   ├── joy_motor_pkg/       # joystick → wrench → 6 ESC PWM
│   ├── stm32_bridge/
│   │   ├── stm32_bridge/protocol.py           # UART çerçevesi (Python tarafı)
│   │   └── stm32_bridge/stm32_bridge_node.py  # ROS ↔ UART köprüsü
│   └── imu_bridge/
│       ├── imu_bridge/i2c.py                  # /dev/i2c-N ioctl sarmalayıcı
│       ├── imu_bridge/bno08x.py                # BNO08x SHTP/SH-2 sürücüsü
│       └── imu_bridge/imu_node.py             # ROS node'u
├── stm32_fw/                # STM32 firmware (CubeMX'e eklenecek dosyalar)
│   ├── Inc/up_protocol.h    Src/up_protocol.c   # çerçeveleme + CRC
│   ├── Inc/up_app.h         Src/up_app.c        # ESC/step/LED/failsafe
│   ├── test_protocol.c      test/hal_stub.h     # masaüstü testleri
│   └── README.md            # CubeMX ayarları + akım uyarısı
├── tools/
│   ├── stm32_link_test.py   # gerçek kartı ROS'suz sür
│   ├── fake_stm32.py        # sanal STM32 (kart olmadan geliştirme)
│   ├── test_link_e2e.py     # protokol/arm/failsafe testi
│   └── imu_test.py          # IMU ham okuma + I2C tarama
├── step_control.py          # ESKİ: Pi GPIO'sundan step motor (artık STM32'de)
├── motor_kontrol.py         # ESKİ: Pi GPIO'sundan tek ESC testi
├── servo_kontrol.py         # ESKİ: Pi GPIO'sundan servo testi
├── gpio_test.py             # ESKİ: L298N tanı testi
└── micro_test.py            # ESKİ: mikroadım/ENA-ENB tanı testi
```

**ESKİ** işaretli kök dizin scriptleri Pi'nin GPIO'sunu doğrudan sürüyor.
Eyleyiciler STM32'ye taşındığı için üretimde kullanılmıyorlar; motor/sürücü
tanısı için referans olarak duruyor. Çalıştırmadan önce STM32'nin aynı
donanıma bağlı olmadığından emin olun — iki sürücü aynı pinleri sürerse
çakışırlar.

---

## Sorun giderme

**STM32'den STATUS gelmiyor (`link_ok: false`)**
```bash
ls -l /dev/ttyAMA0                   # aygıt var mı? yoksa ./setup_hardware.sh
python3 tools/stm32_link_test.py --port /dev/ttyAMA0   # status yaz
```
- `/dev/ttyAMA0` yok → Pi 5'te `dtoverlay=uart0-pi5` eksik (reboot gerekli).
- Aygıt var, STATUS yok → **RX yönü**: STM32 PA2 → Pi GPIO15 (pin 10).
  TX/RX çaprazlanmamış olabilir.
- Çerçeve hatası çok (`rx_frame_errors` artıyor) → baud uyuşmuyor veya GND ortak değil.
- ESC'ler nötrde kalıyor → `failsafe` true mu, `esc_armed` true mu (`status`).

**IMU okunmuyor**
```bash
ls -l /dev/i2c-1
i2cdetect -y 1                       # 0x4A / 0x4B görünmeli
python3 tools/imu_test.py --scan
```
- Adres `0x4B` çıktıysa `imu.yaml` içinde `i2c_address: 75` yap.
- Cihaz görünüyor ama "BNO08x cevap vermedi" ya da sürekli `EREMOTEIO` →
  **saat germe**. `/boot/firmware/config.txt` içine
  `dtparam=i2c_arm_baudrate=50000` ekleyip yeniden başlat.
- Yayın var ama yönelim saçmalıyor / doğruluk `low` → manyetometre kalibre
  değil. Aracı havada 8 çizer gibi birkaç saniye çevirin, ya da manyetik
  gürültü yüzündense `orientation_mode: game_rotation_vector`'a geçin.

**Kamera görüntüsü gelmiyor:**
```bash
rpicam-hello --list-cameras          # kamera görünüyor mu?
ps aux | grep rpicam-vid             # streamer rpicam'i başlatmış mı?
cat cam_ctrl/stats.json              # streaming:true ve throughput > 0 mı?
```
Yalnızca bir `rpicam-vid` çalışabilir — eski bir test süreci kamerayı tutuyorsa
`pkill -x rpicam-vid` ile kapat.

**Container build/launch logları:**
```bash
docker compose logs -f               # veya: docker logs -f ros2_humble
```

**rosbridge bağlantısı:**
```bash
ss -ltnp | grep 9090                 # 9090 dinleniyor mu?
```

**`docker` komutu "permission denied":** docker grubuna eklendikten sonra bir kez
logout/login yapmadıysan; `sg docker -c "docker ..."` ile geçici olarak çalıştır
(start.sh bunu otomatik halleder).

**Yeni paket görünmüyor** (`Package 'stm32_bridge' not found`): container'ın
`install/` dizini eski. Yeniden build et:
```bash
docker compose run --rm ros2 bash -c "colcon build --symlink-install"
```

**Temiz baştan başlatma:**
```bash
pkill -x rpicam-vid; docker compose down
./start.sh
```
