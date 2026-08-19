# STM32 eyleyici kartı — firmware

Raspberry Pi'nin UART üzerinden sürdüğü eyleyici kartı: **6 ESC PWM**,
**4 telli step motor** (17HS3401) ve **LED**.

Bu klasör hazır bir proje değil, **CubeMX projesine eklenecek 4 dosya**:

| Dosya | İş |
|-------|-----|
| `Inc/up_protocol.h`, `Src/up_protocol.c` | UART çerçeveleme + CRC (HAL'sız, PC'de de derlenir) |
| `Inc/up_app.h`, `Src/up_app.c` | ESC PWM, step motor, LED, failsafe, STATUS |
| `test_protocol.c` | masaüstü protokol testi (firmware'e girmez) |
| `test/hal_stub.h` | masaüstü sözdizimi kontrolü için sahte HAL (firmware'e girmez) |

Python tarafı: [`src/stm32_bridge/stm32_bridge/protocol.py`](../src/stm32_bridge/stm32_bridge/protocol.py).
**Protokol değişirse iki dosya birlikte değişmeli.**

---

## Pin haritası

| Görev | Pin | Donanım işlevi |
|-------|-----|----------------|
| ESC 1 PWM | PB0 | TIM3_CH3 |
| ESC 2 PWM | PB1 | TIM3_CH4 |
| ESC 3 PWM | PB6 | TIM4_CH1 |
| ESC 4 PWM | PB7 | TIM4_CH2 |
| ESC 5 PWM | PB8 | TIM4_CH3 |
| ESC 6 PWM | PB9 | TIM4_CH4 |
| Step IN1 | PA4 | GPIO Output |
| Step IN2 | PA5 | GPIO Output |
| Step IN3 | PA6 | GPIO Output |
| Step IN4 | PA7 | GPIO Output |
| LED | PB12 | GPIO Output |
| Pi'ye TX | PA2 | USART2_TX |
| Pi'den RX | PA3 | USART2_RX |
| (dahili) step zaman tabanı | — | TIM2, pin kullanmıyor |

Bu proje **STM32F401RCT6 (LQFP64)** içindir. Bu modelde bütün PWM çıkışları
**AF2**, USART2 pinleri **AF7** olarak seçilir; pinlerin tamamı LQFP64 pakette
mevcuttur. F1'deki alternatif fonksiyon remap mekanizması burada yoktur;
CubeMX her pin için aşağıdaki AF'yi üretmelidir.

| Pinler | CubeMX modu |
|--------|-------------|
| PB0, PB1 | `TIM3_CH3`, `TIM3_CH4` / AF2 |
| PB6..PB9 | `TIM4_CH1..CH4` / AF2 |
| PA2, PA3 | `USART2_TX`, `USART2_RX` / AF7 |
| PA4..PA7, PB12 | `GPIO_Output` |

### Kablolama (Pi ↔ STM32)

```
Pi GPIO14 / TXD (pin 8)  ─────────►  PA3  (USART2_RX)
Pi GPIO15 / RXD (pin 10) ◄─────────  PA2  (USART2_TX)
Pi GND          (pin 6)  ──────────  STM32 GND      ← ortak toprak ŞART
```

TX ile RX **çaprazlanır**. İkisi de 3.3 V mantık seviyesi, seviye çevirici
gerekmiyor. STM32'yi Pi'nin 3.3 V pininden beslemeyin — motor/ESC akımı
Pi'nin regülatörünü zorlar; STM32'ye ayrı 5 V verip yalnızca GND'yi ortaklayın.

---

## CubeMX ayarları

**Saat:** F401RCT6 için önerilen CubeMX kurulumu SYSCLK/HCLK = **84 MHz**,
APB1 = **42 MHz** (`/2`) şeklindedir. TIM2/3/4 saatleri donanım tarafından
APB1'in iki katına, yani **84 MHz**'e çıkar. Kod
`HAL_RCC_GetPCLK1Freq()` ve APB1 prescaler alanından timer saatini hesapladığı
için PSC değerlerini otomatik olarak 1 MHz sayaç frekansına ayarlar.

**TIM3 — ESC 1-2**
- Mode: `PWM Generation CH3`, `PWM Generation CH4`
- Prescaler / Counter Period: **ne yazarsan yaz**, `up_app_init()` bunları
  kod içinde 1 MHz sayaç + 20000 tick (50 Hz) olacak şekilde eziyor.

**TIM4 — ESC 3-6**
- Mode: `PWM Generation CH1..CH4`

**TIM2 — step motor zaman tabanı**
- Clock Source: `Internal Clock`
- NVIC: **TIM2 global interrupt → enabled** (bu olmadan step motor dönmez)
- Prescaler/Period yine kodda ayarlanıyor (10 kHz kesme).

**USART2 — Pi hattı**
- Mode: `Asynchronous`
- Baud: **115200**, 8N1
- NVIC: **USART2 global interrupt → enabled**

**GPIO**
- PA4, PA5, PA6, PA7 → `GPIO_Output`, başlangıç LOW, `No pull`, hız `High`
- PB12 → `GPIO_Output`, başlangıç LOW

**F401 GPIO ayrıntısı:** PWM pinlerinde `Alternate Function Push Pull`,
`No pull`, `High` veya `Very High` speed; USART2 TX'te `AF Push Pull`, RX'te
CubeMX'in AF7 varsayılanı kullanılabilir. USART ve PWM sinyalleri 3.3 V
mantık seviyesindedir.

> PA4-PA7'yi **High speed** yapın: step fazları BSRR ile tek yazmada
> değişiyor, yavaş sürücü ayarında kenarlar yayılır.

---

## main.c'ye ekleme

```c
/* USER CODE BEGIN Includes */
#include "up_app.h"
/* USER CODE END Includes */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  up_app_init(&htim3, &htim4, &htim2, &huart2);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    up_app_loop();
    /* USER CODE END 3 */
  }
}
```

`up_app.c` şu üç HAL callback'ini kendisi tanımlıyor:
`HAL_UART_RxCpltCallback`, `HAL_UART_ErrorCallback`,
`HAL_TIM_PeriodElapsedCallback`.

**CubeMX bunlardan birini zaten üretiyorsa** (örneğin HAL zaman tabanı için
TIM kullanıyorsan `HAL_TIM_PeriodElapsedCallback` çakışır) derleyici
"multiple definition" verir. Çözüm: proje ayarlarına `UP_APP_NO_CALLBACKS`
tanımını ekle ve kendi callback'lerinden çağır:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) { up_app_stepper_tick(); }
  else { HAL_IncTick(); }          /* CubeMX'in kendi işi */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) { up_app_uart_rx_byte(rx); ... }
}
```

---

## Ayarlanabilir sabitler (`up_app.h`)

| Tanım | Varsayılan | Ne yapar |
|-------|-----------|----------|
| `UP_PWM_PERIOD_US` | 20000 | ESC darbe periyodu (50 Hz) |
| `UP_ESC_ARM_MS` | 2000 | açılışta nötr verilerek beklenen süre |
| `UP_FAILSAFE_MS` | 500 | komut yokluğunda nötre dönme süresi |
| `UP_STATUS_PERIOD_MS` | 100 | STATUS gönderim periyodu |
| `UP_STEP_TICK_HZ` | 10000 | TIM2 kesme frekansı = adım çözünürlüğü |
| `UP_STEP_MAX_SPS` | 2000 | adım/s üst sınırı |
| `UP_STEP_HALF_MODE` | 0 | 1 = yarım adım (400 adım/tur), 0 = tam adım (200) |

---

## Davranış

**Açılış:** ESC'lere `UP_ESC_ARM_MS` boyunca 1500 µs (nötr) verilir, sonra
komutlar uygulanmaya başlar. Pi açılırken yayacağı ilk değerin motoru
fırlatmasını bu engelliyor.

**Failsafe:** Pi'den `UP_FAILSAFE_MS` boyunca geçerli çerçeve gelmezse
ESC'ler nötre çekilir, step motor bobinleri bırakılır ve STATUS'ta
`UP_FLAG_FAILSAFE` yükselir. Kablonun çıkması, Pi'nin kapanması, ROS'un
durması — hepsi aynı yola çıkar. Pi tarafı ayrıca 5 Hz heartbeat gönderiyor,
yani eyleyici komutu olmasa da hat canlı sayılıyor.

**Step motor:** adımlar TIM2'nin 10 kHz kesmesinde, Bresenham benzeri bir
toplayıcıyla üretiliyor; istenen adım frekansı kesme frekansının tam böleni
olmak zorunda değil. Faz değişimi tek bir `GPIOA->BSRR` yazması: set ve reset
aynı çevrimde olduğu için **ara durum yok**. Bu önemli — aynı H-köprü kolunun
iki girişi bir an birlikte HIGH kalırsa L298N'de fren + akım sıçraması olur.

**Modlar:** `idle` (bobinler serbest), `hold` (fazda kilitli), `velocity`
(sürekli dönüş), `position` (hedefe git, varınca `hold`).

---

## ⚠ Step motor akımı — okumadan bağlamayın

Pin tablosunda step motor için 4 pin var, yani **L298N'in ENA/ENB'si jumper'lı**
kalıyor ve bobinlere **12 V doğrudan** biniyor.

17HS3401'in faz direnci ~1.5 Ω. Duran motorda bu:

```
I = (12 V − ~2 V köprü düşümü) / 1.5 Ω ≈ 6.7 A     (anma akımı 1.7 A)
```

yani anma akımının ~4 katı. L298N'de chopper/akım sınırlama yok; sonuç aşırı
ısınma, titreme ve muhtemelen sürücünün ölümü. Bu tam olarak Pi tarafında
daha önce yaşanan sorun (bkz. `step_control.py` içindeki notlar).

Üç seçenek:

1. **ENA/ENB'yi PWM'e bağla (en az değişiklik).** Jumper'ları çıkar,
   ENA → **PA8 (TIM1_CH1)**, ENB → **PA9 (TIM1_CH2)**, ~10-20 kHz PWM.
   Duty akım sınırlaması görevi yapar: sürüşte yüksek, kilitte düşük
   (`hold` için ~%25-30 tipik). Bu iki pin firmware'de **henüz yok** —
   eklenmesi gerekir.
2. **Gerçek step sürücü kullan** (A4988 / DRV8825 / TMC2209). Akımı
   potansiyometreyle sınırlarsın ve 4 pin yerine 2 pin (STEP + DIR) yeter;
   `up_app.c`'nin faz tablosu yerine tek STEP darbesi üretilir.
3. **Besleme gerilimini düşür.** 1.7 A × 1.5 Ω + 2 V ≈ 4.5 V. Tork düşer
   ama donanım yanmaz. En hızlı geçici çözüm.

Ne yaparsanız yapın: motoru **`idle` modda bırakın**, gerekmedikçe `hold`'da
tutmayın. Firmware failsafe'te bobinleri kendiliğinden bırakıyor.

---

## Protokol özeti

```
+------+------+--------+-----+----------------+-----------+
| 0xAA | 0x55 | MSG_ID | LEN | payload (LEN)  | CRC16 LE  |
+------+------+--------+-----+----------------+-----------+
```

CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF), `MSG_ID + LEN + payload`
üzerinden. SOF baytları CRC'ye girmez.

**Pi → STM32**

| ID | Mesaj | Payload |
|----|-------|---------|
| 0x01 | ESC | 6 × uint16 LE, darbe µs (1000-2000) |
| 0x02 | STEPPER | uint8 mode, int16 speed_sps, int32 target |
| 0x03 | LED | uint8 (0/1) |
| 0x04 | HEARTBEAT | uint32 seq |

**STM32 → Pi**

| ID | Mesaj | Payload |
|----|-------|---------|
| 0x81 | STATUS | uint32 uptime_ms, uint8 flags, uint8 led, int32 step_pos, int16 step_speed, uint16 rx_ok, uint16 rx_err |
| 0x82 | LOG | ASCII metin |

`flags`: bit0 failsafe, bit1 esc_armed, bit2 step_energized.

---

## Test

**1) Protokol testi (kart gerekmez)**

```bash
cd stm32_fw
gcc -Wall -Wextra -Werror -std=c11 -IInc Src/up_protocol.c test_protocol.c \
    -o /tmp/up_test && /tmp/up_test
```

**2) Firmware sözdizimi kontrolü (ARM derleyicisi gerekmez)**

```bash
cd stm32_fw
gcc -c -Wall -Wextra -Werror -std=c11 -DUP_HOST_TEST -IInc -Itest \
    Src/up_app.c -o /tmp/up_app.o
```

**3) Uçtan uca (sanal STM32 ile, kart gerekmez)**

```bash
python3 tools/test_link_e2e.py
```

**4) Gerçek kartla**

```bash
python3 tools/stm32_link_test.py --port /dev/ttyAMA0
```

Sırayla: `status` (STATUS geliyor mu), `led on`, `esc all 1600`,
`step v 200`, `step idle`. Pervaneleri **çıkarın**.
