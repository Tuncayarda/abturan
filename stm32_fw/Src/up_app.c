/**
 * up_app.c — STM32 eyleyici uygulamasi. Bkz. up_app.h
 *
 * Tasarim notlari
 * ---------------
 * ESC darbeleri donanim timer'indan uretiliyor (TIM3/TIM4, 1 MHz sayac,
 * ARR=20000 -> 50 Hz). CCR degeri dogrudan mikro saniye; yazilim jitter'i
 * darbeye hic karismiyor. Bu isin Pi'de yapilmamasinin sebebi de bu:
 * Linux userspace'te 1500 us'lik darbeyi +-10 us tutmak garanti edilemiyor.
 *
 * Step motor adimlari TIM2'nin 10 kHz kesmesinde uretiliyor. Kesme icinde
 * yalnizca bir toplayici (accumulator) ve tek bir GPIO yazmasi var; hiz
 * degisimi ana dongude kuyruklanip kesmede uygulaniyor.
 *
 * Failsafe: Pi'den UP_FAILSAFE_MS boyunca gecerli cerceve gelmezse ESC'ler
 * notre, step motor serbeste dusuyor. Kablonun cikmasi, Pi'nin kapanmasi ve
 * ROS'un durmasi ayni yola cikiyor.
 */
#include "up_app.h"

#include <string.h>

/* -------------------------------------------------------------------- */
/* STM32F401RCT6 LQFP64 pin haritasi — up_app.h'deki tabloyla ayni       */

#define UP_STEP_PORT        GPIOA
#define UP_STEP_IN1_PIN     GPIO_PIN_4
#define UP_STEP_IN2_PIN     GPIO_PIN_5
#define UP_STEP_IN3_PIN     GPIO_PIN_6
#define UP_STEP_IN4_PIN     GPIO_PIN_7
#define UP_STEP_ALL_PINS    (UP_STEP_IN1_PIN | UP_STEP_IN2_PIN | \
                             UP_STEP_IN3_PIN | UP_STEP_IN4_PIN)

#define UP_LED_PORT         GPIOB
#define UP_LED_PIN          GPIO_PIN_12

/* ESC kanal sirasi: ESC1..ESC6 -> PB0, PB1, PB6, PB7, PB8, PB9 */
static const uint32_t esc_channel[UP_ESC_COUNT] = {
    TIM_CHANNEL_3,  /* ESC1 PB0 TIM3_CH3 */
    TIM_CHANNEL_4,  /* ESC2 PB1 TIM3_CH4 */
    TIM_CHANNEL_1,  /* ESC3 PB6 TIM4_CH1 */
    TIM_CHANNEL_2,  /* ESC4 PB7 TIM4_CH2 */
    TIM_CHANNEL_3,  /* ESC5 PB8 TIM4_CH3 */
    TIM_CHANNEL_4,  /* ESC6 PB9 TIM4_CH4 */
};

/* -------------------------------------------------------------------- */
/* Faz tablolari. Her satir: IN1..IN4                                    */

#if UP_STEP_HALF_MODE
/* Yarim adim: arada tek bobinli durumlar var. Daha yumusak, tork biraz
 * dusuk, 400 adim/tur.                                                  */
static const uint8_t step_table[8][4] = {
    {1, 0, 1, 0}, {0, 0, 1, 0}, {0, 1, 1, 0}, {0, 1, 0, 0},
    {0, 1, 0, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}, {1, 0, 0, 0},
};
#define UP_STEP_PHASES  8u
#else
/* Tam adim: iki bobin de her zaman enerjili. Tork yuksek, 200 adim/tur. */
static const uint8_t step_table[4][4] = {
    {1, 0, 1, 0}, {0, 1, 1, 0}, {0, 1, 0, 1}, {1, 0, 0, 1},
};
#define UP_STEP_PHASES  4u
#endif

/* Faz tablosunu bir kere BSRR maskelerine cevirip saklıyoruz: kesme
 * icinde dongu ve dallanma kalmiyor, tek register yazmasi yetiyor.      */
static uint32_t phase_bsrr[UP_STEP_PHASES];

/* -------------------------------------------------------------------- */
/* Durum                                                                 */

static TIM_HandleTypeDef *tim_a;      /* TIM3: ESC1-2 */
static TIM_HandleTypeDef *tim_b;      /* TIM4: ESC3-6 */
static TIM_HandleTypeDef *tim_step;   /* TIM2: adim zaman tabani */
static UART_HandleTypeDef *uart;      /* USART2: Pi hatti */

/* UART RX halka tamponu. ISR yaziyor, ana dongu okuyor. */
#define RX_RING_SIZE 256u
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint16_t rx_overflow;
static uint8_t rx_byte;               /* HAL_UART_Receive_IT hedefi */

static up_parser_t parser;

static uint16_t esc_target_us[UP_ESC_COUNT];
static bool     esc_armed;
static uint32_t boot_ms;
static uint32_t last_cmd_ms;
static uint32_t last_status_ms;
static bool     failsafe;
static bool     led_state;

/* Step motoru kesme ile ana dongu paylasiyor -> volatile. */
static volatile uint8_t  step_mode = UP_STEP_IDLE;
static volatile int16_t  step_speed_sps;      /* isaretli, komut geldigi gibi */
static volatile int32_t  step_target;
static volatile int32_t  step_position;
static volatile uint32_t step_accum;
static volatile uint8_t  step_phase;
static volatile bool     step_energized;

/* -------------------------------------------------------------------- */
/* Yardimcilar                                                           */

static int16_t clamp_i16(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return (int16_t)lo;
    }
    if (v > hi) {
        return (int16_t)hi;
    }
    return (int16_t)v;
}

/** Timer saati. APB1 on bolucusu 1 degilse timer saati PCLK1'in 2 kati. */
static uint32_t timer_clock_hz(void)
{
#ifdef UP_TIMER_CLOCK_HZ
    return (uint32_t)UP_TIMER_CLOCK_HZ;
#else
    uint32_t clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0u) {
        clk *= 2u;
    }
    return clk;
#endif
}

/** Timer'i 1 us'lik sayac adimina ve verilen periyoda ayarla. */
static void configure_timebase(TIM_HandleTypeDef *htim, uint32_t period_ticks)
{
    const uint32_t presc = (timer_clock_hz() / 1000000u) - 1u;
    __HAL_TIM_SET_PRESCALER(htim, presc);
    __HAL_TIM_SET_AUTORELOAD(htim, period_ticks - 1u);
    /* PSC gölge register; UG olayi olmadan yeni deger yuklenmiyor. */
    htim->Instance->EGR = TIM_EGR_UG;
}

/* -------------------------------------------------------------------- */
/* ESC                                                                   */

static TIM_HandleTypeDef *esc_timer(uint8_t index)
{
    return (index < 2u) ? tim_a : tim_b;
}

/** Darbeyi donanima yaz (us cinsinden, CCR dogrudan us). */
static void esc_write(uint8_t index, uint16_t pulse_us)
{
    if (pulse_us < UP_ESC_MIN_US) {
        pulse_us = UP_ESC_MIN_US;
    } else if (pulse_us > UP_ESC_MAX_US) {
        pulse_us = UP_ESC_MAX_US;
    }
    __HAL_TIM_SET_COMPARE(esc_timer(index), esc_channel[index], pulse_us);
}

static void esc_write_all(uint16_t pulse_us)
{
    for (uint8_t i = 0; i < UP_ESC_COUNT; ++i) {
        esc_write(i, pulse_us);
    }
}

/* -------------------------------------------------------------------- */
/* Step motor                                                            */

static void build_phase_masks(void)
{
    static const uint16_t pins[4] = {
        UP_STEP_IN1_PIN, UP_STEP_IN2_PIN, UP_STEP_IN3_PIN, UP_STEP_IN4_PIN
    };
    for (uint8_t phase = 0; phase < UP_STEP_PHASES; ++phase) {
        uint32_t set = 0u;
        uint32_t reset = 0u;
        for (uint8_t i = 0; i < 4u; ++i) {
            if (step_table[phase][i]) {
                set |= pins[i];
            } else {
                reset |= pins[i];
            }
        }
        /* BSRR: alt 16 bit set, ust 16 bit reset. Tek yazmada hem set hem
         * reset oldugu icin ARADA GECIS DURUMU YOK — bu onemli: iki pin
         * bir an birlikte HIGH kalirsa L298N'de o kolda fren + akim
         * sicramasi oluyor (Pi tarafinda bu, sirali yazmayla cozulmustu). */
        phase_bsrr[phase] = set | (reset << 16);
    }
}

static void step_apply_phase(uint8_t phase)
{
    UP_STEP_PORT->BSRR = phase_bsrr[phase];
    step_energized = true;
}

static void step_release(void)
{
    /* Dort giris de LOW -> L298N cikislari serbest, bobinlerde akim yok. */
    UP_STEP_PORT->BSRR = ((uint32_t)UP_STEP_ALL_PINS) << 16;
    step_energized = false;
}

void up_app_stepper_tick(void)
{
    const uint8_t mode = step_mode;

    if (mode == UP_STEP_IDLE) {
        if (step_energized) {
            step_release();
        }
        return;
    }

    if (!step_energized) {
        step_apply_phase(step_phase);
    }

    int8_t dir = 0;
    if (mode == UP_STEP_POSITION) {
        if (step_position == step_target) {
            step_mode = UP_STEP_HOLD;   /* varinca kilitli bekle */
            step_accum = 0u;
            return;
        }
        dir = (step_target > step_position) ? 1 : -1;
    } else if (mode == UP_STEP_VELOCITY) {
        if (step_speed_sps == 0) {
            step_mode = UP_STEP_HOLD;
            step_accum = 0u;
            return;
        }
        dir = (step_speed_sps > 0) ? 1 : -1;
    } else {
        return;                         /* HOLD: fazi tut, adim atma */
    }

    int32_t speed = step_speed_sps;
    if (speed < 0) {
        speed = -speed;
    }
    if (speed == 0) {
        return;
    }

    /* Bresenham benzeri toplayici: her kesmede hiz kadar ekle, esik
     * asilinca bir adim at. Boylece adim frekansi kesme frekansinin
     * tam boleni olmak zorunda degil.                                   */
    step_accum += (uint32_t)speed;
    if (step_accum < UP_STEP_TICK_HZ) {
        return;
    }
    step_accum -= UP_STEP_TICK_HZ;

    if (dir > 0) {
        step_phase = (uint8_t)((step_phase + 1u) % UP_STEP_PHASES);
    } else {
        step_phase = (uint8_t)((step_phase + UP_STEP_PHASES - 1u) % UP_STEP_PHASES);
    }
    step_apply_phase(step_phase);
    step_position += dir;
}

static void stepper_command(const up_stepper_cmd_t *cmd)
{
    const int16_t speed = clamp_i16(cmd->speed_sps, -UP_STEP_MAX_SPS,
                                    UP_STEP_MAX_SPS);
    /* Kesmenin yarim guncellenmis komut gormemesi icin kritik bolge. */
    __disable_irq();
    step_speed_sps = speed;
    step_target = cmd->target;
    step_mode = cmd->mode;
    if (cmd->mode == UP_STEP_IDLE) {
        step_accum = 0u;
    }
    __enable_irq();
}

/* -------------------------------------------------------------------- */
/* UART                                                                  */

void up_app_uart_rx_byte(uint8_t byte)
{
    const uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING_SIZE);
    if (next == rx_tail) {
        rx_overflow++;      /* ana dongu geride kaldi; en yeniyi at */
        return;
    }
    rx_ring[rx_head] = byte;
    rx_head = next;
}

static bool rx_pop(uint8_t *out)
{
    if (rx_tail == rx_head) {
        return false;
    }
    *out = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING_SIZE);
    return true;
}

static void uart_send(const uint8_t *data, size_t len)
{
    /* 22 baytlik STATUS 115200 baud'da ~1.9 ms. 10 Hz'de gonderiyoruz,
     * bloklamasi sorun degil; DMA eklemek gereksiz karmasiklik olurdu. */
    (void)HAL_UART_Transmit(uart, (uint8_t *)data, (uint16_t)len, 20u);
}

static void send_log(const char *text)
{
    uint8_t frame[UP_MAX_FRAME];
    const size_t n = up_encode_log(frame, sizeof(frame), text);
    if (n > 0u) {
        uart_send(frame, n);
    }
}

static void send_status(void)
{
    up_status_t st;
    memset(&st, 0, sizeof(st));
    st.uptime_ms = HAL_GetTick() - boot_ms;
    st.flags = (uint8_t)((failsafe ? UP_FLAG_FAILSAFE : 0u)
                       | (esc_armed ? UP_FLAG_ESC_ARMED : 0u)
                       | (step_energized ? UP_FLAG_STEP_ENERGIZED : 0u));
    st.led = led_state ? 1u : 0u;
    st.stepper_position = step_position;
    st.stepper_speed_sps = step_speed_sps;
    st.rx_ok = parser.ok_count;
    st.rx_err = (uint16_t)(parser.err_count + rx_overflow);

    uint8_t frame[UP_MAX_FRAME];
    const size_t n = up_encode_status(frame, sizeof(frame), &st);
    if (n > 0u) {
        uart_send(frame, n);
    }
}

/* -------------------------------------------------------------------- */
/* Cerceve isleme                                                        */

static void handle_frame(uint8_t msg_id, const uint8_t *payload, uint8_t len)
{
    switch (msg_id) {
    case UP_MSG_ESC: {
        up_esc_cmd_t cmd;
        if (!up_decode_esc(payload, len, &cmd)) {
            return;
        }
        memcpy(esc_target_us, cmd.pulse_us, sizeof(esc_target_us));
        last_cmd_ms = HAL_GetTick();
        break;
    }
    case UP_MSG_STEPPER: {
        up_stepper_cmd_t cmd;
        if (!up_decode_stepper(payload, len, &cmd)) {
            return;
        }
        stepper_command(&cmd);
        last_cmd_ms = HAL_GetTick();
        break;
    }
    case UP_MSG_LED: {
        bool state = false;
        if (!up_decode_led(payload, len, &state)) {
            return;
        }
        led_state = state;
        HAL_GPIO_WritePin(UP_LED_PORT, UP_LED_PIN,
                          state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        last_cmd_ms = HAL_GetTick();
        break;
    }
    case UP_MSG_HEARTBEAT: {
        uint32_t seq = 0u;
        if (!up_decode_heartbeat(payload, len, &seq)) {
            return;
        }
        /* Icerigi kullanmiyoruz; onemli olan cercevenin gelmis olmasi:
         * eyleyici komutu olmasa bile hat canli demek.                  */
        last_cmd_ms = HAL_GetTick();
        break;
    }
    default:
        break;
    }
}

/* -------------------------------------------------------------------- */
/* Init / ana dongu                                                      */

void up_app_init(TIM_HandleTypeDef *tim_esc_a,
                 TIM_HandleTypeDef *tim_esc_b,
                 TIM_HandleTypeDef *tim_step_base,
                 UART_HandleTypeDef *huart)
{
    tim_a = tim_esc_a;
    tim_b = tim_esc_b;
    tim_step = tim_step_base;
    uart = huart;

    up_parser_init(&parser);
    build_phase_masks();
    step_release();
    HAL_GPIO_WritePin(UP_LED_PORT, UP_LED_PIN, GPIO_PIN_RESET);
    led_state = false;

    /* -- ESC PWM: 1 MHz sayac, 20 ms periyot -> CCR = darbe (us) ------ */
    configure_timebase(tim_a, UP_PWM_PERIOD_US);
    configure_timebase(tim_b, UP_PWM_PERIOD_US);

    for (uint8_t i = 0; i < UP_ESC_COUNT; ++i) {
        esc_target_us[i] = UP_ESC_NEUTRAL_US;
        esc_write(i, UP_ESC_NEUTRAL_US);
        HAL_TIM_PWM_Start(esc_timer(i), esc_channel[i]);
    }

    /* -- Step motor zaman tabani: 1 MHz sayac, UP_STEP_TICK_HZ kesme -- */
    configure_timebase(tim_step, 1000000u / UP_STEP_TICK_HZ);
    HAL_TIM_Base_Start_IT(tim_step);

    /* -- Pi hatti: bayt bayt kesmeli alim ---------------------------- */
    HAL_UART_Receive_IT(uart, &rx_byte, 1u);

    boot_ms = HAL_GetTick();
    last_cmd_ms = boot_ms;
    last_status_ms = boot_ms;
    esc_armed = false;
    failsafe = true;      /* arm bitene kadar failsafe sayiyoruz */

    send_log("up_app hazir");
}

void up_app_loop(void)
{
    const uint32_t now = HAL_GetTick();

    /* 1) Gelen baytlari cerceveye cevir */
    uint8_t byte;
    while (rx_pop(&byte)) {
        uint8_t msg_id = 0u;
        const uint8_t *payload = NULL;
        uint8_t payload_len = 0u;
        if (up_parser_feed(&parser, byte, &msg_id, &payload, &payload_len)) {
            handle_frame(msg_id, payload, payload_len);
        }
    }

    /* 2) Arm sekansi: ESC'ler acilista UP_ESC_ARM_MS boyunca notr gorur.
     *    Bu sure bitmeden komut uygulanmiyor — aksi halde Pi acilirken
     *    yaydigi ilk deger motoru firlatabilir.                         */
    if (!esc_armed) {
        esc_write_all(UP_ESC_NEUTRAL_US);
        if ((now - boot_ms) >= UP_ESC_ARM_MS) {
            esc_armed = true;
            send_log("ESC arm tamam");
        }
    }

    /* 3) Failsafe: komut kesildi mi? */
    const bool timed_out = (now - last_cmd_ms) > UP_FAILSAFE_MS;
    if (timed_out != failsafe && esc_armed) {
        failsafe = timed_out;
        send_log(timed_out ? "FAILSAFE: komut yok" : "komut geri geldi");
    }

    if (failsafe) {
        esc_write_all(UP_ESC_NEUTRAL_US);
        /* Bobinleri de birak: hem isinmayi hem sahipsiz tork uygulamayi
         * onluyor.                                                       */
        step_mode = UP_STEP_IDLE;
    } else if (esc_armed) {
        for (uint8_t i = 0; i < UP_ESC_COUNT; ++i) {
            esc_write(i, esc_target_us[i]);
        }
    }

    /* 4) Durum raporu */
    if ((now - last_status_ms) >= UP_STATUS_PERIOD_MS) {
        last_status_ms = now;
        send_status();
    }
}

/* -------------------------------------------------------------------- */
/* HAL callback'leri.
 * CubeMX ayni isimleri uretiyorsa UP_APP_NO_CALLBACKS tanimla ve
 * kendi callback'lerinden up_app_uart_rx_byte / up_app_stepper_tick
 * cagir (bkz. README).                                                  */
#ifndef UP_APP_NO_CALLBACKS

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart) {
        up_app_uart_rx_byte(rx_byte);
        HAL_UART_Receive_IT(uart, &rx_byte, 1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Overrun/framing hatasindan sonra alim kendiliginden yeniden
     * baslamiyor; yeniden silahlamazsak hat sessizlesiyor.              */
    if (huart == uart) {
        HAL_UART_Receive_IT(uart, &rx_byte, 1u);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == tim_step) {
        up_app_stepper_tick();
    }
}

#endif /* UP_APP_NO_CALLBACKS */
