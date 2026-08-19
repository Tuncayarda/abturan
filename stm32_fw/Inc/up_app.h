/**
 * up_app.h — STM32 eyleyici uygulamasi.
 *
 * Donanim gorevleri (STM32F401RCT6, LQFP64):
 *
 *   ESC 1 PWM        PB0    TIM3_CH3 / AF2
 *   ESC 2 PWM        PB1    TIM3_CH4 / AF2
 *   ESC 3 PWM        PB6    TIM4_CH1 / AF2
 *   ESC 4 PWM        PB7    TIM4_CH2 / AF2
 *   ESC 5 PWM        PB8    TIM4_CH3 / AF2
 *   ESC 6 PWM        PB9    TIM4_CH4 / AF2
 *   Step motor IN1   PA4    GPIO cikis
 *   Step motor IN2   PA5    GPIO cikis
 *   Step motor IN3   PA6    GPIO cikis
 *   Step motor IN4   PA7    GPIO cikis
 *   LED              PB12   GPIO cikis
 *   Pi'ye TX         PA2    USART2_TX / AF7
 *   Pi'den RX        PA3    USART2_RX / AF7
 *
 * Kullanim (main.c icinde, CubeMX urettigi init'lerden SONRA):
 *
 *     up_app_init(&htim3, &htim4, &htim2, &huart2);
 *     while (1) {
 *         up_app_loop();
 *     }
 *
 * ve kesmelerden:
 *     HAL_UART_RxCpltCallback     -> up_app_uart_rx_byte(...)
 *     HAL_TIM_PeriodElapsedCallback (TIM2) -> up_app_stepper_tick()
 *
 * (up_app.c bu iki callback'i kendisi de saglar; CubeMX ayni isimleri
 *  uretiyorsa UP_APP_NO_CALLBACKS tanimlayip yukaridaki gibi elle bagla.)
 */
#ifndef UP_APP_H
#define UP_APP_H

#include <stdbool.h>
#include <stdint.h>

/* HAL tipleri (TIM_HandleTypeDef / UART_HandleTypeDef) CubeMX'in urettigi
 * main.h uzerinden geliyor. UP_HOST_TEST ile masaustunde sadece sozdizimi
 * kontrolu icin sahte bir baslik konabiliyor (bkz. stm32_fw/README.md). */
#ifdef UP_HOST_TEST
#include "hal_stub.h"
#else
#include "main.h"
#endif

#include "up_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -- ayarlanabilir sabitler --------------------------------------------- */

/** ESC darbe periyodu (us). 20000 us = 50 Hz, standart hobi ESC.         */
#ifndef UP_PWM_PERIOD_US
#define UP_PWM_PERIOD_US        20000u
#endif

/** Aciliste ESC'lere notr verilerek beklenen sure (ms).                   */
#ifndef UP_ESC_ARM_MS
#define UP_ESC_ARM_MS           2000u
#endif

/**
 * Pi'den bu sure boyunca gecerli cerceve gelmezse failsafe:
 * ESC'ler notr, step motor serbest. Kablo kopmasi / Pi cokmesi / ROS
 * durmasi hepsi bu yola dusuyor.
 */
#ifndef UP_FAILSAFE_MS
#define UP_FAILSAFE_MS          500u
#endif

/** STATUS cercevesi gonderim periyodu (ms).                              */
#ifndef UP_STATUS_PERIOD_MS
#define UP_STATUS_PERIOD_MS     100u
#endif

/** Step motor kesme frekansi (Hz). TIM2 bu hizda tetiklenmeli.           */
#ifndef UP_STEP_TICK_HZ
#define UP_STEP_TICK_HZ         10000u
#endif

/**
 * Step motor hiz siniri (adim/s). Kesme frekansini gecemez (her kesmede
 * en fazla bir adim atiliyor). 17HS3401 200 adim/tur; 2000 adim/s = 600 rpm
 * ki L298N ile ulasilamayacak kadar yuksek — gercek sinir ~800.
 */
#ifndef UP_STEP_MAX_SPS
#define UP_STEP_MAX_SPS         2000
#endif

/**
 * 1 = yarim adim (8 faz, daha yumusak/sessiz, 400 adim/tur)
 * 0 = tam adim  (4 faz, daha yuksek tork, 200 adim/tur)
 */
#ifndef UP_STEP_HALF_MODE
#define UP_STEP_HALF_MODE       0
#endif

/* -- API ---------------------------------------------------------------- */

/**
 * Uygulamayi baslat.
 *
 * @param tim_esc_a  ESC1-2 timer'i (TIM3: CH3=PB0, CH4=PB1)
 * @param tim_esc_b  ESC3-6 timer'i (TIM4: CH1..CH4 = PB6..PB9)
 * @param tim_step   step motor zaman tabani (TIM2, UP_STEP_TICK_HZ kesme)
 * @param huart      Pi ile konusan UART (USART2)
 *
 * Timer prescaler/autoreload degerleri burada KOD ICINDE ayarlaniyor;
 * CubeMX'te ne yazdigin onemli degil.
 */
void up_app_init(TIM_HandleTypeDef *tim_esc_a,
                 TIM_HandleTypeDef *tim_esc_b,
                 TIM_HandleTypeDef *tim_step,
                 UART_HandleTypeDef *huart);

/** main() icindeki while(1) dongusunde cagir. Blokllamaz. */
void up_app_loop(void);

/** TIM2 kesmesinden cagir (UP_STEP_TICK_HZ frekansinda). */
void up_app_stepper_tick(void);

/** UART RX kesmesinden cagir, gelen her bayt icin. */
void up_app_uart_rx_byte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif /* UP_APP_H */
