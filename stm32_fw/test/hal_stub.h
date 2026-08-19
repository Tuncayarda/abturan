/**
 * hal_stub.h — SADECE MASAUSTU SOZDIZIMI KONTROLU ICIN.
 *
 * up_app.c gercek STM32 HAL'ine karsi derleniyor; hedef donanim olmadan da
 * yazim/tip hatalarini yakalamak icin HAL'in kullandigimiz kadarini burada
 * taklit ediyoruz. Firmware'e DAHIL EDILMEZ.
 *
 * Kullanim:
 *   gcc -fsyntax-only -Wall -Wextra -std=c11 -DUP_HOST_TEST \
 *       -IInc -Itest Src/up_app.c
 */
#ifndef HAL_STUB_H
#define HAL_STUB_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    volatile uint32_t PSC, ARR, EGR, CCR1, CCR2, CCR3, CCR4;
} TIM_TypeDef;

typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;

typedef struct { volatile uint32_t BSRR, ODR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CFGR; } RCC_TypeDef;

extern GPIO_TypeDef *GPIOA;
extern GPIO_TypeDef *GPIOB;
extern RCC_TypeDef *RCC;

#define RCC_CFGR_PPRE1      0x00000700u
#define TIM_EGR_UG          0x00000001u

#define GPIO_PIN_4          ((uint16_t)0x0010)
#define GPIO_PIN_5          ((uint16_t)0x0020)
#define GPIO_PIN_6          ((uint16_t)0x0040)
#define GPIO_PIN_7          ((uint16_t)0x0080)
#define GPIO_PIN_12         ((uint16_t)0x1000)

#define TIM_CHANNEL_1       0x00000000u
#define TIM_CHANNEL_2       0x00000004u
#define TIM_CHANNEL_3       0x00000008u
#define TIM_CHANNEL_4       0x0000000Cu

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;

#define __HAL_TIM_SET_PRESCALER(h, v)   ((h)->Instance->PSC = (v))
#define __HAL_TIM_SET_AUTORELOAD(h, v)  ((h)->Instance->ARR = (v))
#define __HAL_TIM_SET_COMPARE(h, ch, v) \
    (((ch) == TIM_CHANNEL_1) ? ((h)->Instance->CCR1 = (v)) : \
     ((ch) == TIM_CHANNEL_2) ? ((h)->Instance->CCR2 = (v)) : \
     ((ch) == TIM_CHANNEL_3) ? ((h)->Instance->CCR3 = (v)) : \
                               ((h)->Instance->CCR4 = (v)))

#define __disable_irq()  ((void)0)
#define __enable_irq()   ((void)0)

uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data,
                                      uint16_t size);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *data,
                                    uint16_t size, uint32_t timeout);

#endif /* HAL_STUB_H */
