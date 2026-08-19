/**
 * up_protocol.h — Raspberry Pi <-> STM32 UART cercevesi.
 *
 * Bu dosya protokolun C tarafi. Python tarafi:
 *     src/stm32_bridge/stm32_bridge/protocol.py
 * IKISI BIRLIKTE DEGISMELI (mesaj id'leri, payload sirasi, CRC ayni).
 *
 * Cerceve:
 *
 *   +------+------+--------+-----+----------------+-----------+
 *   | 0xAA | 0x55 | MSG_ID | LEN | payload (LEN)  | CRC16 LE  |
 *   +------+------+--------+-----+----------------+-----------+
 *
 * CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF); MSG_ID + LEN + payload
 * uzerinden hesaplanir, SOF baytlari CRC'ye girmez.
 *
 * HAL'a bagimli degil — istenirse PC'de derlenip test edilebilir.
 */
#ifndef UP_PROTOCOL_H
#define UP_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UP_SOF1            0xAAu
#define UP_SOF2            0x55u
#define UP_HEADER_LEN      4u    /* SOF1 SOF2 ID LEN */
#define UP_CRC_LEN         2u
#define UP_MAX_PAYLOAD     64u
#define UP_MAX_FRAME       (UP_HEADER_LEN + UP_MAX_PAYLOAD + UP_CRC_LEN)

/* -- Pi -> STM32 -------------------------------------------------------- */
#define UP_MSG_ESC         0x01u  /* 6 x uint16 LE, darbe genisligi (us)    */
#define UP_MSG_STEPPER     0x02u  /* uint8 mode, int16 speed, int32 target  */
#define UP_MSG_LED         0x03u  /* uint8 0/1                              */
#define UP_MSG_HEARTBEAT   0x04u  /* uint32 seq                             */

/* -- STM32 -> Pi -------------------------------------------------------- */
#define UP_MSG_STATUS      0x81u  /* up_status_t (asagida)                  */
#define UP_MSG_LOG         0x82u  /* ASCII metin                            */

/* ESC kanal sayisi ve darbe sinirlari */
#define UP_ESC_COUNT       6u
#define UP_ESC_MIN_US      1000u
#define UP_ESC_NEUTRAL_US  1500u
#define UP_ESC_MAX_US      2000u

/* Step motor modlari (protocol.py ile ayni) */
typedef enum {
    UP_STEP_IDLE     = 0,  /* bobinler serbest — isinma yok               */
    UP_STEP_HOLD     = 1,  /* mevcut fazda kilitli bekle                  */
    UP_STEP_VELOCITY = 2,  /* speed_sps hizinda surekli don               */
    UP_STEP_POSITION = 3   /* target adimina git, varinca HOLD'a gec      */
} up_step_mode_t;

/* STATUS flags bitleri */
#define UP_FLAG_FAILSAFE        (1u << 0) /* komut zaman asimi             */
#define UP_FLAG_ESC_ARMED       (1u << 1) /* arm sekansi bitti             */
#define UP_FLAG_STEP_ENERGIZED  (1u << 2) /* bobinlerde akim var           */

/* Cozulmus mesajlar ------------------------------------------------------ */

typedef struct {
    uint16_t pulse_us[UP_ESC_COUNT];
} up_esc_cmd_t;

typedef struct {
    uint8_t mode;       /* up_step_mode_t                                  */
    int16_t speed_sps;  /* isaretli adim/s (VELOCITY'de yon de belirler)   */
    int32_t target;     /* POSITION modunda mutlak hedef adim              */
} up_stepper_cmd_t;

typedef struct {
    uint32_t uptime_ms;
    uint8_t  flags;
    uint8_t  led;
    int32_t  stepper_position;
    int16_t  stepper_speed_sps;
    uint16_t rx_ok;
    uint16_t rx_err;
} up_status_t;

/* payload boyutlari — Python struct'lari ile birebir */
#define UP_ESC_PAYLOAD_LEN      (2u * UP_ESC_COUNT)  /* 12 */
#define UP_STEPPER_PAYLOAD_LEN  7u                   /* 1 + 2 + 4 */
#define UP_LED_PAYLOAD_LEN      1u
#define UP_HEARTBEAT_PAYLOAD_LEN 4u
#define UP_STATUS_PAYLOAD_LEN   16u                  /* 4+1+1+4+2+2+2 */

/* CRC -------------------------------------------------------------------- */
uint16_t up_crc16(const uint8_t *data, size_t len);

/* Cerceveleme ------------------------------------------------------------ */

/** Cerceve olustur. Donen deger yazilan bayt sayisi, hata durumunda 0. */
size_t up_encode(uint8_t *out, size_t out_size,
                 uint8_t msg_id, const uint8_t *payload, uint8_t payload_len);

size_t up_encode_status(uint8_t *out, size_t out_size, const up_status_t *st);
size_t up_encode_log(uint8_t *out, size_t out_size, const char *text);

/** Payload cozuculer. Uzunluk uymazsa false. */
bool up_decode_esc(const uint8_t *payload, uint8_t len, up_esc_cmd_t *out);
bool up_decode_stepper(const uint8_t *payload, uint8_t len, up_stepper_cmd_t *out);
bool up_decode_led(const uint8_t *payload, uint8_t len, bool *out);
bool up_decode_heartbeat(const uint8_t *payload, uint8_t len, uint32_t *out);

/* Akis parser'i ---------------------------------------------------------- */

typedef struct {
    uint8_t  buf[UP_MAX_FRAME];    /* cozulmekte olan cerceve            */
    uint8_t  payload[UP_MAX_PAYLOAD]; /* tamamlanan cercevenin kopyasi   */
    uint16_t len;
    uint16_t ok_count;
    uint16_t err_count;
} up_parser_t;

void up_parser_init(up_parser_t *p);

/**
 * Tek bayt besle. Tam ve CRC'si dogru bir cerceve tamamlandiginda true
 * doner; *msg_id ve payload doldurulur (payload parser tamponuna isaret
 * eder, bir sonraki besleyene kadar gecerlidir).
 *
 * Senkron kaybinda tampondan bir bayt atip yeni SOF arar — hat bir kez
 * bozulsa bile kalici kilitlenme olmaz.
 */
bool up_parser_feed(up_parser_t *p, uint8_t byte,
                    uint8_t *msg_id, const uint8_t **payload, uint8_t *payload_len);

#ifdef __cplusplus
}
#endif

#endif /* UP_PROTOCOL_H */
