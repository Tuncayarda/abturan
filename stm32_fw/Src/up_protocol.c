/**
 * up_protocol.c — bkz. up_protocol.h
 *
 * Bilincli olarak HAL'siz ve malloc'suz: hem STM32'de hem PC'de derlenir,
 * boylece protokol Python tarafina karsi masaustunde test edilebiliyor
 * (stm32_fw/test_protocol.c).
 */
#include "up_protocol.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* CRC16-CCITT-FALSE. Tablo yok: 18 baytlik cercevede bit bit hesap
 * 84 MHz F401'de cok kisa surer; tablo icin 512 bayt flash gerekmez. */
uint16_t up_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* Kodlama                                                             */

size_t up_encode(uint8_t *out, size_t out_size,
                 uint8_t msg_id, const uint8_t *payload, uint8_t payload_len)
{
    if (out == NULL || payload_len > UP_MAX_PAYLOAD) {
        return 0u;
    }
    const size_t total = UP_HEADER_LEN + payload_len + UP_CRC_LEN;
    if (out_size < total) {
        return 0u;
    }

    out[0] = UP_SOF1;
    out[1] = UP_SOF2;
    out[2] = msg_id;
    out[3] = payload_len;
    if (payload_len > 0u && payload != NULL) {
        memcpy(&out[UP_HEADER_LEN], payload, payload_len);
    }

    const uint16_t crc = up_crc16(&out[2], (size_t)payload_len + 2u);
    out[UP_HEADER_LEN + payload_len]      = (uint8_t)(crc & 0xFFu);
    out[UP_HEADER_LEN + payload_len + 1u] = (uint8_t)(crc >> 8);
    return total;
}

/* Little-endian yazicilar. Struct'i dogrudan memcpy ETMIYORUZ: derleyici
 * araya dolgu (padding) koyuyor ve Python tarafi paketli okuyor.         */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

size_t up_encode_status(uint8_t *out, size_t out_size, const up_status_t *st)
{
    if (st == NULL) {
        return 0u;
    }
    uint8_t payload[UP_STATUS_PAYLOAD_LEN];
    put_u32(&payload[0], st->uptime_ms);
    payload[4] = st->flags;
    payload[5] = st->led;
    put_u32(&payload[6], (uint32_t)st->stepper_position);
    put_u16(&payload[10], (uint16_t)st->stepper_speed_sps);
    put_u16(&payload[12], st->rx_ok);
    put_u16(&payload[14], st->rx_err);
    return up_encode(out, out_size, UP_MSG_STATUS, payload,
                     (uint8_t)sizeof(payload));
}

size_t up_encode_log(uint8_t *out, size_t out_size, const char *text)
{
    if (text == NULL) {
        return 0u;
    }
    size_t len = strlen(text);
    if (len > UP_MAX_PAYLOAD) {
        len = UP_MAX_PAYLOAD;
    }
    return up_encode(out, out_size, UP_MSG_LOG, (const uint8_t *)text,
                     (uint8_t)len);
}

/* ------------------------------------------------------------------ */
/* Cozme                                                               */

bool up_decode_esc(const uint8_t *payload, uint8_t len, up_esc_cmd_t *out)
{
    if (payload == NULL || out == NULL || len != UP_ESC_PAYLOAD_LEN) {
        return false;
    }
    for (uint8_t i = 0; i < UP_ESC_COUNT; ++i) {
        uint16_t us = get_u16(&payload[i * 2u]);
        /* Pi tarafi da kisiyor, ama bozuk bir cerceve CRC'yi gecerse
         * motorlara sacma bir darbe gitmesin.                          */
        if (us < UP_ESC_MIN_US) {
            us = UP_ESC_MIN_US;
        } else if (us > UP_ESC_MAX_US) {
            us = UP_ESC_MAX_US;
        }
        out->pulse_us[i] = us;
    }
    return true;
}

bool up_decode_stepper(const uint8_t *payload, uint8_t len, up_stepper_cmd_t *out)
{
    if (payload == NULL || out == NULL || len != UP_STEPPER_PAYLOAD_LEN) {
        return false;
    }
    out->mode      = payload[0];
    out->speed_sps = (int16_t)get_u16(&payload[1]);
    out->target    = (int32_t)get_u32(&payload[3]);
    return (out->mode <= (uint8_t)UP_STEP_POSITION);
}

bool up_decode_led(const uint8_t *payload, uint8_t len, bool *out)
{
    if (payload == NULL || out == NULL || len != UP_LED_PAYLOAD_LEN) {
        return false;
    }
    *out = (payload[0] != 0u);
    return true;
}

bool up_decode_heartbeat(const uint8_t *payload, uint8_t len, uint32_t *out)
{
    if (payload == NULL || out == NULL || len != UP_HEARTBEAT_PAYLOAD_LEN) {
        return false;
    }
    *out = get_u32(payload);
    return true;
}

/* ------------------------------------------------------------------ */
/* Akis parser'i                                                       */

void up_parser_init(up_parser_t *p)
{
    if (p != NULL) {
        memset(p, 0, sizeof(*p));
    }
}

static void drop_front(up_parser_t *p, uint16_t count)
{
    if (count >= p->len) {
        p->len = 0u;
        return;
    }
    memmove(p->buf, &p->buf[count], (size_t)(p->len - count));
    p->len = (uint16_t)(p->len - count);
}

bool up_parser_feed(up_parser_t *p, uint8_t byte,
                    uint8_t *msg_id, const uint8_t **payload, uint8_t *payload_len)
{
    if (p == NULL) {
        return false;
    }

    if (p->len < (uint16_t)sizeof(p->buf)) {
        p->buf[p->len++] = byte;
    } else {
        /* Tampon dolduysa cerceve zaten kayip; en eski bayti at.        */
        drop_front(p, 1u);
        p->buf[p->len++] = byte;
        p->err_count++;
    }

    for (;;) {
        if (p->len == 0u) {
            return false;
        }
        if (p->buf[0] != UP_SOF1) {
            drop_front(p, 1u);
            continue;
        }
        if (p->len < 2u) {
            return false;
        }
        if (p->buf[1] != UP_SOF2) {
            drop_front(p, 1u);
            continue;
        }
        if (p->len < UP_HEADER_LEN) {
            return false;
        }

        const uint8_t len_field = p->buf[3];
        if (len_field > UP_MAX_PAYLOAD) {
            p->err_count++;
            drop_front(p, 1u);
            continue;
        }

        const uint16_t total = (uint16_t)(UP_HEADER_LEN + len_field + UP_CRC_LEN);
        if (p->len < total) {
            return false;
        }

        const uint16_t want = up_crc16(&p->buf[2], (size_t)len_field + 2u);
        const uint16_t got  = get_u16(&p->buf[UP_HEADER_LEN + len_field]);
        if (want != got) {
            /* Bu SOF muhtemelen veri icindeki rastlanti — bir bayt at.  */
            p->err_count++;
            drop_front(p, 1u);
            continue;
        }

        if (len_field > 0u) {
            memcpy(p->payload, &p->buf[UP_HEADER_LEN], len_field);
        }
        if (msg_id != NULL) {
            *msg_id = p->buf[2];
        }
        if (payload != NULL) {
            *payload = p->payload;
        }
        if (payload_len != NULL) {
            *payload_len = len_field;
        }
        p->ok_count++;
        drop_front(p, total);
        return true;
    }
}
