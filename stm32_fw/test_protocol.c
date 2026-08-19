/**
 * test_protocol.c — protokolun masaustu testi.
 *
 * up_protocol.c HAL'siz oldugu icin PC'de derlenebiliyor. Bu test hem
 * kendi icinde tutarliligi (encode -> parse) hem de Python tarafiyla
 * bayt bayt ayni cerceveyi urettigini dogruluyor.
 *
 * Derle ve calistir:
 *   gcc -Wall -Wextra -Werror -std=c11 -IInc Src/up_protocol.c \
 *       test_protocol.c -o /tmp/up_test && /tmp/up_test
 */
#include "up_protocol.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check(int cond, const char *what)
{
    printf("%-52s %s\n", what, cond ? "OK" : "FAIL");
    if (!cond) {
        failures++;
    }
}

static void dump(const char *label, const uint8_t *d, size_t n)
{
    printf("  %s:", label);
    for (size_t i = 0; i < n; ++i) {
        printf(" %02x", d[i]);
    }
    printf("\n");
}

int main(void)
{
    /* 1) CRC16-CCITT-FALSE'un bilinen kontrol degeri */
    check(up_crc16((const uint8_t *)"123456789", 9) == 0x29B1u,
          "crc16(\"123456789\") == 0x29B1");

    /* 2) STATUS cercevesi — Python referansiyla karsilastirilacak */
    up_status_t st = {
        .uptime_ms = 1234u,
        .flags = UP_FLAG_ESC_ARMED | UP_FLAG_STEP_ENERGIZED,
        .led = 1u,
        .stepper_position = -4200,
        .stepper_speed_sps = -350,
        .rx_ok = 900u,
        .rx_err = 3u,
    };
    uint8_t frame[UP_MAX_FRAME];
    size_t n = up_encode_status(frame, sizeof(frame), &st);
    check(n == UP_HEADER_LEN + UP_STATUS_PAYLOAD_LEN + UP_CRC_LEN,
          "status cerceve uzunlugu 22");
    dump("STATUS", frame, n);

    /* 3) ESC cercevesini coz */
    uint8_t esc_payload[UP_ESC_PAYLOAD_LEN];
    const uint16_t wanted[UP_ESC_COUNT] = {1000, 1500, 2000, 1234, 900, 2100};
    for (unsigned i = 0; i < UP_ESC_COUNT; ++i) {
        esc_payload[i * 2u]      = (uint8_t)(wanted[i] & 0xFFu);
        esc_payload[i * 2u + 1u] = (uint8_t)(wanted[i] >> 8);
    }
    n = up_encode(frame, sizeof(frame), UP_MSG_ESC, esc_payload,
                  (uint8_t)sizeof(esc_payload));
    dump("ESC   ", frame, n);

    up_parser_t parser;
    up_parser_init(&parser);
    uint8_t msg_id = 0u;
    const uint8_t *payload = NULL;
    uint8_t payload_len = 0u;
    bool got = false;
    for (size_t i = 0; i < n; ++i) {
        got = up_parser_feed(&parser, frame[i], &msg_id, &payload, &payload_len);
    }
    check(got && msg_id == UP_MSG_ESC, "ESC cercevesi parse edildi");

    up_esc_cmd_t esc;
    check(up_decode_esc(payload, payload_len, &esc), "ESC payload cozuldu");
    check(esc.pulse_us[0] == 1000 && esc.pulse_us[2] == 2000
          && esc.pulse_us[3] == 1234, "ESC degerleri dogru");
    check(esc.pulse_us[4] == UP_ESC_MIN_US && esc.pulse_us[5] == UP_ESC_MAX_US,
          "sinir disi ESC degerleri kisildi (900->1000, 2100->2000)");

    /* 4) Step motor cercevesi */
    uint8_t step_payload[UP_STEPPER_PAYLOAD_LEN];
    step_payload[0] = (uint8_t)UP_STEP_POSITION;
    step_payload[1] = 0x90u; step_payload[2] = 0x01u;          /* 400      */
    step_payload[3] = 0xC7u; step_payload[4] = 0xCFu;
    step_payload[5] = 0xFFu; step_payload[6] = 0xFFu;          /* -12345   */
    n = up_encode(frame, sizeof(frame), UP_MSG_STEPPER, step_payload,
                  (uint8_t)sizeof(step_payload));
    dump("STEP  ", frame, n);

    up_parser_init(&parser);
    got = false;
    for (size_t i = 0; i < n; ++i) {
        got = up_parser_feed(&parser, frame[i], &msg_id, &payload, &payload_len);
    }
    up_stepper_cmd_t step;
    check(got && up_decode_stepper(payload, payload_len, &step),
          "STEP cercevesi parse edildi");
    check(step.mode == UP_STEP_POSITION && step.speed_sps == 400
          && step.target == -12345, "STEP alanlari dogru (400 adim/s, -12345)");

    /* 5) Senkron kaybindan toparlanma: onune cop, arasina bozuk CRC */
    up_parser_init(&parser);
    uint8_t led_frame[UP_MAX_FRAME];
    uint8_t led_payload = 1u;
    size_t led_len = up_encode(led_frame, sizeof(led_frame), UP_MSG_LED,
                               &led_payload, 1u);

    uint8_t stream[128];
    size_t s = 0;
    stream[s++] = 0xFFu; stream[s++] = 0xAAu; stream[s++] = 0x00u; /* cop */
    memcpy(&stream[s], led_frame, led_len);
    stream[s + led_len - 1u] ^= 0xFFu;   /* CRC'yi boz */
    s += led_len;
    memcpy(&stream[s], led_frame, led_len);   /* saglam kopya */
    s += led_len;

    int frames = 0;
    bool led_state = false;
    for (size_t i = 0; i < s; ++i) {
        if (up_parser_feed(&parser, stream[i], &msg_id, &payload, &payload_len)) {
            frames++;
            check(up_decode_led(payload, payload_len, &led_state), "LED payload");
        }
    }
    check(frames == 1 && led_state, "cop + bozuk CRC sonrasi senkron geri geldi");
    check(parser.err_count > 0u, "bozuk cerceve err_count'a yazildi");

    printf("\n%s (%d hata)\n", failures ? "BASARISIZ" : "TUM TESTLER GECTI",
           failures);
    return failures ? 1 : 0;
}
