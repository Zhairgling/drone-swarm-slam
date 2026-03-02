/* Host-compilable test for the MAVLink frame parser.
   Compile: cc -o test_mavlink_parser test_mavlink_parser_host.c \
                ../components/mavlink_bridge/mavlink_parser.c \
                -I../components/mavlink_bridge/include
   Run:     ./test_mavlink_parser                                        */

#include "mavlink_parser.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int tests_run  = 0;
static int tests_pass = 0;

#define RUN(name)                                                              \
    do {                                                                       \
        tests_run++;                                                           \
        printf("  %-50s ", #name);                                             \
        name();                                                                \
        tests_pass++;                                                          \
        printf("PASS\n");                                                      \
    } while (0)

/* ── helpers ─────────────────────────────────────────────────── */

static mavlink_parser_result_t feed(mavlink_parser_t *p,
                                    const uint8_t *data, size_t len)
{
    mavlink_parser_result_t res = MAVLINK_PARSER_INCOMPLETE;
    for (size_t i = 0; i < len; i++) {
        res = mavlink_parser_parse_byte(p, data[i]);
    }
    return res;
}

static size_t build_v1(uint8_t *buf, uint8_t plen)
{
    size_t i = 0;
    buf[i++] = 0xFE; buf[i++] = plen;
    buf[i++] = 0; buf[i++] = 1; buf[i++] = 1; buf[i++] = 0;
    for (uint8_t j = 0; j < plen; j++) buf[i++] = j;
    buf[i++] = 0xAA; buf[i++] = 0xBB;
    return i;
}

static size_t build_v2(uint8_t *buf, uint8_t plen)
{
    size_t i = 0;
    buf[i++] = 0xFD; buf[i++] = plen;
    buf[i++] = 0; buf[i++] = 0; buf[i++] = 0;
    buf[i++] = 1; buf[i++] = 1;
    buf[i++] = 0; buf[i++] = 0; buf[i++] = 0;
    for (uint8_t j = 0; j < plen; j++) buf[i++] = j;
    buf[i++] = 0xCC; buf[i++] = 0xDD;
    return i;
}

static size_t build_v2_signed(uint8_t *buf, uint8_t plen)
{
    size_t i = 0;
    buf[i++] = 0xFD; buf[i++] = plen;
    buf[i++] = 0x01; buf[i++] = 0; buf[i++] = 0;
    buf[i++] = 1; buf[i++] = 1;
    buf[i++] = 0; buf[i++] = 0; buf[i++] = 0;
    for (uint8_t j = 0; j < plen; j++) buf[i++] = j;
    buf[i++] = 0xEE; buf[i++] = 0xFF;
    for (int j = 0; j < 13; j++) buf[i++] = (uint8_t)(0x50 + j);
    return i;
}

/* ── tests ───────────────────────────────────────────────────── */

static void test_init(void)
{
    mavlink_parser_t p;
    mavlink_parser_init(&p);
    assert(p.state == MAVLINK_PARSE_IDLE);
    assert(p.idx == 0);
}

static void test_v1_empty_payload(void)
{
    uint8_t f[16]; size_t fl = build_v1(f, 0);
    assert(fl == 8);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
    assert(memcmp(f, o, fl) == 0);
}

static void test_v1_payload(void)
{
    uint8_t f[32]; size_t fl = build_v1(f, 4);
    assert(fl == 12);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
}

static void test_v1_max_payload(void)
{
    uint8_t f[280]; size_t fl = build_v1(f, 255);
    assert(fl == 263);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
}

static void test_v2_empty_payload(void)
{
    uint8_t f[16]; size_t fl = build_v2(f, 0);
    assert(fl == 12);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
}

static void test_v2_payload(void)
{
    uint8_t f[32]; size_t fl = build_v2(f, 10);
    assert(fl == 22);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
    assert(memcmp(f, o, fl) == 0);
}

static void test_v2_signed_empty(void)
{
    uint8_t f[32]; size_t fl = build_v2_signed(f, 0);
    assert(fl == 25);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
}

static void test_v2_signed_payload(void)
{
    uint8_t f[48]; size_t fl = build_v2_signed(f, 5);
    assert(fl == 30);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
    assert(memcmp(f, o, fl) == 0);
}

static void test_garbage_before_frame(void)
{
    uint8_t garbage[] = {0x00, 0x42, 0xFF, 0x13, 0x37};
    uint8_t f[16]; size_t fl = build_v1(f, 2);
    mavlink_parser_t p; mavlink_parser_init(&p);
    assert(feed(&p, garbage, sizeof(garbage)) == MAVLINK_PARSER_INCOMPLETE);
    assert(feed(&p, f, fl) == MAVLINK_PARSER_COMPLETE);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(ol == fl);
    assert(memcmp(f, o, fl) == 0);
}

static void test_back_to_back_v1(void)
{
    uint8_t f1[16], f2[16];
    size_t l1 = build_v1(f1, 2), l2 = build_v1(f2, 3);
    uint8_t stream[32];
    memcpy(stream, f1, l1);
    memcpy(stream + l1, f2, l2);

    mavlink_parser_t p; mavlink_parser_init(&p);
    int found = 0;
    for (size_t i = 0; i < l1 + l2; i++) {
        if (mavlink_parser_parse_byte(&p, stream[i]) == MAVLINK_PARSER_COMPLETE) {
            found++;
            mavlink_parser_init(&p);
        }
    }
    assert(found == 2);
}

static void test_back_to_back_mixed(void)
{
    uint8_t f1[16], f2[32];
    size_t l1 = build_v1(f1, 1), l2 = build_v2(f2, 3);
    uint8_t stream[48];
    memcpy(stream, f1, l1);
    memcpy(stream + l1, f2, l2);

    mavlink_parser_t p; mavlink_parser_init(&p);
    int found = 0;
    for (size_t i = 0; i < l1 + l2; i++) {
        if (mavlink_parser_parse_byte(&p, stream[i]) == MAVLINK_PARSER_COMPLETE) {
            found++;
            mavlink_parser_init(&p);
        }
    }
    assert(found == 2);
}

static void test_frame_content(void)
{
    uint8_t f[32]; size_t fl = build_v2(f, 3);
    mavlink_parser_t p; mavlink_parser_init(&p);
    feed(&p, f, fl);
    const uint8_t *o; size_t ol;
    mavlink_parser_get_frame(&p, &o, &ol);
    assert(o[0] == 0xFD);
    assert(o[1] == 3);
    assert(o[10] == 0 && o[11] == 1 && o[12] == 2);
    assert(o[13] == 0xCC && o[14] == 0xDD);
}

static void test_incomplete_does_not_complete(void)
{
    mavlink_parser_t p; mavlink_parser_init(&p);
    /* Feed only 3 bytes of a v2 frame with 10-byte payload (needs 22 total) */
    uint8_t partial[] = {0xFD, 10, 0x00};
    assert(feed(&p, partial, sizeof(partial)) == MAVLINK_PARSER_INCOMPLETE);
    assert(p.state == MAVLINK_PARSE_ACCUMULATING);
}

/* ── main ────────────────────────────────────────────────────── */

int main(void)
{
    printf("MAVLink parser host tests\n");

    RUN(test_init);
    RUN(test_v1_empty_payload);
    RUN(test_v1_payload);
    RUN(test_v1_max_payload);
    RUN(test_v2_empty_payload);
    RUN(test_v2_payload);
    RUN(test_v2_signed_empty);
    RUN(test_v2_signed_payload);
    RUN(test_garbage_before_frame);
    RUN(test_back_to_back_v1);
    RUN(test_back_to_back_mixed);
    RUN(test_frame_content);
    RUN(test_incomplete_does_not_complete);

    printf("\n%d/%d tests passed\n", tests_pass, tests_run);
    return tests_pass == tests_run ? 0 : 1;
}
