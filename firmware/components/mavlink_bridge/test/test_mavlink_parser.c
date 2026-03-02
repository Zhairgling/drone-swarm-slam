#include "unity.h"
#include "mavlink_parser.h"

#include <string.h>

/* ── helpers ─────────────────────────────────────────────────── */

/* Feed an array of bytes to the parser and return the result of the
   last byte.  Asserts INCOMPLETE for all bytes except the last. */
static mavlink_parser_result_t feed_bytes(mavlink_parser_t *p,
                                          const uint8_t *data, size_t len)
{
    mavlink_parser_result_t res = MAVLINK_PARSER_INCOMPLETE;
    for (size_t i = 0; i < len; i++) {
        res = mavlink_parser_parse_byte(p, data[i]);
        if (i < len - 1) {
            TEST_ASSERT_EQUAL(MAVLINK_PARSER_INCOMPLETE, res);
        }
    }
    return res;
}

/* Build a minimal MAVLink v1 frame (no real CRC — just structure). */
static size_t build_v1_frame(uint8_t *buf, uint8_t payload_len)
{
    size_t idx = 0;
    buf[idx++] = MAVLINK_V1_STX;          /* STX   */
    buf[idx++] = payload_len;              /* LEN   */
    buf[idx++] = 0x00;                     /* SEQ   */
    buf[idx++] = 0x01;                     /* SYSID */
    buf[idx++] = 0x01;                     /* COMPID*/
    buf[idx++] = 0x00;                     /* MSGID */
    for (uint8_t i = 0; i < payload_len; i++) {
        buf[idx++] = i;                    /* payload */
    }
    buf[idx++] = 0xAA;                     /* CKL   */
    buf[idx++] = 0xBB;                     /* CKH   */
    return idx;
}

/* Build a minimal MAVLink v2 frame (no signature). */
static size_t build_v2_frame(uint8_t *buf, uint8_t payload_len)
{
    size_t idx = 0;
    buf[idx++] = MAVLINK_V2_STX;          /* STX            */
    buf[idx++] = payload_len;              /* LEN            */
    buf[idx++] = 0x00;                     /* INCOMPAT_FLAGS */
    buf[idx++] = 0x00;                     /* COMPAT_FLAGS   */
    buf[idx++] = 0x00;                     /* SEQ            */
    buf[idx++] = 0x01;                     /* SYSID          */
    buf[idx++] = 0x01;                     /* COMPID         */
    buf[idx++] = 0x00;                     /* MSGID low      */
    buf[idx++] = 0x00;                     /* MSGID mid      */
    buf[idx++] = 0x00;                     /* MSGID high     */
    for (uint8_t i = 0; i < payload_len; i++) {
        buf[idx++] = i;
    }
    buf[idx++] = 0xCC;                     /* CKL            */
    buf[idx++] = 0xDD;                     /* CKH            */
    return idx;
}

/* Build a MAVLink v2 frame WITH signature (incompat_flags bit 0 set). */
static size_t build_v2_signed_frame(uint8_t *buf, uint8_t payload_len)
{
    size_t idx = 0;
    buf[idx++] = MAVLINK_V2_STX;
    buf[idx++] = payload_len;
    buf[idx++] = 0x01;                     /* INCOMPAT_FLAGS: signing */
    buf[idx++] = 0x00;
    buf[idx++] = 0x00;
    buf[idx++] = 0x01;
    buf[idx++] = 0x01;
    buf[idx++] = 0x00;
    buf[idx++] = 0x00;
    buf[idx++] = 0x00;
    for (uint8_t i = 0; i < payload_len; i++) {
        buf[idx++] = i;
    }
    buf[idx++] = 0xEE;                     /* CKL            */
    buf[idx++] = 0xFF;                     /* CKH            */
    for (int i = 0; i < MAVLINK_SIGNATURE_LEN; i++) {
        buf[idx++] = (uint8_t)(0x50 + i);  /* signature      */
    }
    return idx;
}

/* ── init tests ──────────────────────────────────────────────── */

TEST_CASE("parser_init sets idle state", "[mavlink_parser]")
{
    mavlink_parser_t p;
    mavlink_parser_init(&p);
    TEST_ASSERT_EQUAL(MAVLINK_PARSE_IDLE, p.state);
    TEST_ASSERT_EQUAL(0, p.idx);
}

/* ── v1 frame tests ──────────────────────────────────────────── */

TEST_CASE("parse v1 frame with zero-length payload", "[mavlink_parser]")
{
    uint8_t frame[16];
    size_t frame_len = build_v1_frame(frame, 0);
    TEST_ASSERT_EQUAL(8, frame_len);  /* header(6) + ck(2) */

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, frame_len);
}

TEST_CASE("parse v1 frame with 4-byte payload", "[mavlink_parser]")
{
    uint8_t frame[16];
    size_t frame_len = build_v1_frame(frame, 4);
    TEST_ASSERT_EQUAL(12, frame_len);

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, frame_len);
}

TEST_CASE("parse v1 frame with max payload", "[mavlink_parser]")
{
    uint8_t frame[280];
    size_t frame_len = build_v1_frame(frame, 255);
    TEST_ASSERT_EQUAL(263, frame_len);  /* 6 + 255 + 2 */

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
}

/* ── v2 frame tests ──────────────────────────────────────────── */

TEST_CASE("parse v2 frame with zero-length payload", "[mavlink_parser]")
{
    uint8_t frame[16];
    size_t frame_len = build_v2_frame(frame, 0);
    TEST_ASSERT_EQUAL(12, frame_len);  /* header(10) + ck(2) */

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, frame_len);
}

TEST_CASE("parse v2 frame with 10-byte payload", "[mavlink_parser]")
{
    uint8_t frame[32];
    size_t frame_len = build_v2_frame(frame, 10);
    TEST_ASSERT_EQUAL(22, frame_len);

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, frame_len);
}

/* ── v2 signed frame tests ───────────────────────────────────── */

TEST_CASE("parse v2 signed frame with zero payload", "[mavlink_parser]")
{
    uint8_t frame[32];
    size_t frame_len = build_v2_signed_frame(frame, 0);
    TEST_ASSERT_EQUAL(25, frame_len);  /* 10 + 0 + 2 + 13 */

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, frame_len);
}

TEST_CASE("parse v2 signed frame with 5-byte payload", "[mavlink_parser]")
{
    uint8_t frame[48];
    size_t frame_len = build_v2_signed_frame(frame, 5);
    TEST_ASSERT_EQUAL(30, frame_len);  /* 10 + 5 + 2 + 13 */

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, frame_len);
}

/* ── garbage / resync tests ──────────────────────────────────── */

TEST_CASE("parser ignores leading garbage bytes", "[mavlink_parser]")
{
    uint8_t garbage[] = {0x00, 0x42, 0xFF, 0x13, 0x37};
    uint8_t frame[16];
    size_t frame_len = build_v1_frame(frame, 2);

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    /* Feed garbage first — all should return INCOMPLETE. */
    for (size_t i = 0; i < sizeof(garbage); i++) {
        TEST_ASSERT_EQUAL(MAVLINK_PARSER_INCOMPLETE,
                          mavlink_parser_parse_byte(&p, garbage[i]));
    }

    /* Now feed the real frame. */
    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, frame_len);
}

TEST_CASE("parser resyncs after init on new frame", "[mavlink_parser]")
{
    /* Start a v1 frame but don't finish it, then re-init and start fresh. */
    mavlink_parser_t p;
    mavlink_parser_init(&p);

    uint8_t partial[] = {MAVLINK_V1_STX, 0x04, 0x00};
    for (size_t i = 0; i < sizeof(partial); i++) {
        mavlink_parser_parse_byte(&p, partial[i]);
    }

    /* Reinit (simulates reset after timeout). */
    mavlink_parser_init(&p);
    TEST_ASSERT_EQUAL(MAVLINK_PARSE_IDLE, p.state);

    /* Now a complete frame should parse cleanly. */
    uint8_t frame[16];
    size_t frame_len = build_v2_frame(frame, 1);

    mavlink_parser_result_t res = feed_bytes(&p, frame, frame_len);
    TEST_ASSERT_EQUAL(MAVLINK_PARSER_COMPLETE, res);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);
    TEST_ASSERT_EQUAL(frame_len, out_len);
}

/* ── back-to-back frame tests ────────────────────────────────── */

TEST_CASE("parse two back-to-back v1 frames", "[mavlink_parser]")
{
    uint8_t frame1[16], frame2[16];
    size_t len1 = build_v1_frame(frame1, 2);
    size_t len2 = build_v1_frame(frame2, 3);

    /* Concatenate both frames. */
    uint8_t stream[32];
    memcpy(stream, frame1, len1);
    memcpy(stream + len1, frame2, len2);
    size_t total = len1 + len2;

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    /* Parse first frame. */
    size_t i = 0;
    while (i < total) {
        mavlink_parser_result_t res =
            mavlink_parser_parse_byte(&p, stream[i++]);
        if (res == MAVLINK_PARSER_COMPLETE) {
            const uint8_t *out;
            size_t out_len;
            mavlink_parser_get_frame(&p, &out, &out_len);
            TEST_ASSERT_EQUAL(len1, out_len);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(frame1, out, len1);
            mavlink_parser_init(&p);
            break;
        }
    }

    /* Parse second frame. */
    while (i < total) {
        mavlink_parser_result_t res =
            mavlink_parser_parse_byte(&p, stream[i++]);
        if (res == MAVLINK_PARSER_COMPLETE) {
            const uint8_t *out;
            size_t out_len;
            mavlink_parser_get_frame(&p, &out, &out_len);
            TEST_ASSERT_EQUAL(len2, out_len);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(frame2, out, len2);
            return;
        }
    }

    TEST_FAIL_MESSAGE("Second frame was not completed");
}

TEST_CASE("parse mixed v1 and v2 back-to-back", "[mavlink_parser]")
{
    uint8_t frame1[16], frame2[32];
    size_t len1 = build_v1_frame(frame1, 1);
    size_t len2 = build_v2_frame(frame2, 3);

    uint8_t stream[48];
    memcpy(stream, frame1, len1);
    memcpy(stream + len1, frame2, len2);
    size_t total = len1 + len2;

    mavlink_parser_t p;
    mavlink_parser_init(&p);

    int frames_found = 0;
    for (size_t i = 0; i < total; i++) {
        mavlink_parser_result_t res =
            mavlink_parser_parse_byte(&p, stream[i]);
        if (res == MAVLINK_PARSER_COMPLETE) {
            frames_found++;
            mavlink_parser_init(&p);
        }
    }

    TEST_ASSERT_EQUAL(2, frames_found);
}

/* ── get_frame content tests ─────────────────────────────────── */

TEST_CASE("get_frame returns correct v2 content", "[mavlink_parser]")
{
    uint8_t frame[32];
    size_t frame_len = build_v2_frame(frame, 3);

    mavlink_parser_t p;
    mavlink_parser_init(&p);
    feed_bytes(&p, frame, frame_len);

    const uint8_t *out;
    size_t out_len;
    mavlink_parser_get_frame(&p, &out, &out_len);

    /* Verify STX */
    TEST_ASSERT_EQUAL_HEX8(MAVLINK_V2_STX, out[0]);
    /* Verify payload length byte */
    TEST_ASSERT_EQUAL(3, out[1]);
    /* Verify payload content (bytes 0, 1, 2) */
    TEST_ASSERT_EQUAL(0, out[10]);
    TEST_ASSERT_EQUAL(1, out[11]);
    TEST_ASSERT_EQUAL(2, out[12]);
    /* Verify checksum bytes */
    TEST_ASSERT_EQUAL_HEX8(0xCC, out[13]);
    TEST_ASSERT_EQUAL_HEX8(0xDD, out[14]);
}
