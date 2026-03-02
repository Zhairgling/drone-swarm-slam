#include "mavlink_parser.h"

void mavlink_parser_init(mavlink_parser_t *parser)
{
    parser->state = MAVLINK_PARSE_IDLE;
    parser->idx = 0;
    parser->expected_len = 0;
}

mavlink_parser_result_t mavlink_parser_parse_byte(mavlink_parser_t *parser,
                                                  uint8_t byte)
{
    switch (parser->state) {
    case MAVLINK_PARSE_IDLE:
        if (byte == MAVLINK_V1_STX || byte == MAVLINK_V2_STX) {
            parser->buf[0] = byte;
            parser->idx = 1;
            parser->state = MAVLINK_PARSE_GOT_STX;
        }
        return MAVLINK_PARSER_INCOMPLETE;

    case MAVLINK_PARSE_GOT_STX:
        /* Second byte is always payload length. */
        parser->buf[parser->idx++] = byte;
        if (parser->buf[0] == MAVLINK_V1_STX) {
            parser->expected_len =
                MAVLINK_V1_HEADER_LEN + byte + MAVLINK_CHECKSUM_LEN;
        } else {
            /* v2: signature presence depends on incompat_flags (byte 2),
               which we haven't received yet. Start without signature. */
            parser->expected_len =
                MAVLINK_V2_HEADER_LEN + byte + MAVLINK_CHECKSUM_LEN;
        }
        parser->state = MAVLINK_PARSE_ACCUMULATING;
        return MAVLINK_PARSER_INCOMPLETE;

    case MAVLINK_PARSE_ACCUMULATING:
        if (parser->idx >= MAVLINK_MAX_FRAME_LEN) {
            /* Frame overflow — discard and resync. */
            mavlink_parser_init(parser);
            return MAVLINK_PARSER_INCOMPLETE;
        }

        parser->buf[parser->idx++] = byte;

        /* DESIGN: for v2 frames, check incompat_flags (byte index 2) once
           available to detect whether a 13-byte signature is appended. */
        if (parser->buf[0] == MAVLINK_V2_STX && parser->idx == 3) {
            if (parser->buf[2] & 0x01) {
                parser->expected_len += MAVLINK_SIGNATURE_LEN;
            }
        }

        if (parser->idx >= parser->expected_len) {
            return MAVLINK_PARSER_COMPLETE;
        }
        return MAVLINK_PARSER_INCOMPLETE;
    }

    /* Unreachable, but satisfy compilers. */
    return MAVLINK_PARSER_INCOMPLETE;
}

void mavlink_parser_get_frame(const mavlink_parser_t *parser,
                              const uint8_t **buf, size_t *len)
{
    *buf = parser->buf;
    *len = parser->idx;
}
