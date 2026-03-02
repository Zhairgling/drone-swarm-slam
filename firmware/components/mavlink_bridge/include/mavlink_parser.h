#pragma once

#include <stddef.h>
#include <stdint.h>

#define MAVLINK_V1_STX 0xFE
#define MAVLINK_V2_STX 0xFD

#define MAVLINK_V1_HEADER_LEN  6  /* STX + LEN + SEQ + SYSID + COMPID + MSGID */
#define MAVLINK_V2_HEADER_LEN 10  /* STX + LEN + 2×FLAGS + SEQ + SYSID + COMPID + 3×MSGID */
#define MAVLINK_CHECKSUM_LEN   2
#define MAVLINK_SIGNATURE_LEN 13
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_MAX_FRAME_LEN \
    (MAVLINK_V2_HEADER_LEN + MAVLINK_MAX_PAYLOAD_LEN + \
     MAVLINK_CHECKSUM_LEN + MAVLINK_SIGNATURE_LEN) /* 280 */

typedef enum {
    MAVLINK_PARSE_IDLE,
    MAVLINK_PARSE_GOT_STX,
    MAVLINK_PARSE_ACCUMULATING,
} mavlink_parse_state_t;

typedef enum {
    MAVLINK_PARSER_INCOMPLETE,
    MAVLINK_PARSER_COMPLETE,
} mavlink_parser_result_t;

typedef struct {
    mavlink_parse_state_t state;
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t idx;
    size_t expected_len;
} mavlink_parser_t;

/* Reset the parser to its initial state. */
void mavlink_parser_init(mavlink_parser_t *parser);

/* Feed one byte to the parser.
   Returns MAVLINK_PARSER_COMPLETE when a full frame has been accumulated.
   After getting COMPLETE, call mavlink_parser_get_frame() then
   mavlink_parser_init() before parsing more bytes. */
mavlink_parser_result_t mavlink_parser_parse_byte(mavlink_parser_t *parser,
                                                  uint8_t byte);

/* Get the accumulated frame buffer and its length.
   Only valid after mavlink_parser_parse_byte() returns COMPLETE. */
void mavlink_parser_get_frame(const mavlink_parser_t *parser,
                              const uint8_t **buf, size_t *len);
