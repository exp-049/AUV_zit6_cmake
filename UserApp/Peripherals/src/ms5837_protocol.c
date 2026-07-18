#include "ms5837_protocol.h"

#include <string.h>

static void ms5837_protocol_parser_reset(ms5837_protocol_parser_t *parser)
{
    parser->state = MS5837_PROTOCOL_PARSER_SOF1;
    parser->payload_index = 0U;
    parser->checksum = 0U;
}



static void ms5837_protocol_put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void ms5837_protocol_put_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

void ms5837_protocol_parser_init(ms5837_protocol_parser_t *parser)
{
    if (parser == 0) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    ms5837_protocol_parser_reset(parser);
}

int ms5837_protocol_parser_push(ms5837_protocol_parser_t *parser,
                                uint8_t byte,
                                ms5837_protocol_frame_t *frame_out)
{
    if (parser == 0 || frame_out == 0) {
        return -1;
    }

    switch (parser->state) {
    case MS5837_PROTOCOL_PARSER_SOF1:
        if (byte == MS5837_PROTOCOL_SOF1) {
            parser->state = MS5837_PROTOCOL_PARSER_SOF2;
        }
        break;

    case MS5837_PROTOCOL_PARSER_SOF2:
        if (byte == MS5837_PROTOCOL_SOF2) {
            parser->state = MS5837_PROTOCOL_PARSER_TYPE;
        } else if (byte == MS5837_PROTOCOL_SOF1) {
            parser->state = MS5837_PROTOCOL_PARSER_SOF2;
        } else {
            ms5837_protocol_parser_reset(parser);
        }
        break;

    case MS5837_PROTOCOL_PARSER_TYPE:
        parser->frame.type = byte;
        parser->checksum = byte;
        parser->state = MS5837_PROTOCOL_PARSER_LENGTH;
        break;

    case MS5837_PROTOCOL_PARSER_LENGTH:
        if (byte > MS5837_PROTOCOL_MAX_PAYLOAD) {
            ms5837_protocol_parser_reset(parser);
            return -1;
        }
        parser->frame.length = byte;
        parser->checksum ^= byte;
        parser->payload_index = 0U;
        parser->state = (byte == 0U) ? MS5837_PROTOCOL_PARSER_CHECKSUM
                                     : MS5837_PROTOCOL_PARSER_PAYLOAD;
        break;

    case MS5837_PROTOCOL_PARSER_PAYLOAD:
        parser->frame.payload[parser->payload_index++] = byte;
        parser->checksum ^= byte;
        if (parser->payload_index >= parser->frame.length) {
            parser->state = MS5837_PROTOCOL_PARSER_CHECKSUM;
        }
        break;

    case MS5837_PROTOCOL_PARSER_CHECKSUM:
        if (byte == parser->checksum) {
            *frame_out = parser->frame;
            ms5837_protocol_parser_reset(parser);
            return 1;
        }
        ms5837_protocol_parser_reset(parser);
        return -1;

    default:
        ms5837_protocol_parser_reset(parser);
        return -1;
    }

    return 0;
}

uint8_t ms5837_protocol_checksum(uint8_t type, uint8_t length,
                                 const uint8_t *payload)
{
    uint8_t checksum = type ^ length;
    uint8_t i;

    if (payload == 0 && length != 0U) {
        return 0U;
    }

    for (i = 0U; i < length; ++i) {
        checksum ^= payload[i];
    }
    return checksum;
}

uint8_t ms5837_protocol_encode_frame(uint8_t type,
                                      const uint8_t *payload,
                                      uint8_t payload_length,
                                      uint8_t *output,
                                      uint8_t output_size)
{
    uint8_t total_length;

    if (payload_length > MS5837_PROTOCOL_MAX_PAYLOAD ||
        output == 0 || output_size < (uint8_t)(payload_length + 5U) ||
        (payload_length != 0U && payload == 0)) {
        return 0U;
    }

    total_length = (uint8_t)(payload_length + 5U);
    output[0] = MS5837_PROTOCOL_SOF1;
    output[1] = MS5837_PROTOCOL_SOF2;
    output[2] = type;
    output[3] = payload_length;
    if (payload_length != 0U) {
        memcpy(&output[4], payload, payload_length);
    }
    output[4U + payload_length] = ms5837_protocol_checksum(type,
                                                            payload_length,
                                                            payload);
    return total_length;
}

uint8_t ms5837_protocol_encode_data(uint16_t depth_cm,
                                    uint32_t pressure_01mbar,
                                    int16_t temperature_01c,
                                    uint16_t sample_seq,
                                    uint8_t status,
                                    uint8_t *output,
                                    uint8_t output_size)
{
    uint8_t payload[11];

    ms5837_protocol_put_u16(&payload[0], depth_cm);
    ms5837_protocol_put_u32(&payload[2], pressure_01mbar);
    ms5837_protocol_put_u16(&payload[6], (uint16_t)temperature_01c);
    ms5837_protocol_put_u16(&payload[8], sample_seq);
    payload[10] = status & 0x07U;
    return ms5837_protocol_encode_frame(MS5837_PROTOCOL_TYPE_DATA,
                                         payload, sizeof(payload),
                                         output, output_size);
}

uint8_t ms5837_protocol_encode_rate_ack(uint8_t request_id,
                                        ms5837_protocol_result_t result,
                                        uint16_t requested_rate_hz,
                                        uint16_t applied_rate_hz,
                                        uint8_t *output,
                                        uint8_t output_size)
{
    uint8_t payload[6];

    payload[0] = request_id;
    payload[1] = (uint8_t)result;
    ms5837_protocol_put_u16(&payload[2], requested_rate_hz);
    ms5837_protocol_put_u16(&payload[4], applied_rate_hz);
    return ms5837_protocol_encode_frame(MS5837_PROTOCOL_TYPE_RATE_ACK,
                                         payload, sizeof(payload),
                                         output, output_size);
}

uint8_t ms5837_protocol_encode_handshake(uint32_t host_nonce,
                                         uint8_t *output,
                                         uint8_t output_size)
{
    uint8_t payload[4];

    ms5837_protocol_put_u32(payload, host_nonce);
    return ms5837_protocol_encode_frame(MS5837_PROTOCOL_TYPE_HANDSHAKE,
                                         payload, sizeof(payload),
                                         output, output_size);
}

uint8_t ms5837_protocol_encode_handshake_ack(uint32_t host_nonce,
                                             uint8_t device_state,
                                             uint16_t current_rate_hz,
                                             uint8_t sensor_flags,
                                             uint8_t *output,
                                             uint8_t output_size)
{
    uint8_t payload[9];

    ms5837_protocol_put_u32(&payload[0], host_nonce);
    payload[4] = MS5837_PROTOCOL_VERSION;
    payload[5] = device_state;
    ms5837_protocol_put_u16(&payload[6], current_rate_hz);
    payload[8] = sensor_flags & 0x07U;
    return ms5837_protocol_encode_frame(MS5837_PROTOCOL_TYPE_HANDSHAKE_ACK,
                                         payload, sizeof(payload),
                                         output, output_size);
}

int ms5837_protocol_decode_set_rate(const ms5837_protocol_frame_t *frame,
                                    uint8_t *request_id,
                                    uint16_t *requested_rate_hz)
{
    if (frame == 0 || request_id == 0 || requested_rate_hz == 0 ||
        frame->type != MS5837_PROTOCOL_TYPE_SET_RATE || frame->length != 3U) {
        return -1;
    }

    *request_id = frame->payload[0];
    *requested_rate_hz = (uint16_t)frame->payload[1] |
                         ((uint16_t)frame->payload[2] << 8);
    return 0;
}

int ms5837_protocol_decode_handshake(const ms5837_protocol_frame_t *frame,
                                     uint32_t *host_nonce)
{
    if (frame == 0 || host_nonce == 0 ||
        frame->type != MS5837_PROTOCOL_TYPE_HANDSHAKE || frame->length != 4U) {
        return -1;
    }

    *host_nonce = (uint32_t)frame->payload[0] |
                  ((uint32_t)frame->payload[1] << 8) |
                  ((uint32_t)frame->payload[2] << 16) |
                  ((uint32_t)frame->payload[3] << 24);
    return 0;
}
