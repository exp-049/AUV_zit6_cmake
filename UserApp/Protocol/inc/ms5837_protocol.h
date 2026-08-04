#ifndef MS5837_PROTOCOL_H
#define MS5837_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MS5837_PROTOCOL_SOF1             0xA5U
#define MS5837_PROTOCOL_SOF2             0x5AU
#define MS5837_PROTOCOL_VERSION          0x01U
#define MS5837_PROTOCOL_MAX_PAYLOAD      32U
#define MS5837_PROTOCOL_MAX_FRAME_SIZE   (MS5837_PROTOCOL_MAX_PAYLOAD + 5U)

#ifndef MS5837_PROTOCOL_USE_DMA_IDLE
#define MS5837_PROTOCOL_USE_DMA_IDLE     1U
#endif

#define MS5837_PROTOCOL_TYPE_DATA          0x01U
#define MS5837_PROTOCOL_TYPE_SET_RATE      0x10U
#define MS5837_PROTOCOL_TYPE_HANDSHAKE     0x20U
#define MS5837_PROTOCOL_TYPE_RATE_ACK      0x90U
#define MS5837_PROTOCOL_TYPE_HANDSHAKE_ACK 0xA0U

#define MS5837_PROTOCOL_STATUS_SENSOR_OK  (1U << 0)
#define MS5837_PROTOCOL_STATUS_CALIBRATED (1U << 1)
#define MS5837_PROTOCOL_STATUS_DATA_NEW   (1U << 2)

#define MS5837_PROTOCOL_RATE_MIN_HZ      15U
#define MS5837_PROTOCOL_RATE_MAX_HZ      150U
#define MS5837_PROTOCOL_DEFAULT_RATE_HZ  60U

/* Host/device liveness contract used by the watchdog adapter. */
#define MS5837_PROTOCOL_HANDSHAKE_INTERVAL_MS  2000U
#define MS5837_PROTOCOL_WATCHDOG_TIMEOUT_MS    5000U

typedef enum {
    MS5837_PROTOCOL_RESULT_OK = 0x00U,
    MS5837_PROTOCOL_RESULT_OUT_OF_RANGE = 0x01U,
    MS5837_PROTOCOL_RESULT_INVALID = 0x02U,
    MS5837_PROTOCOL_RESULT_BUSY = 0x03U
} ms5837_protocol_result_t;

typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t payload[MS5837_PROTOCOL_MAX_PAYLOAD];
} ms5837_protocol_frame_t;

typedef enum {
    MS5837_PROTOCOL_PARSER_SOF1 = 0,
    MS5837_PROTOCOL_PARSER_SOF2,
    MS5837_PROTOCOL_PARSER_TYPE,
    MS5837_PROTOCOL_PARSER_LENGTH,
    MS5837_PROTOCOL_PARSER_PAYLOAD,
    MS5837_PROTOCOL_PARSER_CHECKSUM
} ms5837_protocol_parser_state_t;

typedef struct {
    ms5837_protocol_parser_state_t state;
    ms5837_protocol_frame_t frame;
    uint8_t payload_index;
    uint8_t checksum;
} ms5837_protocol_parser_t;

void ms5837_protocol_parser_init(ms5837_protocol_parser_t *parser);

int ms5837_protocol_parser_push(ms5837_protocol_parser_t *parser,
                                uint8_t byte,
                                ms5837_protocol_frame_t *frame_out);

uint8_t ms5837_protocol_checksum(uint8_t type, uint8_t length,
                                 const uint8_t *payload);

uint8_t ms5837_protocol_encode_frame(uint8_t type,
                                     const uint8_t *payload,
                                     uint8_t payload_length,
                                     uint8_t *output,
                                     uint8_t output_size);

uint8_t ms5837_protocol_encode_data(uint16_t depth_cm,
                                    uint32_t pressure_01mbar,
                                    int16_t temperature_01c,
                                    uint16_t sample_seq,
                                    uint8_t status,
                                    uint8_t *output,
                                    uint8_t output_size);

uint8_t ms5837_protocol_encode_rate_ack(uint8_t request_id,
                                        ms5837_protocol_result_t result,
                                        uint16_t requested_rate_hz,
                                        uint16_t applied_rate_hz,
                                        uint8_t *output,
                                        uint8_t output_size);

uint8_t ms5837_protocol_encode_handshake(uint32_t host_nonce,
                                         uint8_t *output,
                                         uint8_t output_size);

uint8_t ms5837_protocol_encode_handshake_ack(uint32_t host_nonce,
                                             uint8_t device_state,
                                             uint16_t current_rate_hz,
                                             uint8_t sensor_flags,
                                             uint8_t *output,
                                             uint8_t output_size);

int ms5837_protocol_decode_set_rate(const ms5837_protocol_frame_t *frame,
                                    uint8_t *request_id,
                                    uint16_t *requested_rate_hz);

int ms5837_protocol_decode_handshake(const ms5837_protocol_frame_t *frame,
                                     uint32_t *host_nonce);

#ifdef __cplusplus
}
#endif

#endif /* MS5837_PROTOCOL_H */
