#ifndef PUSHROD_PROTOCOL_H
#define PUSHROD_PROTOCOL_H

#include "ms5837_protocol.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PUSHROD_PROTOCOL_TYPE_TASK 0x30U
#define PUSHROD_PROTOCOL_TYPE_ACK  0xB0U

#define PUSHROD_PROTOCOL_TASK_PAYLOAD_LENGTH 12U
#define PUSHROD_PROTOCOL_ACK_PAYLOAD_LENGTH  7U
#define PUSHROD_PROTOCOL_TASK_FRAME_SIZE \
    (PUSHROD_PROTOCOL_TASK_PAYLOAD_LENGTH + 5U)
#define PUSHROD_PROTOCOL_ACK_FRAME_SIZE \
    (PUSHROD_PROTOCOL_ACK_PAYLOAD_LENGTH + 5U)

#define PUSHROD_PROTOCOL_POWER_MIN_X1000 (-1000)
#define PUSHROD_PROTOCOL_POWER_MAX_X1000 (1000)

typedef enum {
    PUSHROD_PROTOCOL_RESULT_OK = 0x00U,
    PUSHROD_PROTOCOL_RESULT_INVALID_FRAME = 0x01U,
    PUSHROD_PROTOCOL_RESULT_CRC_ERROR = 0x02U,
    PUSHROD_PROTOCOL_RESULT_POWER_OUT_OF_RANGE = 0x03U,
    PUSHROD_PROTOCOL_RESULT_DURATION_INVALID = 0x04U,
    PUSHROD_PROTOCOL_RESULT_ID_OUT_OF_ORDER = 0x05U,
    PUSHROD_PROTOCOL_RESULT_ID_CONFLICT = 0x06U,
    PUSHROD_PROTOCOL_RESULT_QUEUE_FULL = 0x07U,
    PUSHROD_PROTOCOL_RESULT_NOT_INITIALIZED = 0x08U
} pushrod_protocol_result_t;

typedef struct {
    uint32_t task_id;
    int16_t power_x1000;
    uint32_t duration_ms;
} pushrod_protocol_task_t;

typedef struct {
    uint32_t task_id;
    uint8_t result;
    uint8_t queue_count;
    uint8_t ready;
} pushrod_protocol_ack_t;

/** CRC-16/CCITT-FALSE: init 0xFFFF, polynomial 0x1021. */
uint16_t pushrod_protocol_crc16_ccitt_false(const uint8_t *data,
                                            size_t length);

/** Calculate the inner task CRC over TYPE, LEN, and payload bytes 0..9. */
uint16_t pushrod_protocol_task_crc(const pushrod_protocol_task_t *task);

/** Validate task fields without encoding a frame. */
pushrod_protocol_result_t pushrod_protocol_validate_task(
    const pushrod_protocol_task_t *task);

/**
 * Encode a complete TYPE=0x30 pushrod task frame.
 * Returns the frame length (17) on success, or 0 for invalid input/output.
 */
uint8_t pushrod_protocol_encode_task(const pushrod_protocol_task_t *task,
                                      uint8_t *output,
                                      uint8_t output_size);

/**
 * Decode and validate a complete TYPE=0x30 task frame.
 * Returns OK or the detailed task error code.
 */
pushrod_protocol_result_t pushrod_protocol_decode_task(
    const ms5837_protocol_frame_t *frame,
    pushrod_protocol_task_t *task);

/** Encode a TYPE=0xB0 task acknowledgement frame. */
uint8_t pushrod_protocol_encode_ack(const pushrod_protocol_ack_t *ack,
                                     uint8_t *output,
                                     uint8_t output_size);

/** Decode a TYPE=0xB0 task acknowledgement frame. */
int pushrod_protocol_decode_ack(const ms5837_protocol_frame_t *frame,
                                pushrod_protocol_ack_t *ack);

#ifdef __cplusplus
}
#endif

#endif /* PUSHROD_PROTOCOL_H */
