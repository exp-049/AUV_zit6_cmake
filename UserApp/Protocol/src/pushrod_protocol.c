#include "pushrod_protocol.h"

static void pushrod_protocol_put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void pushrod_protocol_put_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static uint16_t pushrod_protocol_get_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t pushrod_protocol_get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static uint16_t pushrod_protocol_crc16_update(uint16_t crc, uint8_t data)
{
    uint8_t bit;

    crc ^= (uint16_t)data << 8;
    for (bit = 0U; bit < 8U; ++bit) {
        if ((crc & 0x8000U) != 0U) {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t pushrod_protocol_crc16_ccitt_false(const uint8_t *data,
                                            size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;

    if (data == 0 && length != 0U) {
        return 0U;
    }

    for (i = 0U; i < length; ++i) {
        crc = pushrod_protocol_crc16_update(crc, data[i]);
    }
    return crc;
}

uint16_t pushrod_protocol_task_crc(const pushrod_protocol_task_t *task)
{
    uint8_t bytes[12];

    if (task == 0) {
        return 0U;
    }

    pushrod_protocol_put_u32(&bytes[0], task->task_id);
    pushrod_protocol_put_u16(&bytes[4], (uint16_t)task->power_x1000);
    pushrod_protocol_put_u32(&bytes[6], task->duration_ms);
    bytes[10] = 0U;
    bytes[11] = 0U;

    /* Inner CRC covers TYPE, LEN, and payload bytes 0..9. */
    {
        uint16_t crc = 0xFFFFU;
        crc = pushrod_protocol_crc16_update(crc, PUSHROD_PROTOCOL_TYPE_TASK);
        crc = pushrod_protocol_crc16_update(
            crc, PUSHROD_PROTOCOL_TASK_PAYLOAD_LENGTH);
        for (uint8_t i = 0U; i < 10U; ++i) {
            crc = pushrod_protocol_crc16_update(crc, bytes[i]);
        }
        return crc;
    }
}

pushrod_protocol_result_t pushrod_protocol_validate_task(
    const pushrod_protocol_task_t *task)
{
    if (task == 0) {
        return PUSHROD_PROTOCOL_RESULT_INVALID_FRAME;
    }
    if (task->power_x1000 < PUSHROD_PROTOCOL_POWER_MIN_X1000 ||
        task->power_x1000 > PUSHROD_PROTOCOL_POWER_MAX_X1000) {
        return PUSHROD_PROTOCOL_RESULT_POWER_OUT_OF_RANGE;
    }
    if (task->duration_ms == 0U) {
        return PUSHROD_PROTOCOL_RESULT_DURATION_INVALID;
    }
    return PUSHROD_PROTOCOL_RESULT_OK;
}

uint8_t pushrod_protocol_encode_task(const pushrod_protocol_task_t *task,
                                      uint8_t *output,
                                      uint8_t output_size)
{
    uint8_t payload[PUSHROD_PROTOCOL_TASK_PAYLOAD_LENGTH];
    uint16_t crc;

    if (pushrod_protocol_validate_task(task) != PUSHROD_PROTOCOL_RESULT_OK ||
        output == 0 || output_size < PUSHROD_PROTOCOL_TASK_FRAME_SIZE) {
        return 0U;
    }

    pushrod_protocol_put_u32(&payload[0], task->task_id);
    pushrod_protocol_put_u16(&payload[4], (uint16_t)task->power_x1000);
    pushrod_protocol_put_u32(&payload[6], task->duration_ms);
    crc = pushrod_protocol_task_crc(task);
    pushrod_protocol_put_u16(&payload[10], crc);

    return ms5837_protocol_encode_frame(
        PUSHROD_PROTOCOL_TYPE_TASK, payload, sizeof(payload), output,
        output_size);
}

pushrod_protocol_result_t pushrod_protocol_decode_task(
    const ms5837_protocol_frame_t *frame,
    pushrod_protocol_task_t *task)
{
    pushrod_protocol_task_t decoded;
    uint16_t received_crc;

    if (frame == 0 || task == 0 ||
        frame->type != PUSHROD_PROTOCOL_TYPE_TASK ||
        frame->length != PUSHROD_PROTOCOL_TASK_PAYLOAD_LENGTH) {
        return PUSHROD_PROTOCOL_RESULT_INVALID_FRAME;
    }

    decoded.task_id = pushrod_protocol_get_u32(&frame->payload[0]);
    decoded.power_x1000 = (int16_t)pushrod_protocol_get_u16(&frame->payload[4]);
    decoded.duration_ms = pushrod_protocol_get_u32(&frame->payload[6]);
    received_crc = pushrod_protocol_get_u16(&frame->payload[10]);

    if (received_crc != pushrod_protocol_task_crc(&decoded)) {
        return PUSHROD_PROTOCOL_RESULT_CRC_ERROR;
    }

    {
        pushrod_protocol_result_t result =
            pushrod_protocol_validate_task(&decoded);
        if (result != PUSHROD_PROTOCOL_RESULT_OK) {
            return result;
        }
    }

    *task = decoded;
    return PUSHROD_PROTOCOL_RESULT_OK;
}

uint8_t pushrod_protocol_encode_ack(const pushrod_protocol_ack_t *ack,
                                     uint8_t *output,
                                     uint8_t output_size)
{
    uint8_t payload[PUSHROD_PROTOCOL_ACK_PAYLOAD_LENGTH];

    if (ack == 0 || output == 0 || output_size < PUSHROD_PROTOCOL_ACK_FRAME_SIZE) {
        return 0U;
    }

    pushrod_protocol_put_u32(&payload[0], ack->task_id);
    payload[4] = ack->result;
    payload[5] = ack->queue_count;
    payload[6] = ack->ready;
    return ms5837_protocol_encode_frame(
        PUSHROD_PROTOCOL_TYPE_ACK, payload, sizeof(payload), output,
        output_size);
}

int pushrod_protocol_decode_ack(const ms5837_protocol_frame_t *frame,
                                pushrod_protocol_ack_t *ack)
{
    if (frame == 0 || ack == 0 || frame->type != PUSHROD_PROTOCOL_TYPE_ACK ||
        frame->length != PUSHROD_PROTOCOL_ACK_PAYLOAD_LENGTH) {
        return -1;
    }

    ack->task_id = pushrod_protocol_get_u32(&frame->payload[0]);
    ack->result = frame->payload[4];
    ack->queue_count = frame->payload[5];
    ack->ready = frame->payload[6];
    return 0;
}
