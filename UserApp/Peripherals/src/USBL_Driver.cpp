#include "USBL_Driver.hpp"

namespace auv {

namespace {
static constexpr uint8_t kAcousticValidBit = 1u << 1;
static constexpr uint8_t kGpsValidBit = 1u << 3;
static constexpr uint8_t kUsblValidBit = 1u << 5;
}

void USBL_Driver::init() {
    rx_port_.startReceive();
}

bool USBL_Driver::update(UsblState& state) {
    uint8_t temp_buf[256];
    uint16_t len = rx_port_.read(temp_buf, 256);

    if (len > 0) {
        for (uint16_t i = 0; i < len; i++) {
            if (parseByte(temp_buf[i])) {
                if (validateFrame()) {
                    decodePacket(state);
                    state = state_;
                    frame_len_ = 0;
                    return true;
                }
            }
        }
    }
    return false;
}

uint8_t USBL_Driver::checkData(const uint8_t* data, uint8_t size) {
    uint8_t v = data[0];
    for (uint8_t i = 1; i < size; ++i) {
        v ^= data[i];
    }
    return v;
}

bool USBL_Driver::parseByte(uint8_t b) {
    if (frame_len_ == 0) {
        if (b == kHeader1) {
            packet_buf_[frame_len_++] = b;
        }
        return false;
    }

    if (frame_len_ == 1) {
        if (b == kHeader2) {
            packet_buf_[frame_len_++] = b;
        } else if (b == kHeader1) {
            packet_buf_[0] = kHeader1;
            frame_len_ = 1;
        } else {
            frame_len_ = 0;
        }
        return false;
    }

    if (packet_buf_[frame_len_ - 1] == kHeader1 && b == kHeader2) {
        packet_buf_[0] = kHeader1;
        packet_buf_[1] = kHeader2;
        frame_len_ = 2;
        return false;
    }

    if (frame_len_ >= kMaxFrameSize) {
        frame_len_ = 0;
        return false;
    }

    packet_buf_[frame_len_++] = b;

    return (frame_len_ >= 2 &&
            packet_buf_[frame_len_ - 2] == 0xED &&
            packet_buf_[frame_len_ - 1] == 0xDE);
}

bool USBL_Driver::validateFrame() {
    if (frame_len_ != kTargetFrameSize) {
        frame_len_ = 0;
        return false;
    }

    if (packet_buf_[0] != kHeader1 || packet_buf_[1] != kHeader2) {
        frame_len_ = 0;
        return false;
    }

    if (checkData(packet_buf_, 130) != packet_buf_[130]) {
        frame_len_ = 0;
        return false;
    }

    return true;
}

void USBL_Driver::decodePacket(UsblState& s) {
    state_.sensor_status = packet_buf_[115];
    state_.nav_mode = packet_buf_[129];

    const bool acoustic_valid = (state_.sensor_status & kAcousticValidBit) != 0;
    const bool gps_valid = (state_.sensor_status & kGpsValidBit) != 0;
    const bool usbl_valid = (state_.sensor_status & kUsblValidBit) != 0;

    if (acoustic_valid) {
        memcpy(&state_.roll,  packet_buf_ + 2,  4);
        memcpy(&state_.pitch, packet_buf_ + 6,  4);
        memcpy(&state_.yaw,   packet_buf_ + 10, 4);
        memcpy(state_.energy, packet_buf_ + 83, 8);
    }

    if (gps_valid) {
        memcpy(&state_.beacon_north, packet_buf_ + 99, 4);
        memcpy(&state_.beacon_east,  packet_buf_ + 103, 4);
    }

    if (usbl_valid || acoustic_valid) {
        memcpy(&state_.beacon_depth, packet_buf_ + 107, 4);
    }

    state_.timestamp = HAL_GetTick();

    s = state_;
}

} // namespace auv
