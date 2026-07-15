/**
 * @file USBL_Driver.hpp
 * @brief USBL超短基线定位系统驱动类
 *
 * 职责：
 * 1. 通过串口（UART+DMA）接收并解析 USBL 数据包。
 * 2. 提取基阵姿态、斜距、能量及信标位置等数据。
 *
 * 实际协议：
 * - 帧头：0xEA 0xAE (字节 0, 1)
 * - 帧尾：0xEB 0xBE (字节 131, 132)
 * - 校验：异或校验 (从 0 字节到 129 字节，保存在 130 字节)
 * - 总帧长：133 字节
 */

#ifndef __USBL_DRIVER_HPP
#define __USBL_DRIVER_HPP

#include <cstdint>

namespace auv {
namespace peripheral {

struct UsblPortDiagnostics;

/**
 * @struct UsblPortOps
 * @brief USBL 串口硬件操作接口
 */
struct UsblPortOps {
    void *ctx;
    bool (*init)(void *ctx);
    uint16_t (*read)(void *ctx, uint8_t *buf, uint16_t max_len);
    void (*getDiagnostics)(void *ctx, UsblPortDiagnostics *out);
    uint32_t (*getTickMs)(void *ctx);
};

/**
 * @struct UsblPortDiagnostics
 * @brief 供调试输出使用的串口/DMA快照，不参与协议解析。
 */
struct UsblPortDiagnostics {
    uint32_t events = 0;
    uint32_t invalid_events = 0;
    uint32_t valid_frames = 0;
    uint32_t invalid_frames = 0;
    uint16_t write_pos = 0;
    uint16_t dma_remaining = 0;
    bool dma_enabled = false;
    uint32_t uart_isr = 0;
    uint8_t rx_preview[4] = {};
};

/**
 * @struct UsblState
 * @brief 完整 USBL 数据帧的解析快照
 */
struct UsblState {
    float roll = 0.0f;              // 偏移 2: 基阵横滚 (度)
    float pitch = 0.0f;             // 偏移 6: 基阵俯仰 (度)
    float yaw = 0.0f;               // 偏移 10: 基阵航向 (度)
    float pressure = 0.0f;          // 偏移 14: 基阵压力计 (m)
    float slant_range[4] = {};      // 偏移 18..33: 四路斜距 (m)
    float latitude = 0.0f;          // 偏移 38: 信标纬度 (deg)
    float longitude = 0.0f;         // 偏移 42: 信标经度 (deg)
    float time_diff[3] = {};        // 偏移 46..57: 通道 12/13/14 时间差
    int16_t passive_attitude[3] = {}; // 偏移 59..64: 0.01 度
    uint16_t signal_strength[4] = {};  // 偏移 65..72
    uint8_t energy[8] = {};          // 偏移 83..90: 30..37 kHz
    float signal = 0.0f;             // 偏移 91: 综合信号强度
    float gain = 0.0f;               // 偏移 95: 模拟系统增益
    float beacon_north = 0.0f;       // 偏移 99: 信标北向位置 (m)
    float beacon_east = 0.0f;        // 偏移 103: 信标东向位置 (m)
    float beacon_depth = 0.0f;       // 偏移 107: 信标压力计/深度 (m)
    uint8_t sensor_status = 0;      // 偏移 115: 传感器状态位
    uint8_t year = 0;                // 偏移 116
    uint8_t month = 0;               // 偏移 117
    uint8_t day = 0;                 // 偏移 118
    uint8_t hour = 0;                // 偏移 119
    uint8_t minute = 0;              // 偏移 120
    float second = 0.0f;             // 偏移 121
    uint8_t nav_mode = 0;            // 偏移 129: 当前导航模式
    uint8_t checksum = 0;             // 偏移 130
    uint32_t timestamp = 0;           // 系统接收时间戳 (ms)
};

/**
 * @class USBL_Driver
 * @brief USBL 驱动实现类
 */
class USBL_Driver {
public:
    static constexpr uint16_t kFrameSize = 133;

    /**
     * @brief 构造函数
     * @param ops USBL 串口硬件操作接口
     */
    explicit USBL_Driver(UsblPortOps ops) : ops_(ops) {}

    /**
     * @brief 初始化驱动，启动非阻塞接收
     */
    void init();

    /**
     * @brief 驱动更新函数，建议在主循环中高频调用
     * @param[out] state 输出解码后的USBL状态
     * @return true 表示成功解析到一个完整的新帧
     */
    bool update(UsblState& state);

    /**
     * @brief 获取最新解析到的 USBL 数据快照
     */
    UsblState getUsblState() const { return state_; }

    /** @brief 拷贝最近一次校验通过的原始帧，供 Debug 输出使用。 */
    uint16_t copyLastFrame(uint8_t *dst, uint16_t max_len) const;

    /** @brief 获取驱动和底层 DMA 的诊断快照。 */
    void getDiagnostics(UsblPortDiagnostics &out) const;

private:
    UsblPortOps ops_;

    static constexpr uint16_t kMaxFrameSize = 256;     ///< 缓冲区最大尺寸
    static constexpr uint16_t kTargetFrameSize = 133;  ///< 已知 USBL 帧长为 133 字节

    static constexpr uint8_t kHeader1 = 0xEA;
    static constexpr uint8_t kHeader2 = 0xAE;
    static constexpr uint8_t kTail1 = 0xEB;
    static constexpr uint8_t kTail2 = 0xBE;

    uint8_t packet_buf_[kMaxFrameSize] = {0};          ///< 帧解析临时缓冲区
    uint16_t frame_len_ = 0;                           ///< 当前解析长度

    UsblState state_{};                                ///< 缓存的最新有效数据
    uint32_t valid_frames_ = 0;
    uint32_t invalid_frames_ = 0;

    // 内部私有方法
    uint8_t checkData(const uint8_t* data, uint16_t size);
    bool parseByte(uint8_t b);
    bool validateFrame();
    void decodePacket(UsblState& state);
};

} // namespace peripheral
} // namespace auv

#endif // __USBL_DRIVER_HPP
