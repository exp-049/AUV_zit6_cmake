/**
 * @file USBL_Driver.hpp
 * @brief USBL超短基线定位系统驱动类
 *
 * 职责：
 * 1. 通过串口（UART+DMA）接收并解析 USBL 数据包。
 * 2. 提取基阵姿态、斜距、能量及信标位置等数据。
 *
 * 协议参考：
 * - 帧尾：0xED 0xDE (字节 131, 132)
 * - 校验：异或校验 (从0字节到129字节，保存在130字节)
 * - 总帧长：133 字节
 */

#ifndef __USBL_DRIVER_HPP
#define __USBL_DRIVER_HPP

#include "SerialPort.hpp"
#include <string.h>

namespace auv {

/**
 * @struct UsblState
 * @brief 解析后的 USBL 数据结构体
 */
struct UsblState {
    float roll;             // 偏移 2: 基阵横滚 (度)
    float pitch;            // 偏移 6: 基阵俯仰 (度)
    float yaw;              // 偏移 10: 基阵航向 (度)
    uint8_t energy[8];      // 偏移 83-90: 30k-37k间隔1k信号的能量
    float beacon_north;     // 偏移 99: 信标北向位置 (m)
    float beacon_east;      // 偏移 103: 信标东向位置 (m)
    float beacon_depth;     // 偏移 107: 信标压力计数据/深度 (m)
    uint8_t sensor_status;  // 偏移 115: 传感器状态标志位
    uint8_t nav_mode;       // 偏移 129: 当前导航模式
    uint32_t timestamp;     // 系统接收时间戳 (ms)
};

/**
 * @class USBL_Driver
 * @brief USBL 驱动实现类
 */
class USBL_Driver {
public:
    /**
     * @brief 构造函数
     * @param rx_uart 接收数据的 UART 句柄 (对应 USART1)
     */
    explicit USBL_Driver(UART_HandleTypeDef* rx_uart)
        : rx_port_(rx_uart, 512) {}

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

private:
    SerialPort rx_port_;                               ///< 串口接收驱动

    static constexpr uint16_t kMaxFrameSize = 256;     ///< 缓冲区最大尺寸
    static constexpr uint16_t kTargetFrameSize = 133;  ///< 已知 USBL 帧长为 133 字节

    static constexpr uint8_t kHeader1 = 0xEB;
    static constexpr uint8_t kHeader2 = 0xBE;

    uint8_t packet_buf_[kMaxFrameSize] = {0};          ///< 帧解析临时缓冲区
    uint16_t frame_len_ = 0;                           ///< 当前解析长度

    UsblState state_{};                                ///< 缓存的最新有效数据

    // 内部私有方法
    uint8_t checkData(const uint8_t* data, uint8_t size);
    bool parseByte(uint8_t b);
    bool validateFrame();
    void decodePacket(UsblState& state);
};

} // namespace auv

#endif // __USBL_DRIVER_HPP