#pragma once

#include <string>
#include <cstdint>
#include <functional>
#include <thread>
#include <atomic>

namespace uwb_align {

/**
 * @brief UWB TOF Report Message 解析结果
 *
 * 格式示例: mc 0f 00000663 000005a3 00000512 000004cb 095f c1 00024c24 a0:0
 * 字段说明见 6.3 TOF Report Message 协议文档。
 */
struct TofRange {
    uint8_t  mask        = 0;          ///< 有效 range 位掩码 (bit0=RANGE0 有效, bit1=RANGE1, ...)
    uint32_t range_mm[4] = {0,0,0,0};  ///< 标签到各基站的距离 (毫米), 仅 mask 对应位为 1 时有意义
    uint8_t  tag_id      = 0;          ///< 标签 short ID
    uint8_t  base_id     = 0;          ///< 基站 short ID (aT:A 格式中的 A)
};

/**
 * @brief UWB 串口读取与 TOF 协议解析
 *
 * 以 460800 8N1 打开串口，逐行读取并解析 "mc ..." 格式报文，
 * 通过回调将解析结果传递给调用者（运行在独立后台线程中）。
 */
class UwbSerial {
public:
    using RangeCb = std::function<void(const TofRange&)>;

    UwbSerial();
    ~UwbSerial();

    /**
     * @brief 打开串口（8N1，无硬件流控）
     * @param port     串口设备路径, 如 /dev/ttyUSB0
     * @param baud_rate 波特率, 默认 460800
     * @return true 成功, false 失败
     */
    bool open(const std::string& port, int baud_rate = 460800);

    /** @brief 关闭串口 */
    void close();

    /** @brief 设置解析结果回调（须在 startReading 之前调用）*/
    void setCallback(RangeCb cb);

    /** @brief 启动后台读取线程 */
    bool startReading();

    /** @brief 停止后台读取线程（阻塞直至线程退出）*/
    void stopReading();

    /** @brief 串口是否已打开 */
    bool isOpen() const { return fd_ >= 0; }

private:
    void readLoop();

    /**
     * @brief 解析单行文本为 TofRange
     * @return true 解析成功（MID=="mc" 且字段数量正确）
     */
    static bool parseLine(const std::string& line, TofRange& out);

    int              fd_      = -1;
    std::atomic<bool> running_{false};
    std::thread      read_thread_;
    RangeCb          callback_;
};

}  // namespace uwb_align
