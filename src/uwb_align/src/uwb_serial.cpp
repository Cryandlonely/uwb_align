#include "uwb_align/uwb_serial.h"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

#include <sstream>
#include <stdexcept>
#include <cstring>
#include <cerrno>

namespace uwb_align {

// ---- 波特率整数转 termios 常量 ----
static speed_t baudToConst(int baud)
{
    switch (baud) {
        case 460800: return B460800;
        case 230400: return B230400;
        case 115200: return B115200;
        case 57600:  return B57600;
        case 38400:  return B38400;
        case 19200:  return B19200;
        case 9600:   return B9600;
        default:     return B460800;
    }
}

// ============================================================
UwbSerial::UwbSerial() = default;

UwbSerial::~UwbSerial()
{
    stopReading();
    close();
}

bool UwbSerial::open(const std::string& port, int baud_rate)
{
    // O_RDWR | O_NOCTTY: 读写 + 不分配控制终端
    // O_NONBLOCK: 非阻塞打开（避免挂起），后续切回阻塞
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }
    // 恢复阻塞模式
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 波特率
    speed_t spd = baudToConst(baud_rate);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    // 原始模式：无 echo、无信号、无特殊字符处理
    cfmakeraw(&tty);

    // 8 数据位, 1 停止位, 无校验
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;   // 无校验
    tty.c_cflag &= ~CSTOPB;   // 1 停止位
    tty.c_cflag &= ~CRTSCTS;  // 无硬件流控
    tty.c_cflag |= (CLOCAL | CREAD);

    // read() 至少等到 1 个字节，超时 100ms
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;  // 0.1 s

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void UwbSerial::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void UwbSerial::setCallback(RangeCb cb)
{
    callback_ = std::move(cb);
}

bool UwbSerial::startReading()
{
    if (fd_ < 0 || running_) return false;
    running_ = true;
    read_thread_ = std::thread(&UwbSerial::readLoop, this);
    return true;
}

void UwbSerial::stopReading()
{
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

// ---- 后台读取循环 ----
void UwbSerial::readLoop()
{
    std::string line_buf;
    line_buf.reserve(128);

    while (running_) {
        char c = 0;
        ssize_t n = ::read(fd_, &c, 1);
        if (n <= 0) {
            // 超时或错误，继续循环
            continue;
        }

        if (c == '\n' || c == '\r') {
            if (!line_buf.empty()) {
                TofRange range{};
                if (parseLine(line_buf, range) && callback_) {
                    callback_(range);
                }
                line_buf.clear();
            }
        } else if (c >= 0x20) {
            // 仅保留可打印字符，防止控制字符污染解析
            line_buf += c;
        }
    }
}

// ---- TOF 协议解析 ----
// 格式: mc 0f 00000663 000005a3 00000512 000004cb 095f c1 00024c24 a0:0
//  [0]  [1]    [2]       [3]       [4]       [5]   [6] [7]   [8]    [9]
//  MID MASK  RANGE0    RANGE1    RANGE2    RANGE3  NR  RSEQ  DBG   aT:A
bool UwbSerial::parseLine(const std::string& line, TofRange& out)
{
    std::istringstream ss(line);
    std::string tokens[10];

    for (int i = 0; i < 10; ++i) {
        if (!(ss >> tokens[i])) {
            return false;  // 字段数不足
        }
    }

    // MID 必须为 "mc"（标签−基站优化距离报告）
    if (tokens[0] != "mc") {
        return false;
    }

    try {
        out.mask = static_cast<uint8_t>(std::stoul(tokens[1], nullptr, 16));
        for (int i = 0; i < 4; ++i) {
            out.range_mm[i] = static_cast<uint32_t>(std::stoul(tokens[2 + i], nullptr, 16));
        }

        // 解析 aT:A，例如 "a0:3" -> tag_id=0, base_id=3
        // tokens[9] 格式: a<tag>:<base>
        const std::string& at = tokens[9];
        if (at.size() >= 3 && at[0] == 'a') {
            std::string ids = at.substr(1);  // "<tag>:<base>"
            const auto colon = ids.find(':');
            if (colon != std::string::npos) {
                out.tag_id  = static_cast<uint8_t>(std::stoul(ids.substr(0, colon)));
                out.base_id = static_cast<uint8_t>(std::stoul(ids.substr(colon + 1)));
            }
        }
    } catch (const std::exception&) {
        return false;
    }

    return true;
}

}  // namespace uwb_align
