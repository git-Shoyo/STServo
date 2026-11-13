/** @file stservo_posix.cpp @brief Linuxのノンブロッキングシリアル入出力と単調増加時刻。 */
#include "stservo_posix.hpp"
#include <cerrno>
#include <chrono>
#include <climits>
#include <fcntl.h>
#include <poll.h>
#include <thread>
#include <unistd.h>

namespace {
bool baudConstant(std::uint32_t baud, speed_t& value) {
    switch (baud) {
    case 38400: value = B38400; return true;
    case 57600: value = B57600; return true;
    case 115200: value = B115200; return true;
#ifdef B76800
    case 76800: value = B76800; return true;
#endif
#ifdef B128000
    case 128000: value = B128000; return true;
#endif
#ifdef B250000
    case 250000: value = B250000; return true;
#endif
#ifdef B500000
    case 500000: value = B500000; return true;
#endif
#ifdef B1000000
    case 1000000: value = B1000000; return true;
#endif
    default: return false;
    }
}
}
namespace stservo {
PosixTransport::PosixTransport() : fd_(-1), error_(0), original_(), original_valid_(false) {}
PosixTransport::~PosixTransport() { close(); }
bool PosixTransport::open(const char* path, std::uint32_t baud) {
    if (fd_ >= 0) { error_ = EBUSY; return false; }
    speed_t speed;
    if (!path || !*path || !baudConstant(baud, speed)) { error_ = EINVAL; return false; }
    fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) { error_ = errno; return false; }
    if (tcgetattr(fd_, &original_) != 0) {
        const int saved = errno; close(); error_ = saved; return false;
    }
    original_valid_ = true;
    termios config = original_;
    cfmakeraw(&config);
    config.c_cflag |= CLOCAL | CREAD;
    config.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
    config.c_cflag &= ~CRTSCTS;
#endif
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 0;
    if (cfsetispeed(&config, speed) != 0 || cfsetospeed(&config, speed) != 0 ||
        tcsetattr(fd_, TCSANOW, &config) != 0 || tcflush(fd_, TCIOFLUSH) != 0) {
        const int saved = errno; close(); error_ = saved; return false;
    }
    error_ = 0;
    return true;
}
void PosixTransport::close() {
    if (fd_ >= 0) {
        if (original_valid_) (void)tcsetattr(fd_, TCSANOW, &original_);
        (void)::close(fd_);
    }
    fd_ = -1;
    original_valid_ = false;
}
bool PosixTransport::isOpen() const { return fd_ >= 0; }
int PosixTransport::errorNumber() const { return error_; }
bool PosixTransport::write(const std::uint8_t* data, std::size_t size, std::uint32_t timeout) {
    if (fd_ < 0) { error_ = EBADF; return false; }
    if (!data || !size || !timeout) { error_ = EINVAL; return false; }
    const auto start = nowMs();
    std::size_t sent = 0;
    while (sent < size) {
        const auto elapsed = static_cast<std::uint32_t>(nowMs() - start);
        if (elapsed >= timeout) { error_ = ETIMEDOUT; return false; }
        const auto left = timeout - elapsed;
        pollfd descriptor = {fd_, POLLOUT, 0};
        const int ready = ::poll(&descriptor, 1, left > INT_MAX ? INT_MAX : static_cast<int>(left));
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) { error_ = errno; return false; }
        if (!ready) { error_ = ETIMEDOUT; return false; }
        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) { error_ = EIO; return false; }
        const auto count = ::write(fd_, data + sent, size - sent);
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (count <= 0) { error_ = count < 0 ? errno : EIO; return false; }
        sent += static_cast<std::size_t>(count);
    }
    error_ = 0;
    return true;
}
int PosixTransport::readByte() {
    if (fd_ < 0) { error_ = EBADF; return -2; }
    pollfd descriptor = {fd_, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 0);
    if (ready < 0 && errno == EINTR) return -1;
    if (ready < 0) { error_ = errno; return -2; }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) { error_ = EIO; return -2; }
    if (!ready || !(descriptor.revents & POLLIN)) return -1;
    std::uint8_t byte;
    const auto count = ::read(fd_, &byte, 1);
    if (count == 1) return byte;
    if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))) return -1;
    error_ = errno;
    return -2;
}
bool PosixTransport::discardInput() {
    if (fd_ < 0) { error_ = EBADF; return false; }
    if (tcflush(fd_, TCIFLUSH) == 0) return true;
    error_ = errno;
    return false;
}
std::uint32_t PosixTransport::nowMs() const {
    return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
void PosixTransport::idle() { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
