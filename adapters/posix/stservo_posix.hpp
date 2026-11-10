/** @file stservo_posix.hpp @brief Linux termios transport for Raspberry Pi / USB-UART. */
#pragma once
#include "STServo/stservo.hpp"
#include <termios.h>

namespace stservo {
/** @brief Owns a nonblocking 8N1 serial file descriptor.
 * @details This adapter uses OS-buffered full-duplex UART; an external servo
 * interface must handle single-wire direction. It does not bit-bang Pi GPIOs.
 * UART/USB device selection and hardware setup belong to the application.
 * It restores saved terminal settings on close. Not thread-safe.
 */
class PosixTransport final : public Transport {
public:
    /** @brief Construct a closed port. */
    PosixTransport();
    /** @brief Restore saved terminal settings and release the descriptor. */
    ~PosixTransport() override;
    PosixTransport(const PosixTransport&) = delete;
    PosixTransport& operator=(const PosixTransport&) = delete;
    /** @brief Open an existing device (e.g. /dev/ttyAMA0) and configure raw 8N1.
     * @return False on failure; errorNumber() contains the errno value.
     * @note A call while already open fails with EBUSY and preserves the port.
     */
    bool open(const char* device, std::uint32_t baud = 1000000);
    /** @brief Close the owned descriptor; safe to call repeatedly. */
    void close();
    /** @brief Whether a serial descriptor is currently owned. */
    bool isOpen() const;
    /** @brief Most recently recorded system error (errno value). */
    int errorNumber() const;
    bool write(const std::uint8_t* data, std::size_t size, std::uint32_t timeout_ms) override;
    int readByte() override;
    bool discardInput() override;
    std::uint32_t nowMs() const override;
    void idle() override;
private:
    int fd_;
    int error_;
    termios original_;
    bool original_valid_;
};
}
