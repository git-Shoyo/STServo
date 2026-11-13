/** @file stservo.cpp @brief FT-SCSのフレーム処理と、複数IDへの同期的な通信処理。 */
#include "STServo/stservo.hpp"
#include <cstring>

constexpr std::uint8_t STServo::MaxId;
constexpr std::uint8_t STServo::BroadcastId;
constexpr std::size_t STServo::MaxPacketSize;
constexpr std::size_t STServo::MaxReadSize;
constexpr std::size_t STServo::MaxWriteSize;
constexpr std::size_t STServo::MaxSyncVelocityServos;

namespace {
bool validTimeout(std::uint32_t value) { return value && value <= 0x7fffffffU; }
class Parser {
public:
    std::uint8_t bytes[STServo::MaxPacketSize] = {};
    std::size_t count = 0;
    std::size_t expected = 0;
    void reset() { count = expected = 0; }
    STServo::Result feed(std::uint8_t value, bool& complete) {
        complete = false;
        if (count < 2) {
            if (value == 0xff) bytes[count++] = value;
            else reset();
            return STServo::Result::OK;
        }
        // Extra FF preamble bytes cannot be IDs; preserve the header.
        if (count == 2 && value == 0xff) return STServo::Result::OK;
        bytes[count++] = value;
        if (count == 4) {
            expected = static_cast<std::size_t>(value) + 4;
            if (value < 2 || expected > sizeof(bytes)) {
                reset();
                return STServo::Result::LENGTH_ERROR;
            }
        }
        if (expected && count == expected) {
            complete = true;
            std::uint8_t sum = 0;
            for (std::size_t i = 2; i < count; ++i) sum += bytes[i];
            if (sum != 0xff) { reset(); return STServo::Result::CHECKSUM_ERROR; }
        }
        return STServo::Result::OK;
    }
};
}

STServo::Reply::Reply(Result result_, std::uint8_t id_)
    : result(result_), id(id_), servo_error(0), status_received(false), parameter_count(0) {}
bool STServo::Reply::ok() const { return result == Result::OK; }
STServo::STServo(stservo::Transport& port, std::uint32_t timeout, EchoMode echo)
    : transport_(port), timeout_ms_(timeout), echo_(echo) {}
bool STServo::setTimeoutMs(std::uint32_t timeout) {
    if (!validTimeout(timeout)) return false;
    timeout_ms_ = timeout;
    return true;
}
std::uint32_t STServo::timeoutMs() const { return timeout_ms_; }
void STServo::setEchoMode(EchoMode mode) { echo_ = mode; }
STServo::EchoMode STServo::echoMode() const { return echo_; }

STServo::Reply STServo::ping(std::uint8_t id) {
    if (id > MaxId) return Reply(Result::INVALID_ARGUMENT, id);
    std::uint8_t frame[] = {0xff, 0xff, id, 2, 1, 0};
    genCheckSum(frame, sizeof(frame));
    return exchange(id, frame, sizeof(frame), true, 0, nullptr);
}
STServo::Reply STServo::readRegister(std::uint8_t id, std::uint8_t address,
                                   std::uint8_t* output, std::size_t size) {
    if (id > MaxId || !output || !size || size > 256U - address)
        return Reply(Result::INVALID_ARGUMENT, id);
    if (size > MaxReadSize) return Reply(Result::BUFFER_ERROR, id);
    std::uint8_t frame[] = {0xff, 0xff, id, 4, 2, address, static_cast<std::uint8_t>(size), 0};
    genCheckSum(frame, sizeof(frame));
    return exchange(id, frame, sizeof(frame), true, size, output);
}
STServo::Reply STServo::writeRegister(std::uint8_t id, std::uint8_t address,
                                    const std::uint8_t* data, std::size_t size,
                                    WriteResponse response) {
    if (id > BroadcastId || !data || !size || size > 256U - address ||
        (id == BroadcastId && response == WriteResponse::Required))
        return Reply(Result::INVALID_ARGUMENT, id);
    if (size > MaxWriteSize) return Reply(Result::BUFFER_ERROR, id);
    std::uint8_t frame[MaxPacketSize] = {0xff, 0xff, id,
        static_cast<std::uint8_t>(size + 3), 3, address};
    std::memcpy(frame + 6, data, size);
    genCheckSum(frame, size + 7);
    return exchange(id, frame, size + 7, response == WriteResponse::Required, 0, nullptr);
}
STServo::Result STServo::syncWriteRegister(std::uint8_t address, std::uint8_t width,
                                         const std::uint8_t* ids, const std::uint8_t* data,
                                         std::size_t count) {
    if (!ids || !data || !count || !width || width > 256U - address)
        return Result::INVALID_ARGUMENT;
    if (count > (MaxPacketSize - 8) / (static_cast<std::size_t>(width) + 1))
        return Result::BUFFER_ERROR;
    for (std::size_t i = 0; i < count; ++i) {
        if (ids[i] > MaxId) return Result::INVALID_ARGUMENT;
        for (std::size_t j = 0; j < i; ++j)
            if (ids[j] == ids[i]) return Result::INVALID_ARGUMENT;
    }
    std::uint8_t frame[MaxPacketSize] = {0xff, 0xff, BroadcastId, 0, 0x83, address, width};
    std::size_t position = 7;
    for (std::size_t i = 0; i < count; ++i) {
        frame[position++] = ids[i];
        std::memcpy(frame + position, data + i * width, width);
        position += width;
    }
    frame[3] = static_cast<std::uint8_t>(position - 3);
    genCheckSum(frame, position + 1);
    return exchange(BroadcastId, frame, position + 1, false, 0, nullptr).result;
}
STServo::Reply STServo::setTorqueEnabled(std::uint8_t id, bool enabled, WriteResponse response) {
    const std::uint8_t value = enabled ? 1 : 0;
    return writeRegister(id, 0x28, &value, 1, response);
}
STServo::Reply STServo::setPositionMode(std::uint8_t id, WriteResponse response) {
    const std::uint8_t mode = 0;
    return writeRegister(id, 0x21, &mode, 1, response);
}
STServo::Reply STServo::setPosition(std::uint8_t id, std::uint16_t position, WriteResponse response) {
    if (position > 4095) return Reply(Result::INVALID_ARGUMENT, id);
    const std::uint8_t data[] = {static_cast<std::uint8_t>(position),
                               static_cast<std::uint8_t>(position >> 8)};
    return writeRegister(id, 0x2a, data, sizeof(data), response);
}
STServo::Reply STServo::setId(std::uint8_t id, std::uint8_t new_id) {
    if (id > MaxId || new_id > MaxId) return Reply(Result::INVALID_ARGUMENT, id);
    return writeRegister(id, 0x05, &new_id, 1, WriteResponse::None);
}
STServo::Reply STServo::setEepromLocked(std::uint8_t id, bool locked, WriteResponse response) {
    const std::uint8_t value = locked ? 1 : 0;
    return writeRegister(id, 0x37, &value, 1, response);
}
STServo::Reply STServo::setVelocityMode(std::uint8_t id, WriteResponse response) {
    const std::uint8_t mode = 1;
    return writeRegister(id, 0x21, &mode, 1, response);
}
void STServo::encodeVelocity(std::int32_t velocity, std::uint8_t* output) {
    std::int64_t magnitude = velocity;
    if (magnitude < 0) magnitude = -magnitude;
    if (magnitude > 0x7fff) magnitude = 0x7fff;
    std::uint16_t encoded = static_cast<std::uint16_t>(magnitude);
    if (velocity < 0) encoded |= 0x8000;
    output[0] = static_cast<std::uint8_t>(encoded);
    output[1] = static_cast<std::uint8_t>(encoded >> 8);
}
STServo::Reply STServo::setVelocity(std::uint8_t id, std::int32_t velocity, WriteResponse response) {
    std::uint8_t data[2];
    encodeVelocity(velocity, data);
    return writeRegister(id, 0x2e, data, sizeof(data), response);
}
STServo::Reply STServo::sts_exchange(std::uint8_t id, std::int32_t velocity) {
    return setVelocity(id, velocity);
}
STServo::Result STServo::syncSetVelocity(const std::uint8_t* ids,
                                      const std::int32_t* velocities, std::size_t count) {
    if (!ids || !velocities || !count) return Result::INVALID_ARGUMENT;
    if (count > MaxSyncVelocityServos) return Result::BUFFER_ERROR;
    std::uint8_t data[MaxSyncVelocityServos * 2];
    for (std::size_t i = 0; i < count; ++i) encodeVelocity(velocities[i], data + i * 2);
    return syncWriteRegister(0x2e, 2, ids, data, count);
}
STServo::Reply STServo::exchange(std::uint8_t id, const std::uint8_t* frame, std::size_t size,
                               bool expect_status, std::size_t expected_parameters,
                               std::uint8_t* output) {
    Reply reply(Result::TIMEOUT, id);
    if (!validTimeout(timeout_ms_)) { reply.result = Result::INVALID_ARGUMENT; return reply; }
    const std::uint32_t start = transport_.nowMs();
    if (!transport_.discardInput()) { reply.result = Result::TRANSPORT_ERROR; return reply; }
    std::uint32_t elapsed = transport_.nowMs() - start;
    if (elapsed >= timeout_ms_) return reply;
    if (!transport_.write(frame, size, timeout_ms_ - elapsed)) {
        reply.result = Result::TRANSPORT_ERROR;
        return reply;
    }
    if (!expect_status && echo_ == EchoMode::None) { reply.result = Result::OK; return reply; }
    Parser parser;
    std::size_t echo_position = 0;
    const bool has_echo = echo_ == EchoMode::LocalEcho;
    Result last_error = Result::TIMEOUT;
    while (static_cast<std::uint32_t>(transport_.nowMs() - start) < timeout_ms_) {
        const int value = transport_.readByte();
        if (value <= -2 || value > 255) { reply.result = Result::TRANSPORT_ERROR; return reply; }
        if (value == -1) { transport_.idle(); continue; }
        const auto byte = static_cast<std::uint8_t>(value);
        if (has_echo && echo_position < size) {
            if (byte != frame[echo_position++]) { reply.result = Result::ECHO_ERROR; return reply; }
            if (echo_position == size) {
                if (!expect_status) { reply.result = Result::OK; return reply; }
                last_error = Result::ECHO_ONLY;
            }
            continue;
        }
        bool complete = false;
        const Result parsed = parser.feed(byte, complete);
        if (parsed != Result::OK) { last_error = parsed; continue; }
        if (!complete) continue;
        const std::size_t parameters = parser.count - 6;
        const std::uint8_t packet_id = parser.bytes[2];
        const std::uint8_t error = parser.bytes[4];
        if (packet_id != id) { parser.reset(); continue; }
        if (parameters != expected_parameters && !(error && expected_parameters && parameters == 0)) {
            last_error = Result::LENGTH_ERROR;
            parser.reset();
            continue;
        }
        reply.result = Result::OK;
        reply.servo_error = error;
        reply.status_received = true;
        reply.parameter_count = parameters;
        if (output && !error) std::memcpy(output, parser.bytes + 5, parameters);
        return reply;
    }
    reply.result = last_error;
    return reply;
}
void STServo::genCheckSum(std::uint8_t* frame, std::size_t size) {
    if (!frame || size < 6) return;
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i + 1 < size; ++i) sum += frame[i];
    frame[size - 1] = static_cast<std::uint8_t>(~sum);
}
std::uint8_t STServo::receiveCheckSum(const std::uint8_t* data, std::size_t size) {
    if (!data || size < 4) return static_cast<std::uint8_t>(Result::LENGTH_ERROR);
    std::uint8_t sum = 0;
    for (std::size_t i = 0; i < size; ++i) sum += data[i];
    return static_cast<std::uint8_t>(sum == 0xff ? Result::OK : Result::CHECKSUM_ERROR);
}
const char* STServo::resultName(Result result) {
    switch (result) {
    case Result::OK: return "OK";
    case Result::CHECKSUM_ERROR: return "CHECKSUM_ERROR";
    case Result::HEADER_ERROR: return "HEADER_ERROR";
    case Result::ID_ERROR: return "ID_ERROR";
    case Result::LENGTH_ERROR: return "LENGTH_ERROR";
    case Result::TIMEOUT: return "TIMEOUT";
    case Result::BUFFER_ERROR: return "BUFFER_ERROR";
    case Result::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
    case Result::TRANSPORT_ERROR: return "TRANSPORT_ERROR";
    case Result::ECHO_ONLY: return "ECHO_ONLY";
    case Result::ECHO_ERROR: return "ECHO_ERROR";
    }
    return "UNKNOWN";
}
