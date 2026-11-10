/** @file main.cpp @brief Raspberry Pi 5/Linux multi-ID PING/READ example; no motion commands. */
#include "STServo/stservo.hpp"
#include "stservo_posix.hpp"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static bool number(const char* text, unsigned long max, unsigned long& value) {
    if (!text || !*text || *text == '-') return false;
    char* end = nullptr;
    errno = 0;
    value = std::strtoul(text, &end, 10);
    return !errno && end && *end == '\0' && value <= max;
}
static int usage(const char* name) {
    std::fprintf(stderr, "Usage: %s --device PATH [--baud N] [--timeout MS] [--echo] ID... | --scan\n", name);
    return 1;
}
int main(int argc, char** argv) {
    const char* device = nullptr;
    std::uint32_t baud = 1000000, timeout = 100;
    bool scan = false, echo = false;
    std::vector<std::uint8_t> ids;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--device") == 0) {
            if (++i == argc) return usage(argv[0]);
            device = argv[i];
        } else if (std::strcmp(argv[i], "--baud") == 0 || std::strcmp(argv[i], "--timeout") == 0) {
            const bool is_baud = std::strcmp(argv[i], "--baud") == 0;
            unsigned long value = 0;
            if (++i == argc || !number(argv[i], is_baud ? 1000000UL : 0x7fffffffUL, value) || !value)
                return usage(argv[0]);
            if (is_baud) baud = static_cast<std::uint32_t>(value);
            else timeout = static_cast<std::uint32_t>(value);
        } else if (std::strcmp(argv[i], "--scan") == 0) scan = true;
        else if (std::strcmp(argv[i], "--echo") == 0) echo = true;
        else {
            unsigned long value = 0;
            if (!number(argv[i], STServo::MaxId, value)) return usage(argv[0]);
            ids.push_back(static_cast<std::uint8_t>(value));
        }
    }
    if (!device || (scan && !ids.empty()) || (!scan && ids.empty())) return usage(argv[0]);
    if (scan) for (unsigned id = 0; id <= STServo::MaxId; ++id) ids.push_back(static_cast<std::uint8_t>(id));
    stservo::PosixTransport transport;
    if (!transport.open(device, baud)) {
        std::fprintf(stderr, "open/configure %s: %s\n", device, std::strerror(transport.errorNumber()));
        return 1;
    }
    STServo bus(transport, timeout, echo ? STServo::EchoMode::LocalEcho : STServo::EchoMode::None);
    std::size_t responding = 0;
    bool failed = false;
    for (const auto id : ids) {
        const auto reply = bus.ping(id);
        if (reply.ok() && reply.status_received) ++responding;
        if (!scan || reply.status_received)
            std::printf("id=%u result=%s status=%u err=0x%02X\n", static_cast<unsigned>(id),
                        STServo::resultName(reply.result), static_cast<unsigned>(reply.status_received),
                        static_cast<unsigned>(reply.servo_error));
        if (!reply.ok() || !reply.status_received || reply.servo_error) { failed = true; continue; }
        std::uint8_t reported = 0;
        const auto read = bus.readRegister(id, 5, &reported, 1);
        std::printf("id=%u read_id=%s status=%u err=0x%02X value=%u\n", static_cast<unsigned>(id),
                    STServo::resultName(read.result), static_cast<unsigned>(read.status_received),
                    static_cast<unsigned>(read.servo_error), static_cast<unsigned>(reported));
        if (!read.ok() || !read.status_received || read.servo_error || reported != id) failed = true;
    }
    std::printf("responding=%zu queried=%zu\n", responding, ids.size());
    return scan ? (responding ? 0 : 2) : (failed ? 2 : 0);
}
