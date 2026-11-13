/** @file minimal_control.cpp @brief Minimal STS3215 configuration, feedback and goal example. */
#include "STServo/stservo.hpp"
#include "stservo_posix.hpp"
#include <cstdio>
#include <cstring>

int main(int argc, char** argv) {
    if (argc != 3 || (std::strcmp(argv[2], "position") && std::strcmp(argv[2], "speed"))) {
        std::fprintf(stderr, "Usage: %s DEVICE position|speed\n", argv[0]);
        return 1;
    }
    constexpr std::uint8_t id = 1;
    constexpr int goal_position = 2048; // Single-turn position: 0..4095.
    constexpr int goal_speed = 500;    // Register units, not RPM. Negative in speed mode reverses direction.
    const bool speed_mode = std::strcmp(argv[2], "speed") == 0;
    stservo::PosixTransport port;
    if (!port.open(argv[1], 1000000)) return 1;
    STServo bus(port, 100);

    auto read = [&](std::uint8_t address, std::uint8_t* data, std::size_t size) {
        const auto r = bus.readRegister(id, address, data, size);
        return r.ok() && r.status_received && r.servo_error == 0;
    };
    std::uint8_t level, mode, phase;
    if (!read(0x08, &level, 1) || level > 1 ||
        !read(0x21, &mode, 1) || !read(0x12, &phase, 1)) return 1;
    std::printf("before: mode=%u phase=0x%02X reply_level=%u\n", mode, phase, level);
    const auto ack = level ? STServo::WriteResponse::Required : STServo::WriteResponse::None;
    auto accepted = [&](const STServo::Reply& r) {
        return r.ok() && (ack == STServo::WriteResponse::None ||
                         (r.status_received && r.servo_error == 0));
    };
    auto set8 = [&](std::uint8_t address, std::uint8_t value) {
        std::uint8_t actual = 0;
        return accepted(bus.writeRegister(id, address, &value, 1, ack)) &&
               read(address, &actual, 1) && actual == value; // Verify the write.
    };
    if (!set8(0x28, 0)) return 1; // Disable torque before changing modes/targets.
    const std::uint8_t wanted_mode = speed_mode ? 1 : 0;
    const std::uint8_t wanted_phase = speed_mode ? (phase & ~0x08U) : phase; // Speed 0 means stop.
    if (mode != wanted_mode || phase != wanted_phase) {
        const bool unlocked = set8(0x37, 0); // EEPROM unlock.
        bool configured = unlocked && set8(0x21, wanted_mode);
        if (configured && phase != wanted_phase) configured = set8(0x12, wanted_phase);
        const bool locked = set8(0x37, 1); // Attempt re-lock even if configuration failed.
        if (!configured || !locked) return 1;
    }
    if (goal_speed < -32767 || goal_speed > 32767 ||
        (!speed_mode && (goal_speed <= 0 || goal_position < 0 || goal_position > 4095))) return 1;
    // In position mode this is the travel speed; in speed mode it is the signed goal speed.
    if (!accepted(bus.setVelocity(id, goal_speed, ack))) return 1;
    if (!speed_mode) {
        const std::uint8_t goal[] = {static_cast<std::uint8_t>(goal_position),
                                    static_cast<std::uint8_t>(goal_position >> 8)};
        if (!accepted(bus.writeRegister(id, 0x2A, goal, 2, ack))) return 1;
    }
    if (!set8(0x28, 1)) return 1; // Enable torque only after loading the target.

    std::uint8_t feedback[4]; // Present position (0x38..39), present speed (0x3A..3B).
    if (!read(0x38, feedback, 4)) return 1;
    auto signed15 = [](const std::uint8_t* p) {
        const unsigned raw = p[0] | (static_cast<unsigned>(p[1]) << 8);
        const int magnitude = static_cast<int>(raw & 0x7FFFU);
        return (raw & 0x8000U) ? -magnitude : magnitude; // Sign-magnitude, not two's complement.
    };
    std::printf("position=%d speed=%d\n", signed15(feedback), signed15(feedback + 2));
    // In speed mode, setVelocity(id, 0, ack) stops (phase bit 3 was cleared above).
    // This program does not wait for motion completion or stop the servo on exit.
    return 0;
}
