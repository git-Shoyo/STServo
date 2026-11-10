/** @file multi_servo.cpp @brief Platform-neutral usage; call only when motion is intended. */
#include "STServo/stservo.hpp"
#include <initializer_list>

// Supply your application's initialized transport. A single instance owns the bus.
void queryTwoServos(stservo::Transport& transport) {
    STServo bus(transport, 100);
    for (std::uint8_t id : {std::uint8_t(1), std::uint8_t(2)}) {
        const STServo::Reply reply = bus.ping(id);
        if (reply.ok() && reply.status_received && !reply.servo_error) {
            std::uint8_t reported_id = 0;
            const auto read = bus.readRegister(id, 5, &reported_id, 1);
            (void)read; // Application decides how to report/handle each result.
        }
    }
}

STServo::Result sendTwoVelocities(STServo& bus) {
    // Configure velocity mode per ID first. Check acknowledgements if enabled
    // by your servo's response policy. SYNC WRITE itself has no ACK.
    const std::uint8_t ids[] = {1, 2};
    const std::int32_t velocities[] = {500, -500};
    return bus.syncSetVelocity(ids, velocities, 2);
}
