/**
 * @file stservo.hpp
 * @brief Platform-independent FT-SCS bus client for STS servos (C++11).
 */
#pragma once
#include <cstddef>
#include <cstdint>

namespace stservo {
/** @brief Porting boundary. One transport belongs to one exclusively owned bus.
 * @details Implement all five operations for a new platform. Implementations must
 * not retain write()'s buffer. readByte() must return promptly. Configure baud,
 * 8N1 and any electrical half-duplex direction switching outside the core.
 */
class Transport {
public:
    /** @brief Destroy a transport through its abstract interface. */
    virtual ~Transport() = default;
    /** @brief Accept the entire frame before the timeout, or return false.
     * @param data Frame storage, valid only until this call returns.
     * @param size Number of bytes.
     * @param timeout_ms Remaining transaction budget, in milliseconds.
     * @return True only if every byte was accepted; this is not a servo ACK.
     * @note Buffered OS writes may still be on the wire after returning. A
     * manually switched half-duplex port must finish TX and enable RX itself.
     */
    virtual bool write(const std::uint8_t* data, std::size_t size,
                       std::uint32_t timeout_ms) = 0;
    /** @return 0..255: byte; -1: no byte yet; <= -2: transport failure. */
    virtual int readByte() = 0;
    /** @brief Discard bytes already received, without waiting for new input. */
    virtual bool discardInput() = 0;
    /** @brief Monotonic milliseconds modulo 2^32, progressing during waits. */
    virtual std::uint32_t nowMs() const = 0;
    /** @brief Yield/wait briefly; must return promptly within the time budget. */
    virtual void idle() = 0;
};
} // namespace stservo

/** @brief One serial bus serving multiple IDs, with no HAL dependency or heap allocation.
 * @details Calls are synchronous. Use one shared instance per physical bus and
 * externally serialize calls (including calls made by different threads).
 * The transport must outlive this object. No ID registration is required.
 */
class STServo {
public:
    static constexpr std::uint8_t MaxId = 253; ///< Highest unicast ID.
    static constexpr std::uint8_t BroadcastId = 254; ///< Broadcast destination, without individual ACKs.
    static constexpr std::size_t MaxPacketSize = 64; ///< Maximum header-including wire frame size.
    static constexpr std::size_t MaxReadSize = MaxPacketSize - 6; ///< Maximum READ payload size.
    static constexpr std::size_t MaxWriteSize = MaxPacketSize - 7; ///< Maximum WRITE payload size.
    static constexpr std::size_t MaxSyncVelocityServos = (MaxPacketSize - 8) / 3; ///< Maximum two-byte velocity entries in one SYNC WRITE.

    /** @brief Local framing/transport outcome, separate from the servo's ERROR byte. */
    enum class Result : std::uint8_t {
        OK = 0x00, CHECKSUM_ERROR = 0x10, HEADER_ERROR = 0x20,
        ID_ERROR = 0x22, LENGTH_ERROR = 0x23, TIMEOUT = 0x24,
        BUFFER_ERROR = 0x25, INVALID_ARGUMENT = 0x26,
        TRANSPORT_ERROR = 0x27, ECHO_ONLY = 0x28, ECHO_ERROR = 0x29
    };
    /** @brief Select according to the actual electrical interface; no auto-detection.
     * @warning A PING echo is byte-identical to a valid ERROR=1 reply. LocalEcho
     * requires exactly one complete local echo before the real status packet.
     */
    enum class EchoMode { None, LocalEcho };
    /** @brief Whether WRITE must receive a status packet (must match servo configuration). */
    enum class WriteResponse { None, Required };

    /** @brief A result bound to its requested ID, never to a previous servo.
     * @note OK means local communication succeeded. A healthy acknowledged reply
     * additionally requires status_received && servo_error == 0.
     */
    struct Reply {
        Result result; ///< Local protocol/transport result.
        std::uint8_t id; ///< Requested destination ID.
        std::uint8_t servo_error; ///< Meaningful only when status_received is true.
        bool status_received; ///< True only after validating a status frame.
        std::size_t parameter_count; ///< Number of returned parameter bytes.
        /** @brief Construct an unacknowledged local result for a destination. */
        Reply(Result result_, std::uint8_t id_);
        bool ok() const; ///< Local communication outcome only.
    };

    /** @param transport Exclusively owned serial/time adapter.
     * @param timeout_ms Entire operation's budget (1..0x7fffffff milliseconds).
     * @param echo Whether a local echo must be consumed before a status reply.
     */
    explicit STServo(stservo::Transport& transport, std::uint32_t timeout_ms = 10,
                     EchoMode echo = EchoMode::None);
    STServo(const STServo&) = delete;
    STServo& operator=(const STServo&) = delete;

    /** @brief Query one ID (0..253). Broadcast PING is intentionally rejected.
     * @param id Destination unicast ID.
     * @return Communication result and, when received, this ID's servo status.
     */
    Reply ping(std::uint8_t id);
    /** @brief Read contiguous registers (1..MaxReadSize bytes).
     * @param id Destination unicast ID.
     * @param address First register address.
     * @param size Requested data length.
     * @param output Modified only after a valid, correctly sized ERROR=0 reply.
     * @note A nonzero servo ERROR is returned as an acknowledged Reply, without
     * modifying output; a servo may omit parameters in that error reply.
     */
    Reply readRegister(std::uint8_t id, std::uint8_t address,
                       std::uint8_t* output, std::size_t size);
    /** @brief Write contiguous registers. Broadcast is allowed only without ACK.
     * @param id Destination unicast ID, or BroadcastId without an ACK.
     * @param address First register address.
     * @param data Contiguous register values.
     * @param size Number of values in bytes.
     * @param response Whether a status reply is required.
     * @note With response=None, OK means transmission accepted, not servo success.
     */
    Reply writeRegister(std::uint8_t id, std::uint8_t address,
                        const std::uint8_t* data, std::size_t size,
                        WriteResponse response = WriteResponse::None);
    /** @brief Write the same register range for several IDs in one 0x83 broadcast.
     * @param address First register address for every servo.
     * @param bytes_per_servo Register data length for each ID.
     * @param ids Unique IDs in 0..253.
     * @param data Servo-major array: count * bytes_per_servo bytes, in ids order.
     * @param count Number of servos; must fit MaxPacketSize.
     * @return Local send result; SYNC WRITE has no individual acknowledgements.
     */
    Result syncWriteRegister(std::uint8_t address, std::uint8_t bytes_per_servo,
                             const std::uint8_t* ids, const std::uint8_t* data,
                             std::size_t count);
    /** @brief STS operation mode register 0x21 receives 1 (wheel/velocity mode).
     * @param id Destination ID (or broadcast when no ACK is requested).
     * @param response Whether a status reply is required.
     */
    Reply setVelocityMode(std::uint8_t id, WriteResponse response = WriteResponse::None);
    /** @brief STS goal speed registers 0x2e/0x2f; magnitude clamps to 32767.
     * @param id Destination ID (or broadcast when no ACK is requested).
     * @param velocity Signed goal speed in the servo model's register units.
     * @param response Whether a status reply is required.
     * @details Negative values set direction bit 15; bytes are little endian.
     * Confirm the register map for your exact servo model before use.
     */
    Reply setVelocity(std::uint8_t id, std::int32_t velocity,
                      WriteResponse response = WriteResponse::None);
    /** @brief Backwards-name alias for setVelocity(), now with an explicit ID.
     * @param id Destination ID.
     * @param velocity Signed goal speed; required to avoid ambiguous legacy one-argument calls.
     */
    Reply sts_exchange(std::uint8_t id, std::int32_t velocity);
    /** @brief Send individual signed velocities in one SYNC WRITE frame.
     * @param ids Unique destination IDs.
     * @param velocities Signed speeds in matching ID order.
     * @param count Number of IDs/speeds, at most MaxSyncVelocityServos.
     * @note Mode configuration must be performed separately for every servo.
     */
    Result syncSetVelocity(const std::uint8_t* ids, const std::int32_t* velocities,
                           std::size_t count);
    /** @brief Change the entire-operation timeout; false leaves it unchanged.
     * @param timeout_ms Budget in milliseconds, from 1 through 0x7fffffff.
     */
    bool setTimeoutMs(std::uint32_t timeout_ms);
    /** @brief Return the configured entire-operation timeout. */
    std::uint32_t timeoutMs() const;
    /** @brief Select the known local-echo behavior of the physical interface.
     * @param mode Known hardware behavior, never guessed from PING bytes.
     */
    void setEchoMode(EchoMode mode);
    /** @brief Return the configured local-echo behavior. */
    EchoMode echoMode() const;
    /** @brief Fill checksum of a complete header-including frame (size >= 6).
     * @param frame Writable frame including its checksum byte.
     * @param size Entire frame size including the two-byte header.
     */
    static void genCheckSum(std::uint8_t* frame, std::size_t size);
    /** @brief Check an ID-through-checksum packet (without the two FF headers).
     * @param packet Bytes starting with ID, ending with checksum.
     * @param size Packet size excluding the header, at least four.
     */
    static std::uint8_t receiveCheckSum(const std::uint8_t* packet, std::size_t size);
    /** @brief Return a static printable name; ownership stays with the library. */
    static const char* resultName(Result result);

private:
    enum class Command : std::uint8_t { Ping = 1, Read = 2, Write = 3, SyncWrite = 0x83 };
    Reply exchange(std::uint8_t id, const std::uint8_t* frame, std::size_t size,
                   bool expect_status, std::size_t expected_parameters,
                   std::uint8_t* output);
    static void encodeVelocity(std::int32_t velocity, std::uint8_t* output);
    stservo::Transport& transport_;
    std::uint32_t timeout_ms_;
    EchoMode echo_;
};
