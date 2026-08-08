#ifndef SIM_UDPTELEMETRY_HPP
#define SIM_UDPTELEMETRY_HPP

#include <cstdint>
#include <string>

#include "sim/Telemetry.hpp"

namespace sim {
namespace telemetry {

/// Records that fit one ~1.4 KB datagram alongside the header (stays < MTU).
constexpr std::uint32_t kUdpMaxRecords = 45;

/**
 * @brief UDP sender for telemetry frames (remote transport fallback).
 *
 * Each frame is sent as a single datagram: a FrameHeader followed by up to
 * kUdpMaxRecords records (the highest-threat records when the frame is larger,
 * chosen by the caller). Serialization uses a fixed stack buffer, so no heap
 * allocation occurs on the send path.
 */
class UdpTelemetrySender {
public:
    /// @throws std::system_error on socket/resolve failure.
    UdpTelemetrySender(const std::string& host, std::uint16_t port);
    ~UdpTelemetrySender();

    UdpTelemetrySender(const UdpTelemetrySender&) = delete;
    UdpTelemetrySender& operator=(const UdpTelemetrySender&) = delete;

    /// Send one datagram; @p count is clamped to kUdpMaxRecords. Returns bytes
    /// sent, or -1 on error.
    long send(const FrameHeader& header,
              const TelemetryRecord* records,
              std::uint32_t count);

private:
    int fd_{-1};
};

/**
 * @brief UDP receiver that reconstructs telemetry frames from datagrams.
 */
class UdpTelemetryReceiver {
public:
    /// Bind to @p port on all interfaces. @throws std::system_error on failure.
    explicit UdpTelemetryReceiver(std::uint16_t port);
    ~UdpTelemetryReceiver();

    UdpTelemetryReceiver(const UdpTelemetryReceiver&) = delete;
    UdpTelemetryReceiver& operator=(const UdpTelemetryReceiver&) = delete;

    /// Block up to @p timeoutMs for a datagram (<0 = block indefinitely).
    /// @return true if a valid frame was received.
    bool receive(FrameHeader& headerOut,
                 TelemetryRecord* recordsOut,
                 std::uint32_t maxRecords,
                 std::uint32_t& outCount,
                 int timeoutMs = -1);

    /// Local bound port (useful when constructed with port 0 for tests).
    std::uint16_t boundPort() const { return boundPort_; }

private:
    int           fd_{-1};
    std::uint16_t boundPort_{0};
};

} // namespace telemetry
} // namespace sim

#endif // SIM_UDPTELEMETRY_HPP
