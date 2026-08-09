#ifndef SIM_UDPTELEMETRYCONSUMER_HPP
#define SIM_UDPTELEMETRYCONSUMER_HPP

#include <cstdint>

#include "sim/ITelemetryConsumer.hpp"
#include "sim/UdpTelemetry.hpp"

namespace sim {
namespace telemetry {

/**
 * @brief Cross-platform UDP telemetry consumer (remote transport).
 *
 * Non-blocking: each poll drains the socket and returns the most recent
 * datagram, so a slow renderer never falls behind on stale frames. Builds on
 * both Linux and Windows (Winsock), enabling the remote C2 display.
 */
class UdpTelemetryConsumer : public ITelemetryConsumer {
public:
    explicit UdpTelemetryConsumer(std::uint16_t port);

    bool PollLatestFrame(TelemetrySnapshot& out) override;
    const char* SourceName() const override { return "udp"; }

    std::uint16_t boundPort() const { return receiver_.boundPort(); }

private:
    UdpTelemetryReceiver         receiver_;
    std::vector<TelemetryRecord> scratch_;
};

} // namespace telemetry
} // namespace sim

#endif // SIM_UDPTELEMETRYCONSUMER_HPP
