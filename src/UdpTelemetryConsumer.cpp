#include "sim/UdpTelemetryConsumer.hpp"

namespace sim {
namespace telemetry {

UdpTelemetryConsumer::UdpTelemetryConsumer(std::uint16_t port)
    : receiver_(port) {
    scratch_.resize(kUdpMaxRecords);
}

bool UdpTelemetryConsumer::PollLatestFrame(TelemetrySnapshot& out) {
    bool got = false;
    FrameHeader hdr{};
    std::uint32_t count = 0;

    // Drain all queued datagrams; keep only the newest (highest frameId). A
    // non-blocking (0 ms) receive loop empties the socket buffer each poll.
    while (receiver_.receive(hdr, scratch_.data(), kUdpMaxRecords, count,
                             /*timeoutMs=*/0)) {
        if (!got || hdr.frameId >= out.header.frameId) {
            out.header = hdr;
            out.records.assign(scratch_.begin(), scratch_.begin() + count);
        }
        got = true;
    }
    return got;
}

} // namespace telemetry
} // namespace sim
