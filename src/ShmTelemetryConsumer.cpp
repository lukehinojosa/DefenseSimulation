#include "sim/ShmTelemetryConsumer.hpp"

namespace sim {
namespace telemetry {

ShmTelemetryConsumer::ShmTelemetryConsumer(const std::string& name)
    : subscriber_(name) {
    scratch_.resize(kMaxRecordsPerFrame);
}

bool ShmTelemetryConsumer::PollLatestFrame(TelemetrySnapshot& out) {
    FrameHeader hdr{};
    std::uint32_t count = 0;
    if (!subscriber_.latest(hdr, scratch_.data(), kMaxRecordsPerFrame, count)) {
        return false;
    }
    out.header = hdr;
    out.records.assign(scratch_.begin(), scratch_.begin() + count);
    return true;
}

} // namespace telemetry
} // namespace sim
