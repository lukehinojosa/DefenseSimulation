#ifndef SIM_SHMTELEMETRYCONSUMER_HPP
#define SIM_SHMTELEMETRYCONSUMER_HPP

#include <string>

#include "sim/ITelemetryConsumer.hpp"
#include "sim/SharedMemoryChannel.hpp"

namespace sim {
namespace telemetry {

/**
 * @brief POSIX shared-memory telemetry consumer (local transport, Linux only).
 *
 * Wraps a ShmSubscriber and snapshots the latest seqlock-published frame into a
 * TelemetrySnapshot. Reads never block the producer or tear a frame.
 */
class ShmTelemetryConsumer : public ITelemetryConsumer {
public:
    explicit ShmTelemetryConsumer(const std::string& name = kDefaultShmName);

    bool PollLatestFrame(TelemetrySnapshot& out) override;
    const char* SourceName() const override { return "shm"; }

private:
    ShmSubscriber                subscriber_;
    std::vector<TelemetryRecord> scratch_;
};

} // namespace telemetry
} // namespace sim

#endif // SIM_SHMTELEMETRYCONSUMER_HPP
