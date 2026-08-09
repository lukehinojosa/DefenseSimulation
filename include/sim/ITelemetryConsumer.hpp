#ifndef SIM_ITELEMETRYCONSUMER_HPP
#define SIM_ITELEMETRYCONSUMER_HPP

#include <vector>

#include "sim/Telemetry.hpp"

namespace sim {
namespace telemetry {

/// A decoded, self-contained telemetry frame for the renderer to draw.
struct TelemetrySnapshot {
    FrameHeader                  header{};
    std::vector<TelemetryRecord> records;
};

/**
 * @brief Transport-agnostic source of telemetry frames.
 *
 * The visualizer depends only on this interface; concrete implementations wrap
 * the POSIX shared-memory ring (local) or the UDP socket (remote). This is the
 * seam that lets one c2_visualizer binary consume either transport via a
 * `--source` flag.
 */
class ITelemetryConsumer {
public:
    virtual ~ITelemetryConsumer() = default;

    /**
     * @brief Fetch the most recent frame available from the transport.
     * @param out Filled with the latest frame on success.
     * @return true if a valid frame was produced since... well, if a valid
     *         frame is currently available (shm) or arrived since the last
     *         poll (udp); false if nothing new/valid is ready.
     */
    virtual bool PollLatestFrame(TelemetrySnapshot& out) = 0;

    /// Human-readable transport name for the HUD (e.g. "shm", "udp").
    virtual const char* SourceName() const = 0;
};

} // namespace telemetry
} // namespace sim

#endif // SIM_ITELEMETRYCONSUMER_HPP
