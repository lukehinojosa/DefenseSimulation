// Secondary telemetry monitor.
//
// A standalone, lightweight console process that attaches to the simulation's
// POSIX shared-memory telemetry ring and reports live threat levels, intercept
// counts, and the telemetry frame rate. It performs no simulation work -- it is
// a pure consumer, demonstrating the decoupled command-and-display process from
// the architecture diagram.

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

#include "sim/SharedMemoryChannel.hpp"
#include "sim/Telemetry.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

const char* threatName(std::uint8_t level) {
    switch (static_cast<sim::telemetry::ThreatLevel>(level)) {
        case sim::telemetry::ThreatLevel::Critical: return "CRIT";
        case sim::telemetry::ThreatLevel::High:     return "HIGH";
        case sim::telemetry::ThreatLevel::Medium:   return "MED ";
        case sim::telemetry::ThreatLevel::Low:      return "LOW ";
        default:                                    return "none";
    }
}

std::uint64_t nowNs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

} // namespace

int main(int argc, char** argv) {
    using namespace sim::telemetry;

    const std::string shmName = (argc > 1) ? argv[1] : kDefaultShmName;
    const double durationSec  = (argc > 2) ? std::stod(argv[2]) : 30.0;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::printf("Telemetry monitor\n  shm: %s\n  waiting for producer...\n",
                shmName.c_str());

    // Wait (bounded) for the producer to create the shared-memory object.
    std::unique_ptr<ShmSubscriber> sub;
    const auto waitStart = std::chrono::steady_clock::now();
    while (!g_stop) {
        try {
            sub = std::make_unique<ShmSubscriber>(shmName);
            if (sub->producerReady()) break;
        } catch (const std::exception&) {
            // Shared-memory object not created yet; keep waiting.
            sub.reset();
        }
        if (std::chrono::steady_clock::now() - waitStart >
            std::chrono::seconds(15)) {
            std::printf("  no producer after 15 s; exiting.\n");
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!sub) return 1;

    std::printf("  producer attached. monitoring for %.0f s "
                "(Ctrl-C to stop)\n\n", durationSec);

    // Preallocated receive buffer: the consumer never allocates in its loop.
    std::unique_ptr<TelemetryRecord[]> buffer(
        new TelemetryRecord[kMaxRecordsPerFrame]);

    FrameHeader hdr{};
    std::uint32_t count = 0;

    std::uint64_t lastFrameId = 0;
    std::uint64_t framesSeen = 0;
    double        emaFps = 0.0;
    std::uint64_t lastTsNs = 0;

    const auto start = std::chrono::steady_clock::now();
    auto lastPrint = start;

    while (!g_stop) {
        const bool ok =
            sub->latest(hdr, buffer.get(), kMaxRecordsPerFrame, count);

        if (ok && hdr.frameId != lastFrameId) {
            if (lastTsNs != 0 && hdr.timestampNs > lastTsNs) {
                const double dt = (hdr.timestampNs - lastTsNs) / 1e9;
                const double inst = (dt > 0.0) ? 1.0 / dt : 0.0;
                emaFps = (emaFps == 0.0) ? inst : 0.9 * emaFps + 0.1 * inst;
            }
            lastTsNs = hdr.timestampNs;
            lastFrameId = hdr.frameId;
            ++framesSeen;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - lastPrint >= std::chrono::milliseconds(500)) {
            lastPrint = now;

            // Tally threat levels and active entity types from the snapshot.
            int byLevel[5] = {0, 0, 0, 0, 0};
            int hostiles = 0, friendlies = 0;
            for (std::uint32_t i = 0; i < count; ++i) {
                const std::uint8_t lvl = buffer[i].threatLevel;
                if (lvl < 5) ++byLevel[lvl];
                if (buffer[i].entityType ==
                    static_cast<std::uint8_t>(sim::EntityType::Hostile)) {
                    ++hostiles;
                } else if (buffer[i].entityType ==
                           static_cast<std::uint8_t>(sim::EntityType::Friendly)) {
                    ++friendlies;
                }
            }

            std::printf(
                "frame %6llu | rate %6.1f Hz | tracks %5u "
                "(H:%d F:%d) | intercepts %3u | threats  %s:%d %s:%d %s:%d %s:%d\n",
                static_cast<unsigned long long>(hdr.frameId), emaFps, count,
                hostiles, friendlies, hdr.interceptCount,
                threatName(4), byLevel[4], threatName(3), byLevel[3],
                threatName(2), byLevel[2], threatName(1), byLevel[1]);
            std::fflush(stdout);
        }

        if (std::chrono::duration<double>(now - start).count() >= durationSec) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    (void)nowNs;
    std::printf("\nmonitor done. frames observed: %llu\n",
                static_cast<unsigned long long>(framesSeen));
    return 0;
}
