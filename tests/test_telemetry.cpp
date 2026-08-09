// POSIX shared-memory telemetry tests (Linux only). The cross-platform wire
// protocol and UDP tests live in test_udp.cpp so they also run under MSVC.

#include <gtest/gtest.h>

#if defined(_WIN32)
#include <process.h>
#define SIM_GETPID _getpid
#else
#include <unistd.h>
#define SIM_GETPID getpid
#endif

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "sim/SharedMemoryChannel.hpp"
#include "sim/ShmTelemetryConsumer.hpp"
#include "sim/Telemetry.hpp"

using namespace sim;
using namespace sim::telemetry;

namespace {

// Unique shm name per test so parallel/leftover runs don't collide. No leading
// slash / special characters (valid for Boost.Interprocess on all platforms).
std::string uniqueName(const char* tag) {
    return std::string("defsim_test_") + tag + "_" +
           std::to_string(SIM_GETPID());
}

TelemetryRecord makeRec(std::uint32_t id, float x, std::uint8_t level) {
    Entity e;
    e.id       = id;
    e.position = {x, x + 1.0, x + 2.0};
    e.velocity = {1.0, 2.0, 3.0};
    e.type     = EntityType::Hostile;
    return makeRecord(e, static_cast<ThreatLevel>(level));
}

} // namespace

TEST(SharedMemory, PublishThenSubscribeRoundTrip) {
    const std::string name = uniqueName("rt");
    ShmPublisher pub(name, /*unlinkOnClose=*/true);
    ShmSubscriber sub(name);

    std::vector<TelemetryRecord> recs = {
        makeRec(1, 100.0f, 4), makeRec(2, 200.0f, 3), makeRec(3, 300.0f, 1)};
    pub.publish(/*frameId=*/7, /*ts=*/123456789, /*intercepts=*/2,
                recs.data(), static_cast<std::uint32_t>(recs.size()));

    FrameHeader hdr{};
    std::vector<TelemetryRecord> out(kMaxRecordsPerFrame);
    std::uint32_t count = 0;
    ASSERT_TRUE(sub.latest(hdr, out.data(), kMaxRecordsPerFrame, count));

    EXPECT_EQ(hdr.frameId, 7u);
    EXPECT_EQ(hdr.timestampNs, 123456789u);
    EXPECT_EQ(hdr.interceptCount, 2u);
    ASSERT_EQ(count, 3u);
    EXPECT_EQ(out[0].entityId, 1u);
    EXPECT_FLOAT_EQ(out[1].posX, 200.0f);
    EXPECT_EQ(out[2].threatLevel, 1u);
}

TEST(SharedMemory, SubscriberSeesLatestFrame) {
    const std::string name = uniqueName("latest");
    ShmPublisher pub(name);
    ShmSubscriber sub(name);

    std::vector<TelemetryRecord> recs = {makeRec(9, 1.0f, 2)};
    for (std::uint64_t f = 0; f < 100; ++f) {
        pub.publish(f, f * 1000, static_cast<std::uint32_t>(f), recs.data(), 1);
    }

    FrameHeader hdr{};
    std::vector<TelemetryRecord> out(kMaxRecordsPerFrame);
    std::uint32_t count = 0;
    ASSERT_TRUE(sub.latest(hdr, out.data(), kMaxRecordsPerFrame, count));
    EXPECT_EQ(hdr.frameId, 99u); // most recent publish wins
    EXPECT_EQ(hdr.interceptCount, 99u);
}

// Stress the seqlock: a writer hammers the ring while a reader continuously
// snapshots. Every snapshot must be internally consistent (no torn frames).
TEST(SharedMemory, SeqlockYieldsConsistentSnapshots) {
    const std::string name = uniqueName("seq");
    ShmPublisher pub(name);
    ShmSubscriber sub(name);

    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> reads{0};
    std::atomic<std::uint64_t> torn{0};

    std::thread reader([&] {
        std::vector<TelemetryRecord> out(1024);
        FrameHeader hdr{};
        std::uint32_t count = 0;
        while (!stop.load(std::memory_order_relaxed)) {
            if (sub.latest(hdr, out.data(), 1024, count)) {
                const auto expectX = static_cast<float>(hdr.frameId % 250);
                bool consistent = true;
                for (std::uint32_t i = 0; i < count; ++i) {
                    if (out[i].entityId != static_cast<std::uint32_t>(hdr.frameId) ||
                        out[i].posX != expectX) {
                        consistent = false;
                        break;
                    }
                }
                if (!consistent) torn.fetch_add(1);
                reads.fetch_add(1);
            }
        }
    });

    std::vector<TelemetryRecord> recs(512);
    for (std::uint64_t f = 1; f <= 20000; ++f) {
        const auto id = static_cast<std::uint32_t>(f);
        const auto x  = static_cast<float>(f % 250);
        for (auto& r : recs) { r.entityId = id; r.posX = x; }
        pub.publish(f, f, 0, recs.data(), static_cast<std::uint32_t>(recs.size()));
    }
    stop.store(true);
    reader.join();

    EXPECT_EQ(torn.load(), 0u) << "seqlock allowed a torn frame";
    EXPECT_GT(reads.load(), 0u) << "reader never observed a frame";
}

TEST(ShmConsumer, PollLatestFrameRoundTrip) {
    const std::string name = uniqueName("cons");
    ShmPublisher pub(name);
    ShmTelemetryConsumer consumer(name);

    std::vector<TelemetryRecord> recs = {makeRec(4, 40.0f, 3),
                                         makeRec(5, 50.0f, 4)};
    pub.publish(11, 222, 6, recs.data(), 2);

    TelemetrySnapshot snap;
    ASSERT_TRUE(consumer.PollLatestFrame(snap));
    EXPECT_EQ(snap.header.frameId, 11u);
    EXPECT_EQ(snap.header.interceptCount, 6u);
    ASSERT_EQ(snap.records.size(), 2u);
    EXPECT_EQ(snap.records[1].entityId, 5u);
    EXPECT_EQ(std::string(consumer.SourceName()), "shm");
}
