#include <gtest/gtest.h>

#include <unistd.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "sim/SharedMemoryChannel.hpp"
#include "sim/Telemetry.hpp"
#include "sim/UdpTelemetry.hpp"

using namespace sim;
using namespace sim::telemetry;

namespace {

// Unique shm name per test so parallel/leftover runs don't collide.
std::string uniqueName(const char* tag) {
    return std::string("/defsim_test_") + tag + "_" +
           std::to_string(::getpid());
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

TEST(Telemetry, PacketLayoutIsStable) {
    // Wire contract: sizes must not drift across compilers/versions.
    EXPECT_EQ(sizeof(TelemetryRecord), 30u);
    EXPECT_EQ(sizeof(FrameHeader), 32u);
}

TEST(Telemetry, RecordRoundTripsThroughFloats) {
    Entity e;
    e.id       = 42;
    e.position = {12345.5, 67890.25, 9000.0};
    e.velocity = {-250.0, 100.0, -12.5};
    e.type     = EntityType::Friendly;
    auto r = makeRecord(e, ThreatLevel::Medium);

    EXPECT_EQ(r.entityId, 42u);
    EXPECT_FLOAT_EQ(r.posX, 12345.5f);
    EXPECT_FLOAT_EQ(r.velX, -250.0f);
    EXPECT_EQ(r.entityType, static_cast<std::uint8_t>(EntityType::Friendly));
    EXPECT_EQ(r.threatLevel, static_cast<std::uint8_t>(ThreatLevel::Medium));
}

TEST(Telemetry, ThreatClassificationByTimeToImpact) {
    const Vector3 asset{0.0, 0.0, 0.0};

    Entity fast; fast.type = EntityType::Hostile;
    fast.position = {2000.0, 0.0, 0.0};
    fast.velocity = {-1000.0, 0.0, 0.0};      // 2 s to impact
    EXPECT_EQ(classifyThreat(fast, asset), ThreatLevel::Critical);

    Entity slow; slow.type = EntityType::Hostile;
    slow.position = {10000.0, 0.0, 0.0};
    slow.velocity = {-100.0, 0.0, 0.0};       // 100 s to impact
    EXPECT_EQ(classifyThreat(slow, asset), ThreatLevel::Low);

    // Friendlies never register as threats.
    Entity friendly; friendly.type = EntityType::Friendly;
    friendly.position = {2000.0, 0.0, 0.0};
    friendly.velocity = {-1000.0, 0.0, 0.0};
    EXPECT_EQ(classifyThreat(friendly, asset), ThreatLevel::None);

    // Receding hostile is not closing -> no threat.
    Entity receding; receding.type = EntityType::Hostile;
    receding.position = {2000.0, 0.0, 0.0};
    receding.velocity = {500.0, 0.0, 0.0};
    EXPECT_EQ(classifyThreat(receding, asset), ThreatLevel::None);
}

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
                // Consistency invariant: every record in a frame carries the
                // frameId as its entityId and (frameId % 250) as posX.
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

TEST(Udp, SendReceiveRoundTrip) {
    UdpTelemetryReceiver rx(0); // ephemeral port
    UdpTelemetrySender tx("127.0.0.1", rx.boundPort());

    FrameHeader hdr{};
    hdr.magic          = kMagic;
    hdr.version        = kProtocolVersion;
    hdr.frameId        = 555;
    hdr.timestampNs    = 987654321;
    hdr.recordCount    = 3;
    hdr.interceptCount = 4;

    std::vector<TelemetryRecord> recs = {
        makeRec(10, 11.0f, 4), makeRec(20, 22.0f, 2), makeRec(30, 33.0f, 1)};
    ASSERT_GT(tx.send(hdr, recs.data(), 3), 0);

    FrameHeader got{};
    std::vector<TelemetryRecord> out(kUdpMaxRecords);
    std::uint32_t count = 0;
    ASSERT_TRUE(rx.receive(got, out.data(), kUdpMaxRecords, count, 2000));

    EXPECT_EQ(got.frameId, 555u);
    EXPECT_EQ(got.interceptCount, 4u);
    ASSERT_EQ(count, 3u);
    EXPECT_EQ(out[0].entityId, 10u);
    EXPECT_FLOAT_EQ(out[2].posX, 33.0f);
}

TEST(Udp, OversizeFrameIsClampedToDatagramCapacity) {
    UdpTelemetryReceiver rx(0);
    UdpTelemetrySender tx("127.0.0.1", rx.boundPort());

    FrameHeader hdr{};
    hdr.magic       = kMagic;
    hdr.version     = kProtocolVersion;
    hdr.frameId     = 1;
    hdr.recordCount = 1000; // more than a datagram can carry

    std::vector<TelemetryRecord> recs(1000);
    for (std::uint32_t i = 0; i < recs.size(); ++i) recs[i].entityId = i;
    ASSERT_GT(tx.send(hdr, recs.data(), 1000), 0);

    FrameHeader got{};
    std::vector<TelemetryRecord> out(kUdpMaxRecords);
    std::uint32_t count = 0;
    ASSERT_TRUE(rx.receive(got, out.data(), kUdpMaxRecords, count, 2000));
    EXPECT_EQ(count, kUdpMaxRecords); // clamped, not corrupted
    EXPECT_EQ(out[0].entityId, 0u);
}
