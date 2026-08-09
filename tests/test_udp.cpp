// Cross-platform telemetry tests: wire protocol + UDP transport + UDP consumer.
// These build and run on every platform (POSIX sockets or Winsock), so the
// network path is covered by ctest on Windows/MSVC as well as Linux.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "sim/Telemetry.hpp"
#include "sim/TelemetryCodec.hpp"
#include "sim/UdpTelemetry.hpp"
#include "sim/UdpTelemetryConsumer.hpp"

using namespace sim;
using namespace sim::telemetry;

namespace {

TelemetryRecord makeRec(std::uint32_t id, float x, std::uint8_t level) {
    Entity e;
    e.id       = id;
    e.position = {x, x + 1.0, x + 2.0};
    e.velocity = {1.0, 2.0, 3.0};
    e.type     = EntityType::Hostile;
    return makeRecord(e, static_cast<ThreatLevel>(level));
}

} // namespace

// --- Wire protocol -----------------------------------------------------------
TEST(Telemetry, PacketLayoutIsStable) {
    EXPECT_EQ(sizeof(TelemetryRecord), 35u); // protocol v2
    EXPECT_EQ(sizeof(FrameHeader), 32u);
    EXPECT_EQ(kProtocolVersion, 2u);
}

TEST(Telemetry, RecordRoundTripsThroughFloats) {
    Entity e;
    e.id       = 42;
    e.position = {12345.5, 67890.25, 9000.0};
    e.velocity = {-250.0, 100.0, -12.5};
    e.type     = EntityType::Friendly;
    auto r = makeRecord(e, ThreatLevel::Medium, /*targetId=*/99, FLAG_DESTROYED);

    EXPECT_EQ(r.entityId, 42u);
    EXPECT_FLOAT_EQ(r.posX, 12345.5f);
    EXPECT_FLOAT_EQ(r.velX, -250.0f);
    EXPECT_EQ(r.targetId, 99u);
    EXPECT_EQ(r.entityType, static_cast<std::uint8_t>(EntityType::Friendly));
    EXPECT_EQ(r.threatLevel, static_cast<std::uint8_t>(ThreatLevel::Medium));
    EXPECT_EQ(r.flags, static_cast<std::uint8_t>(FLAG_DESTROYED));
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

    Entity friendly; friendly.type = EntityType::Friendly;
    friendly.position = {2000.0, 0.0, 0.0};
    friendly.velocity = {-1000.0, 0.0, 0.0};
    EXPECT_EQ(classifyThreat(friendly, asset), ThreatLevel::None);

    Entity receding; receding.type = EntityType::Hostile;
    receding.position = {2000.0, 0.0, 0.0};
    receding.velocity = {500.0, 0.0, 0.0};
    EXPECT_EQ(classifyThreat(receding, asset), ThreatLevel::None);
}

TEST(Telemetry, UdpRecordPriorityOrdering) {
    Entity h; h.type = EntityType::Hostile;
    Entity f; f.type = EntityType::Friendly;
    Entity n; n.type = EntityType::Neutral;

    auto detonation = makeRecord(h, ThreatLevel::None, kNoTargetId, FLAG_DESTROYED);
    auto engaged    = makeRecord(f, ThreatLevel::None, 7);
    auto critical   = makeRecord(h, ThreatLevel::Critical);
    auto lowThreat  = makeRecord(h, ThreatLevel::Low);
    auto idle       = makeRecord(f, ThreatLevel::None);
    auto neutral    = makeRecord(n, ThreatLevel::None);

    EXPECT_GT(recordPriority(detonation), recordPriority(engaged));
    EXPECT_GT(recordPriority(engaged), recordPriority(critical));
    EXPECT_GT(recordPriority(critical), recordPriority(lowThreat));
    EXPECT_GT(recordPriority(lowThreat), recordPriority(idle));
    EXPECT_GT(recordPriority(idle), recordPriority(neutral));
}

// --- Binary codec ------------------------------------------------------------
TEST(Codec, VlqAndZigZagRoundTrip) {
    using namespace sim::telemetry::codec;
    for (std::int32_t n : {0, 1, -1, 127, -128, 250000, -250000, 2000000000}) {
        EXPECT_EQ(zzDec(zzEnc(n)), n);
    }
    std::uint8_t buf[8];
    for (std::uint32_t v : {0u, 1u, 127u, 128u, 16384u, 4000000000u}) {
        const int n = vlqWrite(buf, v);
        const std::uint8_t* p = buf;
        EXPECT_EQ(vlqRead(p, buf + n), v);
        EXPECT_EQ(p, buf + n);
    }
}

TEST(Codec, FrameRoundTripAndCompactness) {
    using namespace sim::telemetry::codec;
    FrameHeader hdr{};
    hdr.frameId = 12345; hdr.timestampNs = 7000000000ull; // 7000 ms
    hdr.interceptCount = 9;

    std::vector<TelemetryRecord> recs = {
        makeRec(3, 45000.0f, 4),                                  // hostile, crit
        makeRecord([]{ Entity e; e.id=8; e.type=EntityType::Friendly;
                       e.position={10.0,20.0,30.0}; e.velocity={-250.0,7.0,-3.0};
                       return e; }(), ThreatLevel::None, /*target=*/3),
        makeRecord([]{ Entity e; e.id=99; e.type=EntityType::Hostile;
                       e.position={1.0,2.0,3.0}; return e; }(),
                   ThreatLevel::None, kNoTargetId, FLAG_DESTROYED)};

    std::uint8_t buf[kUdpDatagramBudget];
    std::uint32_t enc = 0;
    const std::size_t bytes = encodeFrame(buf, sizeof(buf), hdr, recs.data(),
                                          3, enc);
    EXPECT_EQ(enc, 3u);
    // Compact: far smaller than the fixed 35 B/record layout.
    EXPECT_LT(bytes, 3u * sizeof(TelemetryRecord));

    FrameHeader out{};
    std::vector<TelemetryRecord> got(16);
    std::uint32_t n = 0;
    ASSERT_TRUE(decodeFrame(buf, bytes, out, got.data(), 16, n));
    ASSERT_EQ(n, 3u);
    EXPECT_EQ(out.frameId, 12345u);
    EXPECT_EQ(out.timestampNs, 7000000000ull); // ms-quantized, exact here
    EXPECT_EQ(out.interceptCount, 9u);

    EXPECT_EQ(got[0].entityId, 3u);
    EXPECT_FLOAT_EQ(got[0].posX, 45000.0f);           // meter-quantized
    EXPECT_EQ(got[0].threatLevel, static_cast<std::uint8_t>(ThreatLevel::Critical));
    EXPECT_EQ(got[1].entityId, 8u);
    EXPECT_EQ(got[1].entityType, static_cast<std::uint8_t>(EntityType::Friendly));
    EXPECT_EQ(got[1].targetId, 3u);                   // LOS target survives
    EXPECT_FLOAT_EQ(got[1].velX, -250.0f);            // signed velocity survives
    EXPECT_EQ(got[2].targetId, kNoTargetId);
    EXPECT_TRUE(got[2].flags & FLAG_DESTROYED);       // detonation flag survives
}

// --- UDP transport -----------------------------------------------------------
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

TEST(Udp, NonBlockingReceiveReturnsImmediately) {
    // Regression: timeoutMs==0 must poll (return at once), not block forever.
    UdpTelemetryReceiver rx(0);
    FrameHeader hdr{};
    std::vector<TelemetryRecord> out(kUdpMaxRecords);
    std::uint32_t count = 0;
    const auto t0 = std::chrono::steady_clock::now();
    const bool got = rx.receive(hdr, out.data(), kUdpMaxRecords, count, 0);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count();
    EXPECT_FALSE(got);
    EXPECT_LT(ms, 500) << "non-blocking receive should return promptly";
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

TEST(UdpConsumer, DrainsToLatestFrame) {
    UdpTelemetryConsumer consumer(0);
    UdpTelemetrySender tx("127.0.0.1", consumer.boundPort());

    // Nothing sent yet: poll returns false without blocking.
    TelemetrySnapshot snap;
    EXPECT_FALSE(consumer.PollLatestFrame(snap));

    std::vector<TelemetryRecord> recs = {makeRec(1, 1.0f, 2)};
    for (std::uint64_t f = 1; f <= 5; ++f) {
        FrameHeader hdr{};
        hdr.magic = kMagic; hdr.version = kProtocolVersion;
        hdr.frameId = f; hdr.interceptCount = static_cast<std::uint32_t>(f);
        ASSERT_GT(tx.send(hdr, recs.data(), 1), 0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(consumer.PollLatestFrame(snap));
    EXPECT_EQ(snap.header.frameId, 5u); // newest wins after draining
    EXPECT_EQ(std::string(consumer.SourceName()), "udp");
}
