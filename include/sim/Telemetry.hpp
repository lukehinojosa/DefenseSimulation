#ifndef SIM_TELEMETRY_HPP
#define SIM_TELEMETRY_HPP

#include <atomic>
#include <cstdint>

#include "sim/Entity.hpp"
#include "sim/Guidance.hpp"
#include "sim/Vector3.hpp"

namespace sim {
namespace telemetry {

/// Wire identifiers so a reader can validate a mapping/datagram.
constexpr std::uint32_t kMagic           = 0x44534d31u; // 'DSM1'
constexpr std::uint32_t kProtocolVersion = 1u;

/// Ring geometry. Slots >= 3 let a single writer cycle away from the slot a
/// reader is copying, so the seqlock almost never has to retry.
constexpr std::uint32_t kSlotCount        = 4u;
constexpr std::uint32_t kMaxRecordsPerFrame = 20000u;

/// Default POSIX shared-memory object name (leading slash per shm_open(3)).
constexpr const char* kDefaultShmName = "/defsim_telemetry";

/// Coarse threat classification carried in each record.
enum class ThreatLevel : std::uint8_t {
    None     = 0,
    Low      = 1,
    Medium   = 2,
    High     = 3,
    Critical = 4
};

#pragma pack(push, 1)
/**
 * @brief Fixed-size, byte-packed per-entity telemetry record (wire format).
 *
 * Floats (not doubles) keep the datagram compact; meter precision over a
 * 100 km airspace is well within float range for a display feed.
 */
struct TelemetryRecord {
    std::uint32_t entityId;
    float         posX, posY, posZ;
    float         velX, velY, velZ;
    std::uint8_t  entityType;  // sim::EntityType
    std::uint8_t  threatLevel; // sim::telemetry::ThreatLevel
};

/// Per-frame header prefixing a block of records (wire format).
struct FrameHeader {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint64_t frameId;
    std::uint64_t timestampNs;
    std::uint32_t recordCount;
    std::uint32_t interceptCount;
};
#pragma pack(pop)

static_assert(sizeof(TelemetryRecord) == 30,
              "TelemetryRecord must stay 30 bytes on the wire");
static_assert(sizeof(FrameHeader) == 32,
              "FrameHeader must stay 32 bytes on the wire");

/**
 * @brief Classify an entity's threat level from time-to-impact on an asset.
 *
 * Only hostiles carry a non-zero level. Closer time-to-impact => higher level.
 */
inline ThreatLevel classifyThreat(const Entity& e, const Vector3& defendedAsset) {
    if (e.type != EntityType::Hostile || !e.isActive()) {
        return ThreatLevel::None;
    }
    const Vector3 toAsset = defendedAsset - e.position;
    const double tti = guidance::timeToGo(toAsset, Vector3{} - e.velocity);
    if (tti < 10.0)  return ThreatLevel::Critical;
    if (tti < 30.0)  return ThreatLevel::High;
    if (tti < 60.0)  return ThreatLevel::Medium;
    if (tti < 120.0) return ThreatLevel::Low;
    return ThreatLevel::None;
}

/// Build a wire record from an entity (position/velocity narrowed to float).
inline TelemetryRecord makeRecord(const Entity& e, ThreatLevel level) {
    TelemetryRecord r;
    r.entityId    = e.id;
    r.posX        = static_cast<float>(e.position.x);
    r.posY        = static_cast<float>(e.position.y);
    r.posZ        = static_cast<float>(e.position.z);
    r.velX        = static_cast<float>(e.velocity.x);
    r.velY        = static_cast<float>(e.velocity.y);
    r.velZ        = static_cast<float>(e.velocity.z);
    r.entityType  = static_cast<std::uint8_t>(e.type);
    r.threatLevel = static_cast<std::uint8_t>(level);
    return r;
}

} // namespace telemetry
} // namespace sim

#endif // SIM_TELEMETRY_HPP
