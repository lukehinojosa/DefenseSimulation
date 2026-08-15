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
/// v2 adds per-record targetId (for interceptor LOS lines) and a flags byte
/// (detonation events) so the visualizer can render engagement geometry.
constexpr std::uint32_t kProtocolVersion = 2u;

/// Sentinel targetId meaning "no assigned target".
constexpr std::uint32_t kNoTargetId = 0xFFFFFFFFu;

/// Ring geometry. Slots >= 3 let a single writer cycle away from the slot a
/// reader is copying, so the seqlock almost never has to retry.
constexpr std::uint32_t kSlotCount        = 4u;
constexpr std::uint32_t kMaxRecordsPerFrame = 20000u;

/// Default shared-memory object name. No leading slash / special characters so
/// it is valid for Boost.Interprocess on both POSIX and Windows.
constexpr const char* kDefaultShmName = "defsim_telemetry";

/// Coarse threat classification carried in each record.
enum class ThreatLevel : std::uint8_t {
    None     = 0,
    Low      = 1,
    Medium   = 2,
    High     = 3,
    Critical = 4
};

/// Per-record status flags (bitmask). Only the low 3 bits survive the UDP
/// codec's packed status byte, so keep new flags within 0b111.
enum RecordFlags : std::uint8_t {
    FLAG_NONE      = 0u,
    FLAG_DESTROYED = 1u << 0, ///< Entity was destroyed this frame (detonation).
    FLAG_BOOSTER   = 1u << 1, ///< Motor lit / launch boost phase (hot trail FX).
    FLAG_ASSET_HIT = 1u << 2  ///< Leaked threat struck the defended city (loss).
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
    std::uint32_t targetId;    // interceptor's assigned hostile, or kNoTargetId
    std::uint8_t  entityType;  // sim::EntityType
    std::uint8_t  threatLevel; // sim::telemetry::ThreatLevel
    std::uint8_t  flags;       // sim::telemetry::RecordFlags
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

static_assert(sizeof(TelemetryRecord) == 35,
              "TelemetryRecord must stay 35 bytes on the wire (protocol v2)");
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
inline TelemetryRecord makeRecord(const Entity& e, ThreatLevel level,
                                  std::uint32_t targetId = kNoTargetId,
                                  std::uint8_t flags = FLAG_NONE) {
    TelemetryRecord r;
    r.entityId    = e.id;
    r.posX        = static_cast<float>(e.position.x);
    r.posY        = static_cast<float>(e.position.y);
    r.posZ        = static_cast<float>(e.position.z);
    r.velX        = static_cast<float>(e.velocity.x);
    r.velY        = static_cast<float>(e.velocity.y);
    r.velZ        = static_cast<float>(e.velocity.z);
    r.targetId    = targetId;
    r.entityType  = static_cast<std::uint8_t>(e.type);
    r.threatLevel = static_cast<std::uint8_t>(level);
    r.flags       = flags;
    return r;
}

/**
 * @brief Priority score for UDP record selection (higher = more important).
 *
 * When more records exist than fit in a datagram, the sender keeps the most
 * operationally relevant tracks: detonations and interceptors first, then
 * hostiles by threat level, then everything else.
 */
inline int recordPriority(const TelemetryRecord& r) {
    if (r.flags & FLAG_DESTROYED) return 1000;                 // detonations
    if (r.entityType == static_cast<std::uint8_t>(EntityType::Friendly) &&
        r.targetId != kNoTargetId) {
        return 900;                                            // engaged interceptor
    }
    if (r.entityType == static_cast<std::uint8_t>(EntityType::Hostile)) {
        return 500 + r.threatLevel;                            // hostiles by threat
    }
    if (r.entityType == static_cast<std::uint8_t>(EntityType::Friendly)) {
        return 400;                                            // idle interceptor
    }
    return 0;                                                  // neutral
}

} // namespace telemetry
} // namespace sim

#endif // SIM_TELEMETRY_HPP
