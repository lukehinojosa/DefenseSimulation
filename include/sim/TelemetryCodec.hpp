#ifndef SIM_TELEMETRYCODEC_HPP
#define SIM_TELEMETRYCODEC_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "sim/Telemetry.hpp"

namespace sim {
namespace telemetry {
namespace codec {

// ---------------------------------------------------------------------------
// Low-level primitives (zig-zag + variable-length quantity), styled after the
// reference codec in online-pong/shared/include/pong/codec.h.
// ---------------------------------------------------------------------------

/// Zig-zag: map signed -> unsigned so small |n| stays small under VLQ.
inline std::uint32_t zzEnc(std::int32_t n) {
    return static_cast<std::uint32_t>((n << 1) ^ (n >> 31));
}
inline std::int32_t zzDec(std::uint32_t n) {
    return static_cast<std::int32_t>((n >> 1) ^ (0u - (n & 1u)));
}

/// VLQ: 7 data bits per byte, MSB set means "more bytes follow".
inline int vlqWrite(std::uint8_t* buf, std::uint32_t v) {
    int n = 0;
    do {
        std::uint8_t b = v & 0x7Fu;
        v >>= 7;
        buf[n++] = b | (v ? 0x80u : 0u);
    } while (v);
    return n;
}
inline std::uint32_t vlqRead(const std::uint8_t*& p, const std::uint8_t* end) {
    std::uint32_t v = 0;
    int shift = 0;
    while (p < end && shift < 32) {
        std::uint8_t b = *p++;
        v |= static_cast<std::uint32_t>(b & 0x7Fu) << shift;
        if (!(b & 0x80u)) break;
        shift += 7;
    }
    return v;
}

// ---------------------------------------------------------------------------
// Wire framing
// ---------------------------------------------------------------------------
constexpr std::uint8_t kTag         = 0xD5;  // datagram identifier
constexpr std::uint8_t kCodecVersion = 1;
/// Worst-case bytes for one encoded record (id 5 + packed 1 + 3*pos 3 +
/// 3*vel 3 + target 5 = 29); a round 32 is a safe scratch size.
constexpr std::size_t kMaxRecordBytes = 32;

/// Quantize a position component (meters, >= 0) to an integer.
inline std::uint32_t quantPos(float v) {
    if (v < 0.0f) return 0u;
    return static_cast<std::uint32_t>(std::lround(v));
}

inline int encodeRecord(std::uint8_t* b, const TelemetryRecord& r) {
    std::uint8_t* p = b;
    p += vlqWrite(p, r.entityId);
    // Pack type(2) | threat(3) | flags(3) into one byte.
    const std::uint8_t packed =
        static_cast<std::uint8_t>((r.entityType & 0x3u) |
                                  ((r.threatLevel & 0x7u) << 2) |
                                  ((r.flags & 0x7u) << 5));
    *p++ = packed;
    p += vlqWrite(p, quantPos(r.posX));
    p += vlqWrite(p, quantPos(r.posY));
    p += vlqWrite(p, quantPos(r.posZ));
    p += vlqWrite(p, zzEnc(static_cast<std::int32_t>(std::lround(r.velX))));
    p += vlqWrite(p, zzEnc(static_cast<std::int32_t>(std::lround(r.velY))));
    p += vlqWrite(p, zzEnc(static_cast<std::int32_t>(std::lround(r.velZ))));
    // targetId+1 so that 0 encodes "no target" compactly.
    p += vlqWrite(p, r.targetId == kNoTargetId ? 0u : r.targetId + 1u);
    return static_cast<int>(p - b);
}

inline void decodeRecord(const std::uint8_t*& p, const std::uint8_t* end,
                         TelemetryRecord& r) {
    r.entityId = vlqRead(p, end);
    const std::uint8_t packed = (p < end) ? *p++ : 0u;
    r.entityType  = packed & 0x3u;
    r.threatLevel = (packed >> 2) & 0x7u;
    r.flags       = (packed >> 5) & 0x7u;
    r.posX = static_cast<float>(vlqRead(p, end));
    r.posY = static_cast<float>(vlqRead(p, end));
    r.posZ = static_cast<float>(vlqRead(p, end));
    r.velX = static_cast<float>(zzDec(vlqRead(p, end)));
    r.velY = static_cast<float>(zzDec(vlqRead(p, end)));
    r.velZ = static_cast<float>(zzDec(vlqRead(p, end)));
    const std::uint32_t t = vlqRead(p, end);
    r.targetId = (t == 0u) ? kNoTargetId : (t - 1u);
}

/**
 * @brief Encode a frame header + as many records as fit in @p maxBytes.
 * @param records Priority-ordered; the greedy pack keeps the most important
 *        ones when the budget is exceeded.
 * @param encodedCount Receives how many records were actually written.
 * @return total bytes written.
 */
inline std::size_t encodeFrame(std::uint8_t* buf, std::size_t maxBytes,
                               const FrameHeader& hdr,
                               const TelemetryRecord* records,
                               std::uint32_t count,
                               std::uint32_t& encodedCount) {
    std::uint8_t* p = buf;
    *p++ = kTag;
    *p++ = kCodecVersion;
    p += vlqWrite(p, static_cast<std::uint32_t>(hdr.frameId));
    p += vlqWrite(p, static_cast<std::uint32_t>(hdr.timestampNs / 1000000ull));
    p += vlqWrite(p, hdr.interceptCount);
    std::uint8_t* countPos = p++;  // 1-byte record count, backfilled below

    std::uint32_t enc = 0;
    std::uint8_t scratch[kMaxRecordBytes];
    for (std::uint32_t i = 0; i < count && enc < 255u; ++i) {
        const int rn = encodeRecord(scratch, records[i]);
        if (static_cast<std::size_t>(p - buf) + rn > maxBytes) {
            break;  // budget reached; remaining (lower-priority) records dropped
        }
        std::memcpy(p, scratch, static_cast<std::size_t>(rn));
        p += rn;
        ++enc;
    }
    *countPos = static_cast<std::uint8_t>(enc);
    encodedCount = enc;
    return static_cast<std::size_t>(p - buf);
}

/**
 * @brief Decode a frame produced by encodeFrame.
 * @return true on a valid datagram; fills @p hdr (magic/version normalized to
 *         the internal protocol) and up to @p maxRecords records.
 */
inline bool decodeFrame(const std::uint8_t* buf, std::size_t len,
                        FrameHeader& hdr, TelemetryRecord* out,
                        std::uint32_t maxRecords, std::uint32_t& outCount) {
    if (len < 4) return false;
    const std::uint8_t* p = buf;
    const std::uint8_t* end = buf + len;
    if (*p++ != kTag) return false;
    if (*p++ != kCodecVersion) return false;

    const std::uint32_t frameId = vlqRead(p, end);
    const std::uint32_t tsMs    = vlqRead(p, end);
    const std::uint32_t inter   = vlqRead(p, end);
    if (p >= end) return false;
    const std::uint32_t rc = *p++;

    const std::uint32_t n = (rc < maxRecords) ? rc : maxRecords;
    for (std::uint32_t i = 0; i < n; ++i) {
        decodeRecord(p, end, out[i]);
    }

    hdr.magic          = kMagic;
    hdr.version        = kProtocolVersion;
    hdr.frameId        = frameId;
    hdr.timestampNs    = static_cast<std::uint64_t>(tsMs) * 1000000ull;
    hdr.recordCount    = n;
    hdr.interceptCount = inter;
    outCount = n;
    return true;
}

} // namespace codec
} // namespace telemetry
} // namespace sim

#endif // SIM_TELEMETRYCODEC_HPP
