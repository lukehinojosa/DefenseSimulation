#include "sim/SharedMemoryChannel.hpp"

#include <algorithm>
#include <cstring>

#include <boost/interprocess/mapped_region.hpp>
#if defined(_WIN32)
#include <boost/interprocess/windows_shared_memory.hpp>
#else
#include <boost/interprocess/shared_memory_object.hpp>
#endif

namespace sim {
namespace telemetry {

namespace bip = boost::interprocess;

// Platform-native shared memory: windows_shared_memory (CreateFileMapping,
// kernel-managed, auto-released when the last handle closes) on Windows;
// shared_memory_object (POSIX shm_open) on Unix.
#if defined(_WIN32)
using ShmObject = bip::windows_shared_memory;
#else
using ShmObject = bip::shared_memory_object;
#endif

// Cross-process seqlock correctness depends on lock-free atomics living in the
// shared mapping.
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "64-bit atomics must be lock-free for cross-process use");
static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "32-bit atomics must be lock-free for cross-process use");

// Owns the shared-memory object and its mapping. The same seqlock ring layout
// is placed in the mapping on every platform.
struct ShmMapping {
    ShmObject          shm;
    bip::mapped_region region;
};

// ---------------------------------------------------------------------------
// ShmPublisher
// ---------------------------------------------------------------------------
ShmPublisher::ShmPublisher(const std::string& name, bool unlinkOnClose)
    : name_(name), unlinkOnClose_(unlinkOnClose) {
    auto m = std::make_unique<ShmMapping>();
#if defined(_WIN32)
    // Windows: size is fixed at creation; no truncate, no explicit unlink (the
    // kernel frees the section when the last handle closes).
    m->shm = ShmObject(bip::open_or_create, name_.c_str(), bip::read_write,
                       static_cast<std::size_t>(sizeof(SharedRegion)));
#else
    // POSIX: start from a clean segment so the seqlock/control state is well
    // defined even if a previous run left an object behind.
    ShmObject::remove(name_.c_str());
    m->shm = ShmObject(bip::create_only, name_.c_str(), bip::read_write);
    m->shm.truncate(static_cast<bip::offset_t>(sizeof(SharedRegion)));
#endif
    m->region = bip::mapped_region(m->shm, bip::read_write);

    region_ = static_cast<SharedRegion*>(m->region.get_address());
    mapping_ = std::move(m);

    // A freshly truncated object is zero-filled, so the atomics start at 0.
    // Publish the geometry, then mark the region ready.
    region_->slotCount  = kSlotCount;
    region_->maxRecords = kMaxRecordsPerFrame;
    region_->latestSlot.store(0, std::memory_order_relaxed);
    for (std::uint32_t i = 0; i < kSlotCount; ++i) {
        region_->slots[i].seq.store(0, std::memory_order_relaxed);
    }
    region_->ready.store(1, std::memory_order_release);
}

ShmPublisher::~ShmPublisher() {
    mapping_.reset(); // unmap (closes the last handle)
#if !defined(_WIN32)
    // POSIX shm persists past process exit, so unlink it explicitly. Windows
    // sections are reclaimed automatically once the last handle is closed.
    if (unlinkOnClose_) {
        ShmObject::remove(name_.c_str());
    }
#else
    (void)unlinkOnClose_;
#endif
}

void ShmPublisher::publish(std::uint64_t frameId,
                           std::uint64_t timestampNs,
                           std::uint32_t interceptCount,
                           const TelemetryRecord* records,
                           std::uint32_t count) {
    const std::uint32_t n = std::min(count, region_->maxRecords);
    RingSlot& slot = region_->slots[nextSlot_];

    // Enter the seqlock: bump to an odd value to mark "write in progress".
    const std::uint64_t base = slot.seq.load(std::memory_order_relaxed);
    slot.seq.store(base + 1, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_release);

    slot.header.magic          = kMagic;
    slot.header.version        = kProtocolVersion;
    slot.header.frameId        = frameId;
    slot.header.timestampNs    = timestampNs;
    slot.header.recordCount    = n;
    slot.header.interceptCount = interceptCount;
    if (n > 0) {
        std::memcpy(slot.records, records, static_cast<std::size_t>(n) *
                                               sizeof(TelemetryRecord));
    }

    // Leave the seqlock: back to an even value ("complete").
    std::atomic_thread_fence(std::memory_order_release);
    slot.seq.store(base + 2, std::memory_order_release);

    // Publish this slot as the latest, then advance for the next frame.
    region_->latestSlot.store(nextSlot_, std::memory_order_release);
    nextSlot_ = (nextSlot_ + 1) % region_->slotCount;
}

// ---------------------------------------------------------------------------
// ShmSubscriber
// ---------------------------------------------------------------------------
ShmSubscriber::ShmSubscriber(const std::string& name) : name_(name) {
    auto m = std::make_unique<ShmMapping>();
    m->shm = ShmObject(bip::open_only, name_.c_str(), bip::read_only);
    m->region = bip::mapped_region(m->shm, bip::read_only);

    region_ = static_cast<const SharedRegion*>(m->region.get_address());
    mapping_ = std::move(m);
}

ShmSubscriber::~ShmSubscriber() = default;

bool ShmSubscriber::producerReady() const {
    return region_->ready.load(std::memory_order_acquire) == 1;
}

bool ShmSubscriber::latest(FrameHeader& headerOut,
                           TelemetryRecord* recordsOut,
                           std::uint32_t maxRecords,
                           std::uint32_t& outCount) const {
    if (!producerReady()) {
        return false;
    }

    const std::uint32_t slotIdx =
        region_->latestSlot.load(std::memory_order_acquire);
    if (slotIdx >= region_->slotCount) {
        return false;
    }
    const RingSlot& slot = region_->slots[slotIdx];

    // Bounded seqlock read: sample seq, copy, re-check seq is even & unchanged.
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::uint64_t s1 = slot.seq.load(std::memory_order_acquire);
        if (s1 & 1ull) {
            continue; // writer mid-update
        }
        std::atomic_thread_fence(std::memory_order_acquire);

        const FrameHeader hdr = slot.header;
        const std::uint32_t n = std::min(hdr.recordCount, maxRecords);
        if (n > 0) {
            std::memcpy(recordsOut, slot.records,
                        static_cast<std::size_t>(n) * sizeof(TelemetryRecord));
        }

        std::atomic_thread_fence(std::memory_order_acquire);
        const std::uint64_t s2 = slot.seq.load(std::memory_order_acquire);
        if (s1 == s2) {
            if (hdr.magic != kMagic) {
                return false; // nothing published yet
            }
            headerOut = hdr;
            outCount  = n;
            return true;
        }
    }
    return false; // could not obtain a stable snapshot
}

} // namespace telemetry
} // namespace sim
