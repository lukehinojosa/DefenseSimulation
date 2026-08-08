#include "sim/SharedMemoryChannel.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <system_error>

namespace sim {
namespace telemetry {

// Cross-process seqlock correctness depends on lock-free atomics living in the
// shared mapping.
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "64-bit atomics must be lock-free for cross-process use");
static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "32-bit atomics must be lock-free for cross-process use");

namespace {

[[noreturn]] void throwErrno(const char* what) {
    throw std::system_error(errno, std::generic_category(), what);
}

} // namespace

// ---------------------------------------------------------------------------
// ShmPublisher
// ---------------------------------------------------------------------------
ShmPublisher::ShmPublisher(const std::string& name, bool unlinkOnClose)
    : name_(name), unlinkOnClose_(unlinkOnClose) {
    mappedSize_ = sizeof(SharedRegion);

    fd_ = ::shm_open(name_.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd_ == -1) {
        throwErrno("shm_open");
    }
    if (::ftruncate(fd_, static_cast<off_t>(mappedSize_)) == -1) {
        const int e = errno;
        ::close(fd_);
        ::shm_unlink(name_.c_str());
        throw std::system_error(e, std::generic_category(), "ftruncate");
    }

    void* addr = ::mmap(nullptr, mappedSize_, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd_, 0);
    if (addr == MAP_FAILED) {
        const int e = errno;
        ::close(fd_);
        ::shm_unlink(name_.c_str());
        throw std::system_error(e, std::generic_category(), "mmap");
    }

    region_ = static_cast<SharedRegion*>(addr);

    // First mapping of a freshly ftruncate'd object is zero-filled, so the
    // atomics start at 0. Publish the geometry, then mark the region ready.
    region_->slotCount  = kSlotCount;
    region_->maxRecords = kMaxRecordsPerFrame;
    region_->latestSlot.store(0, std::memory_order_relaxed);
    for (std::uint32_t i = 0; i < kSlotCount; ++i) {
        region_->slots[i].seq.store(0, std::memory_order_relaxed);
    }
    region_->ready.store(1, std::memory_order_release);
}

ShmPublisher::~ShmPublisher() {
    if (region_ != nullptr) {
        ::munmap(region_, mappedSize_);
    }
    if (fd_ != -1) {
        ::close(fd_);
    }
    if (unlinkOnClose_) {
        ::shm_unlink(name_.c_str());
    }
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
    mappedSize_ = sizeof(SharedRegion);

    fd_ = ::shm_open(name_.c_str(), O_RDONLY, 0600);
    if (fd_ == -1) {
        throwErrno("shm_open");
    }
    void* addr = ::mmap(nullptr, mappedSize_, PROT_READ, MAP_SHARED, fd_, 0);
    if (addr == MAP_FAILED) {
        const int e = errno;
        ::close(fd_);
        throw std::system_error(e, std::generic_category(), "mmap");
    }
    region_ = static_cast<const SharedRegion*>(addr);
}

ShmSubscriber::~ShmSubscriber() {
    if (region_ != nullptr) {
        ::munmap(const_cast<SharedRegion*>(region_), mappedSize_);
    }
    if (fd_ != -1) {
        ::close(fd_);
    }
}

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
