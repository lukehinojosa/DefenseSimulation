#ifndef SIM_SHAREDMEMORYCHANNEL_HPP
#define SIM_SHAREDMEMORYCHANNEL_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "sim/Telemetry.hpp"

namespace sim {
namespace telemetry {

/**
 * @brief One ring slot: a seqlock counter guarding a frame header + records.
 *
 * The sequence counter is odd while the writer is mutating the slot and even
 * when the slot holds a complete, consistent frame. A reader that samples an
 * odd count, or a count that changes across its copy, retries. Combined with a
 * ring depth >= 3 (the writer advances to a different slot each frame) torn
 * reads are effectively impossible in steady state.
 */
struct alignas(64) RingSlot {
    std::atomic<std::uint64_t> seq;
    FrameHeader                header;
    TelemetryRecord            records[kMaxRecordsPerFrame];
};

/**
 * @brief Fixed-layout shared region mapped into every participating process.
 *
 * The whole structure is a trivially-copyable POD placed directly in the
 * shared mapping; it performs no dynamic allocation. Cross-process atomics are
 * required to be lock-free (checked at compile time in the .cpp).
 */
struct SharedRegion {
    std::atomic<std::uint32_t> ready;      // 1 once the writer has initialized
    std::atomic<std::uint32_t> latestSlot; // most recently published slot
    std::uint32_t              slotCount;
    std::uint32_t              maxRecords;
    RingSlot                   slots[kSlotCount];
};

/**
 * @brief Writer end of the shared-memory telemetry ring.
 *
 * Creates (or attaches to) the POSIX shared-memory object, maps it, and
 * publishes frames with no allocation on the hot path — records are copied
 * straight into the mapped ring.
 */
class ShmPublisher {
public:
    /**
     * @param name    POSIX shm object name (e.g. "/defsim_telemetry").
     * @param unlinkOnClose  If true, shm_unlink() the object in the destructor
     *        (the owning/producer process should set this).
     * @throws std::system_error on shm_open/ftruncate/mmap failure.
     */
    explicit ShmPublisher(const std::string& name = kDefaultShmName,
                          bool unlinkOnClose = true);
    ~ShmPublisher();

    ShmPublisher(const ShmPublisher&) = delete;
    ShmPublisher& operator=(const ShmPublisher&) = delete;

    /**
     * @brief Publish one frame. Copies @p count records (clamped to capacity)
     *        into the next ring slot under the seqlock, then advances the
     *        published-slot index. No heap allocation occurs.
     */
    void publish(std::uint64_t frameId,
                 std::uint64_t timestampNs,
                 std::uint32_t interceptCount,
                 const TelemetryRecord* records,
                 std::uint32_t count);

    const std::string& name() const { return name_; }

private:
    std::string   name_;
    bool          unlinkOnClose_;
    int           fd_{-1};
    std::size_t   mappedSize_{0};
    SharedRegion* region_{nullptr};
    std::uint32_t nextSlot_{0};
};

/**
 * @brief Reader end of the shared-memory telemetry ring.
 *
 * Attaches read-only to an existing shared-memory object published by a
 * ShmPublisher and snapshots the latest complete frame via the seqlock.
 */
class ShmSubscriber {
public:
    /// @throws std::system_error if the object cannot be opened/mapped.
    explicit ShmSubscriber(const std::string& name = kDefaultShmName);
    ~ShmSubscriber();

    ShmSubscriber(const ShmSubscriber&) = delete;
    ShmSubscriber& operator=(const ShmSubscriber&) = delete;

    /// True once the publisher has initialized the region.
    bool producerReady() const;

    /**
     * @brief Copy the latest complete frame into caller-owned buffers.
     * @param headerOut   Receives the frame header on success.
     * @param recordsOut  Destination array, at least @p maxRecords long.
     * @param maxRecords  Capacity of @p recordsOut.
     * @param outCount    Receives the number of records written.
     * @return true if a consistent frame was captured; false if none is ready
     *         or the seqlock could not stabilize within the retry budget.
     */
    bool latest(FrameHeader& headerOut,
                TelemetryRecord* recordsOut,
                std::uint32_t maxRecords,
                std::uint32_t& outCount) const;

private:
    std::string         name_;
    int                 fd_{-1};
    std::size_t         mappedSize_{0};
    const SharedRegion* region_{nullptr};
};

} // namespace telemetry
} // namespace sim

#endif // SIM_SHAREDMEMORYCHANNEL_HPP
