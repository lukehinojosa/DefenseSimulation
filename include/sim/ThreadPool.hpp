#ifndef SIM_THREADPOOL_HPP
#define SIM_THREADPOOL_HPP

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace sim {

/**
 * @brief Fixed-size worker-thread pool for data-parallel simulation work.
 *
 * Worker threads are spawned once at construction and reused for the life of
 * the pool, avoiding the per-frame cost of creating and joining threads. The
 * primary entry point is parallelFor(), which partitions an index range into
 * contiguous chunks and blocks until every chunk has completed.
 *
 * Thread-safety: enqueue() and parallelFor() are safe to call from the owning
 * thread. parallelFor() is intended to be driven from a single orchestrator
 * thread (the simulation step), which is the common master/worker pattern.
 */
class ThreadPool {
public:
    /**
     * @param threadCount Number of worker threads. When 0, defaults to the
     *        hardware concurrency (at least 1).
     */
    explicit ThreadPool(std::size_t threadCount = 0) {
        if (threadCount == 0) {
            threadCount = std::thread::hardware_concurrency();
            if (threadCount == 0) {
                threadCount = 1;
            }
        }
        workers_.reserve(threadCount);
        for (std::size_t i = 0; i < threadCount; ++i) {
            workers_.emplace_back([this] { workerLoop(); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& w : workers_) {
            if (w.joinable()) {
                w.join();
            }
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    std::size_t threadCount() const { return workers_.size(); }

    /// Enqueue a single task to run on some worker thread.
    void enqueue(std::function<void()> task) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.push(std::move(task));
        }
        cv_.notify_one();
    }

    /**
     * @brief Apply @p body to every index in [begin, end) across the pool.
     * @param body Callable invoked as body(std::size_t index). Because the
     *        range is partitioned into disjoint chunks, distinct indices are
     *        never processed concurrently on overlapping data, so a body that
     *        only writes element `index` needs no synchronization.
     *
     * Blocks until all indices have been processed. Runs inline when the range
     * is empty or the pool has a single worker.
     */
    template <typename Body>
    void parallelFor(std::size_t begin, std::size_t end, Body&& body) {
        if (begin >= end) {
            return;
        }

        const std::size_t total = end - begin;
        const std::size_t workers = workers_.size();

        // For tiny ranges the dispatch overhead dominates; just run inline.
        if (workers <= 1 || total <= 1) {
            for (std::size_t i = begin; i < end; ++i) {
                body(i);
            }
            return;
        }

        // Aim for a handful of chunks per worker to balance uneven load while
        // keeping scheduling overhead low.
        const std::size_t targetChunks = workers * 4;
        std::size_t chunkSize = (total + targetChunks - 1) / targetChunks;
        if (chunkSize == 0) {
            chunkSize = 1;
        }
        const std::size_t chunkCount = (total + chunkSize - 1) / chunkSize;

        // Reuse the pool's persistent completion primitives rather than
        // constructing a mutex/condition_variable per call. Some pthread
        // implementations (notably MinGW winpthreads) do not reliably reclaim
        // those objects across rapid create/destroy cycles, which manifests as
        // a spurious "Not enough space" system_error after many frames.
        remaining_.store(chunkCount, std::memory_order_release);

        for (std::size_t c = 0; c < chunkCount; ++c) {
            const std::size_t lo = begin + c * chunkSize;
            const std::size_t hi = std::min(lo + chunkSize, end);
            enqueue([this, lo, hi, &body] {
                for (std::size_t i = lo; i < hi; ++i) {
                    body(i);
                }
                if (remaining_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                    std::lock_guard<std::mutex> lock(completionMutex_);
                    completionCv_.notify_one();
                }
            });
        }

        std::unique_lock<std::mutex> lock(completionMutex_);
        completionCv_.wait(lock, [this] {
            return remaining_.load(std::memory_order_acquire) == 0;
        });
    }

private:
    void workerLoop() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
                if (stop_ && tasks_.empty()) {
                    return;
                }
                task = std::move(tasks_.front());
                tasks_.pop();
            }
            task();
        }
    }

    std::vector<std::thread>          workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex                        mutex_;
    std::condition_variable           cv_;
    bool                              stop_{false};

    // Persistent completion signalling for parallelFor() (single orchestrator).
    std::mutex                        completionMutex_;
    std::condition_variable           completionCv_;
    std::atomic<std::size_t>          remaining_{0};
};

} // namespace sim

#endif // SIM_THREADPOOL_HPP
