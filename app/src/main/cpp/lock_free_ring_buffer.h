#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <algorithm>

/**
 * High-Performance Single-Producer Single-Consumer (SPSC) Lock-Free Ring Buffer.
 *
 * Guarantees zero dynamic memory allocations (no malloc, no new) and zero locks
 * inside real-time audio threads (SCHED_FIFO / SCHED_RR) to prevent priority inversion
 * and garbage collection pauses.
 */
template <typename T, size_t Capacity>
class LockFreeAudioRingBuffer {
public:
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2 for power-of-two masking");

    LockFreeAudioRingBuffer() : mHead(0), mTail(0) {
        std::memset(mBuffer, 0, sizeof(mBuffer));
    }

    ~LockFreeAudioRingBuffer() = default;

    /**
     * Producer write method (called from audio feeder/decoder thread).
     * @return Number of elements successfully written.
     */
    size_t write(const T* data, size_t count) {
        if (!data || count == 0) return 0;

        const size_t head = mHead.load(std::memory_order_relaxed);
        const size_t tail = mTail.load(std::memory_order_acquire);
        const size_t available = Capacity - (head - tail);
        const size_t toWrite = std::min(count, available);

        if (toWrite == 0) return 0;

        const size_t index = head & (Capacity - 1);
        const size_t firstChunk = std::min(toWrite, Capacity - index);
        const size_t secondChunk = toWrite - firstChunk;

        std::memcpy(&mBuffer[index], data, firstChunk * sizeof(T));
        if (secondChunk > 0) {
            std::memcpy(&mBuffer[0], data + firstChunk, secondChunk * sizeof(T));
        }

        mHead.store(head + toWrite, std::memory_order_release);
        return toWrite;
    }

    /**
     * Consumer read method (called strictly inside AAudio dataCallback).
     * Allocation-free, lock-free, zero syscalls.
     * @return Number of elements successfully read into destination.
     */
    size_t read(T* destination, size_t count) {
        if (!destination || count == 0) return 0;

        const size_t tail = mTail.load(std::memory_order_relaxed);
        const size_t head = mHead.load(std::memory_order_acquire);
        const size_t available = head - tail;
        const size_t toRead = std::min(count, available);

        if (toRead == 0) return 0;

        const size_t index = tail & (Capacity - 1);
        const size_t firstChunk = std::min(toRead, Capacity - index);
        const size_t secondChunk = toRead - firstChunk;

        std::memcpy(destination, &mBuffer[index], firstChunk * sizeof(T));
        if (secondChunk > 0) {
            std::memcpy(destination + firstChunk, &mBuffer[0], secondChunk * sizeof(T));
        }

        mTail.store(tail + toRead, std::memory_order_release);
        return toRead;
    }

    size_t getAvailableRead() const {
        const size_t head = mHead.load(std::memory_order_acquire);
        const size_t tail = mTail.load(std::memory_order_relaxed);
        return head - tail;
    }

    size_t getAvailableWrite() const {
        const size_t head = mHead.load(std::memory_order_relaxed);
        const size_t tail = mTail.load(std::memory_order_acquire);
        return Capacity - (head - tail);
    }

    /**
     * Consumer-side flush: drops everything currently queued. Must only be called from the
     * consumer thread (the AAudio data callback) - it only moves the tail index.
     */
    void discardAll() {
        mTail.store(mHead.load(std::memory_order_acquire), std::memory_order_release);
    }

    void reset() {
        mHead.store(0, std::memory_order_release);
        mTail.store(0, std::memory_order_release);
    }

private:
    T mBuffer[Capacity];
    alignas(64) std::atomic<size_t> mHead;
    alignas(64) std::atomic<size_t> mTail;
};
