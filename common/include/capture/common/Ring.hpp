/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

/*| role: the ring both device block families stage samples in, and every helper that reads or
        replaces one.
    frame: a receive block's device callback fills the ring and the block's own thread drains
        it; a transmit block's work call fills it and the drain hands it to the device. The two
        directions use the same type and the same helpers.
    why: the ring alone, with no lifecycle and no GNU Radio 4, so the sink vocabulary that
        states the full-scale contract can work on the type rather than on a declaration of it.
        capture/common/Device.hpp carries this header for every block that also needs the
        guards.
    invariant: a function whose signature names SpscRing lives here. The sizing arithmetic that
        names only a rate stays with the family that states the rate.
*/
namespace capture {

/*| role: the allocator a ring's storage takes, which leaves a new element unwritten.
    contract: an element default-inserted into a vector with this allocator keeps whatever the
        allocation holds where the type is trivially copyable and trivially destructible, and is
        default-initialized otherwise. Every other construction is the standard allocator's.
    why: a ring reads only the elements its producer wrote, so an initial value is a pass over
        the whole storage that nothing reads. std::complex has a constructor that zeroes it, so
        default-initialization alone still writes every element. A transmit queue at 61.44 MS/s
        is 128 MiB, and that pass ran before the first sample could go out.
    frame: memory from the allocation creates objects of an implicit-lifetime type as they are
        used, and every type with a trivial copy and a trivial destructor is one.
    verified-by: ring.a-resize-writes-no-element
*/
template <typename T>
struct OverwriteAllocator : std::allocator<T> {
    using value_type = T;
    template <typename U>
    struct rebind {
        using other = OverwriteAllocator<U>;
    };

    OverwriteAllocator() noexcept = default;
    template <typename U>
    OverwriteAllocator(const OverwriteAllocator<U>& /*other*/) noexcept {}

    template <typename U>
    void construct(U* p) noexcept(std::is_nothrow_default_constructible_v<U>) {
        if constexpr (!(std::is_trivially_copyable_v<U> && std::is_trivially_destructible_v<U>)) {
            ::new (static_cast<void*>(p)) U;
        }
    }
    template <typename U, typename... Args>
    void construct(U* p, Args&&... args) {
        ::new (static_cast<void*>(p)) U(std::forward<Args>(args)...);
    }
};

template <typename T, typename U>
constexpr bool operator==(const OverwriteAllocator<T>& /*a*/, const OverwriteAllocator<U>& /*b*/) noexcept {
    return true;
}

// The storage of a ring.
template <typename T>
using RingStorage = std::vector<T, OverwriteAllocator<T>>;

/*| role: a single-producer single-consumer drop-newest ring between a device library's
        streaming callback and the block's own thread.
    frame: power-of-two capacity with masked indexing; head and tail are free-running absolute
        counters, never wrapped, so at(absIndex) masks internally. A capacity that is not a
        power of two is rounded up to one, and zero becomes one, so buf.size() is always a
        power of two and always has room for an element.
    contract: the producer must never block, often being inside a vendor library's event
        loop. A chunk that does not fit is dropped whole and counted — dropped is the counter
        behind the rx_overflow tag — and there are never partial writes. One producer thread,
        one consumer thread; reset() only while neither is running.
    verified-by: ring.refused-chunk-counted-whole
*/
template <typename T>
struct SpscRing {
    RingStorage<T>             buf;
    std::atomic<std::size_t>   head{0}; // written by the producer (callback)
    std::atomic<std::size_t>   tail{0}; // written by the consumer
    std::atomic<std::uint64_t> dropped{0};

    /*| contract: a ring holding at least capacity elements, rounded up to the power of two the
            indexing needs.
        trap: mask() is buf.size() - 1, so a capacity that is not a power of two aliases indices
            onto each other: at() and the producer's two memcpy calls then read and write the
            wrong slots and the ring hands back elements nobody wrote, with no counter moving. A
            capacity of zero makes mask() the largest std::size_t. The name of the parameter is
            not a guard, and every caller computes its own capacity.
        verified-by: ring.capacity-rounds-up-to-a-power-of-two
    */
    explicit SpscRing(std::size_t capacity) : buf(std::bit_ceil(std::max<std::size_t>(capacity, 1UZ))) {}

    std::size_t mask() const { return buf.size() - 1; }

    void reset() {
        head.store(0, std::memory_order_relaxed);
        tail.store(0, std::memory_order_relaxed);
        dropped.store(0, std::memory_order_relaxed);
    }

    // Producer: copy n contiguous elements (memcpy fast path). Drops the whole
    // chunk when it does not fit.
    bool write(const T* src, std::size_t n) {
        const std::size_t h    = head.load(std::memory_order_relaxed);
        const std::size_t t    = tail.load(std::memory_order_acquire);
        const std::size_t free = buf.size() - (h - t);
        if (n > free) {
            dropped.fetch_add(n, std::memory_order_relaxed);
            return false;
        }
        const std::size_t pos   = h & mask();
        const std::size_t first = std::min<std::size_t>(n, buf.size() - pos);
        std::memcpy(buf.data() + pos, src, first * sizeof(T));
        std::memcpy(buf.data(), src + first, (n - first) * sizeof(T));
        head.store(h + n, std::memory_order_release);
        return true;
    }

    // Producer: element-wise variant for callbacks that convert while writing, from
    // interleaved integers to CF32 for one. fill(slot, i) writes element i.
    template <typename Fill>
    bool writeWith(std::size_t n, Fill&& fill) {
        const std::size_t h    = head.load(std::memory_order_relaxed);
        const std::size_t t    = tail.load(std::memory_order_acquire);
        const std::size_t free = buf.size() - (h - t);
        if (n > free) {
            dropped.fetch_add(n, std::memory_order_relaxed);
            return false;
        }
        for (std::size_t i = 0; i < n; ++i) {
            fill(buf[(h + i) & mask()], i);
        }
        head.store(h + n, std::memory_order_release);
        return true;
    }

    // Consumer: elements readable right now; tailOut = the absolute read position
    // to pass to at()/advanceTo().
    std::size_t available(std::size_t& tailOut) const {
        const std::size_t h = head.load(std::memory_order_acquire);
        tailOut             = tail.load(std::memory_order_relaxed);
        return h - tailOut;
    }

    const T& at(std::size_t absIndex) const { return buf[absIndex & mask()]; }

    void advanceTo(std::size_t newTail) { tail.store(newTail, std::memory_order_release); }
};

/*| role: the wait a ring's one consumer makes for the ring to change, and the wake the threads
        that change it give.
    contract: the consumer calls arm(), reads again every condition that made it decide to
        wait, and then calls disarm() where one of them has changed and wait() with arm()'s
        answer where none has. A thread that changes one of those conditions calls ring() after
        the change. A ring() made after arm() ends the wait; the consumer's reads after arm()
        see the change a ring() made before it.
    why: the consumer raises a flag and the ringer reads it, each behind a sequentially
        consistent fence, so one of the two always sees the other's write. A ringer that finds
        no consumer waiting makes no system call, and a producer publishing into a ring its
        consumer is draining pays a fence and a load for the publish.
    frame: the wait is a futex wait on a 32-bit counter where the platform has one, and it
        carries no timeout. A thread that stops the consumer rings after it raises the stop.
    verified-by: ring.a-publish-wakes-a-waiting-consumer
*/
struct RingDoorbell {
    std::atomic<std::uint32_t> epoch{0};
    std::atomic<bool>          waiting{false};

    std::uint32_t arm() noexcept {
        waiting.store(true, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_seq_cst);
        return epoch.load(std::memory_order_acquire);
    }

    void disarm() noexcept { waiting.store(false, std::memory_order_relaxed); }

    void wait(std::uint32_t token) noexcept {
        epoch.wait(token, std::memory_order_acquire);
        waiting.store(false, std::memory_order_relaxed);
    }

    void ring() noexcept {
        std::atomic_thread_fence(std::memory_order_seq_cst);
        if (waiting.load(std::memory_order_relaxed)) {
            epoch.fetch_add(1, std::memory_order_release);
            epoch.notify_one();
        }
    }
};

/*| contract: the elements the ring holds right now, bounded by a capacity the caller gives.
        The answer never exceeds that capacity, so every deadline sized from it is bounded.
    trap: tail is loaded first and head second, and the order is the whole of the guarantee. A
        head loaded first lets the producer advance head and the consumer then advance tail past
        the value already read, and the unsigned difference wraps to about 1.8e19; a wait sized
        from that is no wait at all. A tail read first is never above a head read after it, so
        the guarantee the order buys is that the subtraction never wraps. The reading can still
        be above the fill at either instant, both ends having moved between the two loads, and
        the clamp to the capacity bounds it.
    why: the capacity is an argument rather than ring.buf.size() because a rate change replaces
        the ring's storage, and a reader on a third thread asking that vector how large it is
        asks it while it is being reassigned. Each end and each reader keeps the capacity where
        it can see it.
    verified-by: ring.a-queue-reading-reads-no-storage
*/
template <typename T>
std::size_t ringFillBounded(const SpscRing<T>& ring, std::size_t capacity) {
    const std::size_t tail = ring.tail.load(std::memory_order_acquire);
    const std::size_t head = ring.head.load(std::memory_order_acquire);
    return std::min(head - tail, capacity);
}

/*| contract: the same reading for a caller that owns the ring's storage and can read its size.
    verified-by: ring.a-queue-reading-is-bounded-by-the-ring
*/
template <typename T>
std::size_t ringFill(const SpscRing<T>& ring) {
    return ringFillBounded(ring, ring.buf.size());
}

// Elements the ring has room for right now.
template <typename T>
std::size_t ringRoom(const SpscRing<T>& ring) {
    return ring.buf.size() - ringFill(ring);
}

/*| contract: copy what fits and answer how many elements went in, which is zero for a full
        ring. Nothing is dropped and the ring's own drop counter never moves.
    why: the ring's plain write drops a chunk that does not fit and counts it, which is the
        right answer for a device callback that may not block and the wrong one for a work call:
        a sink that cannot place a sample consumes nothing and is offered the same samples
        again.
    verified-by: ring.a-full-ring-consumes-nothing
*/
template <typename T>
std::size_t writeWhatFits(SpscRing<T>& ring, const T* src, std::size_t n) {
    const std::size_t take = std::min(n, ringRoom(ring));
    if (take == 0UZ) {
        return 0UZ;
    }
    std::ignore = ring.write(src, take);
    return take;
}

/*| contract: the same as writeWhatFits, each element passing through fn on its way in, so the
        copy and the transform are one pass over the samples.
    verified-by: ring.a-write-passes-each-element-through-the-transform
*/
template <typename T, typename Fn>
std::size_t writeWhatFitsWith(SpscRing<T>& ring, const T* src, std::size_t n, Fn&& fn) {
    const std::size_t take = std::min(n, ringRoom(ring));
    if (take == 0UZ) {
        return 0UZ;
    }
    const std::size_t h     = ring.head.load(std::memory_order_relaxed);
    const std::size_t pos   = h & ring.mask();
    const std::size_t first = std::min(take, ring.buf.size() - pos);
    std::transform(src, src + first, ring.buf.data() + pos, fn);
    std::transform(src + first, src + take, ring.buf.data(), fn);
    ring.head.store(h + take, std::memory_order_release);
    return take;
}

/*| contract: the storage size a ring takes for a capacity: the power of two at or above it,
        and one for zero.
    verified-by: ring.capacity-rounds-up-to-a-power-of-two
*/
inline std::size_t ringCapacityFor(std::size_t capacity) { return std::bit_ceil(std::max<std::size_t>(capacity, 1UZ)); }

/*| contract: give a ring room for at least capacity elements, rounded up to the power of two
        its indexing needs, and empty it. Storage of exactly that size stays in place. Other
        storage is replaced, by spare where spare holds exactly that size and by a fresh
        allocation otherwise, and the storage replaced goes back in spare. A resize that asks
        for less gives the difference back.
    why: the storage replaced goes back to the caller so a caller holding a lock can free it
        after letting the lock go. Freeing a queue that was written through returns every page
        of it, which at 128 MiB takes milliseconds.
    why: a fresh vector swapped in rather than assign, which keeps the capacity it already
        holds when the new count is smaller: a ring sized for a fast rate would hold that storage
        for the life of the block after a rate change down to a slow one. The swap is one
        allocation and one free whichever way the size moves, where shrink_to_fit behind an
        assign is a request the library may refuse.
    invariant: the caller holds the ring alone. Neither the producer nor the consumer may be
        running, since this replaces the storage both of them index.
    verified-by: ring.a-resize-down-gives-the-storage-back
    verified-by: ring.a-resize-keeps-storage-that-fits
*/
template <typename T>
void resizeRing(SpscRing<T>& ring, std::size_t capacity, RingStorage<T>& spare) {
    const std::size_t size = ringCapacityFor(capacity);
    if (ring.buf.size() != size) {
        if (spare.size() == size) {
            ring.buf.swap(spare);
        } else {
            RingStorage<T> fresh(size);
            ring.buf.swap(fresh);
            spare.swap(fresh);
        }
    }
    ring.reset();
}

// The same resize, freeing the storage it replaces before it returns.
template <typename T>
void resizeRing(SpscRing<T>& ring, std::size_t capacity) {
    RingStorage<T> spare;
    resizeRing(ring, capacity, spare);
}

/*| contract: the same resize, answering how many elements it threw away: the elements the
        ring still held, which for a rate change are the ones the play-out before it could not
        place at the rate they were staged at.
    why: head and tail at zero is the only state both ends can be handed, so the elements left
        cannot be carried across. They have no rate to go out at either: the path that leaves
        them there is a device that stopped consuming. A caller adds the answer to a counter of
        its own, as the receive side counts a drop.
    verified-by: ring.a-resize-counts-what-it-discards
*/
template <typename T>
std::size_t resizeRingCountingDiscard(SpscRing<T>& ring, std::size_t capacity, RingStorage<T>& spare) {
    const std::size_t left = ringFill(ring);
    resizeRing(ring, capacity, spare);
    return left;
}

template <typename T>
std::size_t resizeRingCountingDiscard(SpscRing<T>& ring, std::size_t capacity) {
    RingStorage<T> spare;
    return resizeRingCountingDiscard(ring, capacity, spare);
}

} // namespace capture
