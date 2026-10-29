/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/Ring.hpp>

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>

/*| role: the parts every device sink block shares: the full-scale contract, the queue in front
        of the device, and the vocabulary its counters are stated in.
    frame: a sink takes CF32 on its input port, stages it in a ring and lets its own drain hand
        that ring to the device. The work call never waits: what does not fit stays on the input
        edge for the next call.
    frame: this header and capture/common/Ring.hpp include no GNU Radio 4 header.
*/
namespace capture {

using CF32 = std::complex<float>;

/*| contract: one part of a sample brought inside the range a sink states. A part that is not a
        number comes back as zero, which a sink can always carry.
    trap: clipped and never wrapped. A part past full scale cast into an integer format wraps to
        the opposite sign, which turns an overdriven sample into a loud one of the wrong
        polarity — a different signal, spread across the whole band, rather than a distorted
        version of the intended one. A NaN answers false to both comparisons, so a pair of them
        alone hands it straight on: the integer conversions then round it to whatever is left,
        against a contract that states full scale 1.0.
    why: a sink converts each part through this function, so the clip holds for a new sink
        as well.
    verified-by: sink.overrange-clips-not-wraps
*/
constexpr float clipUnit(float v) noexcept {
    if (v != v) {
        return 0.0f;
    }
    return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
}

// Both parts of a sample clipped independently — the rectangle, not the disc,
// because it is the per-part integer conversion that wraps.
constexpr CF32 clipUnit(CF32 s) noexcept { return {clipUnit(s.real()), clipUnit(s.imag())}; }

/*| role: what a transmit block counts, in one place so the two families state the same
        quantities.
    frame: underrunSamples counts silence the device sent for want of data and underrunEvents
        the transfers or device reports that silence arrived in; a device that keys down for a
        gap rather than filling it states the events alone. discardedAtHold counts the samples
        a pause threw away after the caller had handed them over. discardedAtRateChange counts
        the samples a rate change threw away, which are the ones the play-out before it could
        not place. unsentAtStop counts the samples still queued when a stop's deadline expired.
    invariant: every count is per run and is cleared by the start that begins the run, so a
        caller reads the difference between two reads within one run.
*/
struct TxCounters {
    std::atomic<std::uint64_t> underrunSamples{0};
    std::atomic<std::uint64_t> underrunEvents{0};
    std::atomic<std::uint64_t> discardedAtHold{0};
    std::atomic<std::uint64_t> discardedAtRateChange{0};
    std::atomic<std::uint64_t> unsentAtStop{0};

    void reset() {
        underrunSamples.store(0, std::memory_order_relaxed);
        underrunEvents.store(0, std::memory_order_relaxed);
        discardedAtHold.store(0, std::memory_order_relaxed);
        discardedAtRateChange.store(0, std::memory_order_relaxed);
        unsentAtStop.store(0, std::memory_order_relaxed);
    }
};

/*| role: the floor a queue sized from a rate is not taken below, in samples, for a family that
        runs no slower than 2 MS/s.
    frame: 2^19 samples is a quarter of a second at 2 MS/s. A family that runs slower states a
        floor of its own, because this one is seconds of queue at a fifth of a megasample.
*/
constexpr std::size_t kRingFloorSamples = 1UZ << 19;

/*| role: how much a queue sized from a rate holds: the time it covers at the rate, and the most
        storage it takes, in bytes. A sink states both as settings.
    frame: the defaults are a quarter of a second and 4 GiB, which holds a quarter of a second of
        CF32 at 2 GS/s rounded up to a power of two. A host without that memory states less.
*/
struct QueueBound {
    double seconds  = 0.25;
    double maxBytes = 4294967296.0;
};

/*| contract: the ring capacity for a rate, in samples of sampleBytes bytes: bound.seconds of
        the rate, never below floorSamples, rounded up to the power of two the ring indexes by,
        and never above the largest power of two of samples that fits bound.maxBytes. The floor
        wins over the ceiling. A rate that is not positive, a duration that is not positive and
        a ceiling that is not positive each leave the floor or the part of the rule that still
        holds.
    why: a quarter of a second is long enough that a scheduling delay on the thread filling the
        ring does not starve the device, and short enough that the tail a stop plays out before
        it releases the device is a quarter of a second. Every wait a sink makes is sized from
        this queue, so a floor above the rate's own quarter second lengthens all of them.
    why: the ceiling bounds the storage at the fastest rates, where the duration's worth is
        gigabytes and the device's own buffer holds microseconds.
    frame: the rounding puts the time held between the duration and just under twice it, the
        worst rate being the one just past a power of two, below the ceiling.
    verified-by: sink.the-queue-follows-the-rate
    verified-by: sink.the-queue-is-bounded-by-its-settings
*/
inline std::size_t ringSamplesFor(double rateHz, std::size_t floorSamples = kRingFloorSamples, QueueBound bound = {}, std::size_t sampleBytes = sizeof(CF32)) {
    constexpr double  kLargest = 9.0e15; // below 2^63, so every conversion below is defined
    const std::size_t floor    = std::bit_ceil(std::max<std::size_t>(floorSamples, 1UZ));
    const double      bytes    = bound.maxBytes > 0.0 ? std::min(bound.maxBytes, kLargest) : kLargest;
    const std::size_t ceiling  = std::max(floor, std::bit_floor(static_cast<std::size_t>(bytes) / std::max<std::size_t>(sampleBytes, 1UZ)));
    if (!(rateHz > 0.0) || !(bound.seconds > 0.0)) {
        return floor;
    }
    const double held = std::min(rateHz * bound.seconds, static_cast<double>(ceiling));
    return std::min(ceiling, std::bit_ceil(std::max(static_cast<std::size_t>(held), floor)));
}

/*| contract: how long a queue of queuedSamples takes to play out at rateHz, in milliseconds,
        with a margin for the transfers the device library already holds and a floor for a rate
        that bounds nothing.
    why: every wait in a sink is sized from it. A stop waits this long for the queue to reach
        the air, so a device that has stopped draining costs a bounded delay rather than a
        thread that never returns.
    verified-by: sink.waits-are-bounded-by-the-queue
*/
inline std::chrono::milliseconds playOutMs(std::uint64_t queuedSamples, double rateHz) {
    constexpr double kQueueMarginMs = 250.0;
    constexpr double kQueueFloorMs  = 250.0;
    const double     ms             = rateHz > 0.0 ? 1000.0 * static_cast<double>(queuedSamples) / rateHz + kQueueMarginMs : kQueueFloorMs + kQueueMarginMs;
    return std::chrono::milliseconds(static_cast<long>(std::max(ms, kQueueFloorMs)));
}

/*| role: what a rate change owes the device: whether the rate reaches it at all, whether the
        burst in flight ends first, and whether the transfer starts again behind it.
    contract: a request equal to the rate in force, or one that is not a positive number, asks
        for nothing. A new rate always reaches the device; the burst ends and the transfer
        restarts only where one is running.
    why: the staged settings map is replayed shortly after the stream starts, so a rate applier
        runs at least twice with the same value. A rate written while a transfer runs can stop a
        device's sample clock until it is reset, and a rate given mid-burst sends the samples
        already staged at the old one.
    verified-by: sink.a-rate-change-ends-the-burst
*/
struct RateChange {
    bool write    = false;
    bool endBurst = false;
    bool restart  = false;
};

inline RateChange rateChangeFor(double requestHz, double inForceHz, bool streaming) {
    if (!(requestHz > 0.0) || requestHz == inForceHz) {
        return {};
    }
    return {.write = true, .endBurst = streaming, .restart = streaming};
}

} // namespace capture
