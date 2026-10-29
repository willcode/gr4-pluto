/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <capture/common/Device.hpp>

#include <algorithm>
#include <chrono>
#include <complex>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

/*| role: the publish side every device source block shares: the hold-and-retry publish, the
        rate and frequency tags on a stream's first block, the periodic timing tag and the gap
        marker.
*/
namespace capture {

// Wall-clock nanoseconds for the SDR_WALLCLOCK timing tags.
inline std::uint64_t wallClockNs() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

/*| contract: when the first sample of a drained block was taken, from the clock read as it was
        drained. backlogSamples counts the samples the ring held when it was read, the chunk
        about to be published included, since the first sample of that chunk is the oldest;
        rateHz is the rate those samples were taken at.
    frame: the timing tag names the first sample of its block. A clock read at the drain is
        late by the whole backlog the ring held, and this takes that backlog off.
    invariant: a rate or a backlog of zero leaves the reading alone, and a correction larger
        than the reading floors at zero.
    trap: the backlog is corrected at one rate. A caller drops what its ring holds at a rate
        change and marks the gap. A quarter-second backlog carried from 2.4 MS/s into 250 kS/s
        stamps the next block about two seconds early.
    verified-by: publish.tag-timestamps-the-first-sample
*/
inline std::uint64_t blockStartNs(std::uint64_t drainedNs, std::size_t backlogSamples, double rateHz) {
    if (rateHz <= 0.0 || backlogSamples == 0UZ) {
        return drainedNs;
    }
    const auto backlogNs = static_cast<std::uint64_t>(static_cast<double>(backlogSamples) * 1e9 / rateHz);
    return backlogNs < drainedNs ? drainedNs - backlogNs : 0UL;
}

/*| contract: the minimum interval between timing tags in nanoseconds, from the seconds a
        caller staged. An interval that is not positive reads as zero, which the decision below
        takes as a tag on every block.
    contract: no driver validates tag_interval. A negative or not-a-number interval reads as
        zero, and kCeilingSeconds bounds the rest inside the unsigned type.
    verified-by: publish.tag-interval-outside-the-useful-range
*/
inline std::uint64_t tagIntervalNs(float seconds) {
    constexpr double kCeilingSeconds = 1.0e6; // eleven days, and 1e15 ns
    if (!(seconds > 0.f)) {
        return 0UL;
    }
    return static_cast<std::uint64_t>(std::min(static_cast<double>(seconds), kCeilingSeconds) * 1e9);
}

/*| contract: whether the block about to be published carries a timing tag, and the time the
        next decision measures from. lastTagNs holds what the previous decision left and is
        zero before any tag has gone out; intervalNs comes from tagIntervalNs.
    invariant: the time this leaves is never earlier than the time it was given, so a stamp
        taken from behind cannot make the next interval read longer than it is.
    frame: tWallNs carries the backlog correction and moves backwards when a stall grows the
        backlog. A block stamped earlier than the last tag still carries a tag, and the next
        interval is measured from the later of the two times.
    verified-by: publish.tag-times-never-run-backwards
*/
struct TagDecision {
    bool          emit      = false;
    std::uint64_t lastTagNs = 0;
};

inline TagDecision tagDecision(std::uint64_t tWallNs, std::uint64_t lastTagNs, std::uint64_t intervalNs) {
    const std::uint64_t latest = std::max(tWallNs, lastTagNs);
    if (lastTagNs == 0UL || intervalNs == 0UL || tWallNs < lastTagNs) {
        return {true, latest};
    }
    if (tWallNs - lastTagNs >= intervalNs) {
        return {true, latest};
    }
    return {false, lastTagNs};
}

/*| contract: publish one tag at offset from the writer's position on the block's output, and
        on each further output the block streams where it declares publishFurtherOutputTag,
        which takes the same map and offset. Every output of such a block advances in step, so
        the offset names the same sample on each.
*/
template <typename Block, typename TagMap>
void publishOutputTag(Block& blk, TagMap&& tagMap, std::size_t offset) {
    if constexpr (requires { blk.publishFurtherOutputTag(tagMap, offset); }) {
        blk.publishFurtherOutputTag(tagMap, offset);
    }
    blk.out.publishTag(std::forward<TagMap>(tagMap), offset);
    noteDroppedTags(blk);
}

/*| contract: publish the gap marker at the head of the block the gap falls in front of, with
        the cause beside it where the caller names one. gr:rx_overflow says a gap occurred and
        rx_overflow_cause names what caused it; a caller naming nothing publishes the key alone.
    contract: where the caller counts the samples the gap lost, gr:n_dropped_samples carries
        the count as a gr::Size_t, held at that type's largest value. The count holds the
        samples lost since the last marker. A stream's counts sum to the loss its markers
        cover.
    frame: a cause is a short phrase in the device library's words, "restart" or "lost packet".
        The count is in samples of one output port.
    verified-by: publish.a-marker-carries-the-cause-where-one-is-named
    verified-by: publish.a-marker-carries-the-count-where-one-is-given
*/
template <typename Block>
void emitOverflowTag(Block& blk, std::string_view cause = {}, std::optional<std::uint64_t> droppedSamples = std::nullopt) {
    auto tagMap = blk.out.makeTagMap();
    gr::tag::put(tagMap, gr::tag::RX_OVERFLOW, true);
    if (!cause.empty()) {
        gr::tag::put(tagMap, "rx_overflow_cause", std::string(cause));
    }
    if (droppedSamples.has_value()) {
        gr::tag::put(tagMap, gr::tag::N_DROPPED_SAMPLES, static_cast<gr::Size_t>(std::min<std::uint64_t>(*droppedSamples, std::numeric_limits<gr::Size_t>::max())));
    }
    publishOutputTag(blk, std::move(tagMap), 0UZ);
}

/*| contract: owe a gap marker to the first sample the next publish places, with the cause and
        the count emitOverflowTag takes. A marker already owed takes the later cause and adds
        this count to its own. It carries no count where either part has none.
    frame: the cause is a string of static storage, a literal, or null for none.
    why: a marker published ahead of a publish that places nothing falls on the sample the
        next marker falls on. A reader that merges one sample's tags keeps the later count
        alone.
    verified-by: publish.an-owed-marker-goes-out-once-with-every-count
*/
template <typename Block>
void oweOverflowTag(Block& blk, const char* cause, std::optional<std::uint64_t> droppedSamples) {
    auto& state = blk._outputState;
    if (state.gapOwed) {
        state.gapCount = state.gapCount.has_value() && droppedSamples.has_value() ? std::optional<std::uint64_t>(*state.gapCount + *droppedSamples) : std::nullopt;
    } else {
        state.gapCount = droppedSamples;
    }
    state.gapCause = cause;
    state.gapOwed  = true;
}

// Publish the owed gap marker, where one is owed, on the sample at the writer's position.
template <typename Block>
void placeOwedOverflowTag(Block& blk) {
    auto& state = blk._outputState;
    if (!state.gapOwed) {
        return;
    }
    state.gapOwed = false;
    emitOverflowTag(blk, state.gapCause != nullptr ? std::string_view(state.gapCause) : std::string_view{}, state.gapCount);
}

/*| contract: publish the tags the first sample of a block carries. The first block a stream
        publishes after its start carries gr:sample_rate, the value of the block's own
        sample_rate setting, and gr:frequency, the first entry of its frequency setting where
        that setting holds one. A block carries the periodic timing tag stamped tWallNs where
        tagDecision says one is due. Every source writes the rate the device runs into its
        sample_rate setting before it streams, a rate the device snapped included.
    contract: tWallNs is a time in nanoseconds or a callable returning one. A callable runs only
        where the block emits timing tags, and a block with no tag due builds no tag map.
    frame: the tags go on the sample at the writer's position, the first sample the next
        publish hands downstream, so a caller holding a reservation publishes them before it
        publishes the reservation.
    verified-by: publish.the-first-block-of-a-stream-carries-the-rate-in-force
    verified-by: publish.the-first-block-of-a-stream-carries-the-frequency-in-force
    verified-by: publish.a-block-with-no-tag-due-reads-no-clock
*/
template <typename Block, typename WallNs>
void publishBlockHeadTags(Block& blk, WallNs&& tWallNs) {
    const bool    rateOwed = blk._outputState.rateTagOwed.load(std::memory_order_relaxed) && blk._outputState.rateTagOwed.exchange(false, std::memory_order_acq_rel);
    bool          timing   = false;
    std::uint64_t tagNs    = 0;
    if (blk.emit_timing_tags) {
        if constexpr (std::invocable<WallNs>) {
            tagNs = tWallNs();
        } else {
            tagNs = tWallNs;
        }
        const auto decision = tagDecision(tagNs, blk._lastTagTimeNs, tagIntervalNs(blk.tag_interval.value));
        timing              = decision.emit;
        blk._lastTagTimeNs  = decision.lastTagNs;
    }
    if (!rateOwed && !timing) {
        return;
    }
    auto tagMap = blk.out.makeTagMap();
    if (rateOwed) {
        gr::tag::put(tagMap, gr::tag::SAMPLE_RATE, blk.sample_rate.value);
        if (!blk.frequency.value.empty()) {
            gr::tag::put(tagMap, gr::tag::FREQUENCY, blk.frequency.value.front());
        }
    }
    if (timing) {
        gr::tag::put(tagMap, gr::tag::TRIGGER_NAME, blk.trigger_name.value);
        gr::tag::put(tagMap, gr::tag::TRIGGER_TIME, tagNs);
        gr::tag::put(tagMap, gr::tag::TRIGGER_OFFSET, 0.f);
    }
    publishOutputTag(blk, std::move(tagMap), 0UZ);
}

/*| contract: how long a source waiting for room on its output edge waits when no block of the
        graph moves: the time the stream takes to fill half the edge at rateHz, and at most
        kRoomWaitLimitS. A rate that is not a positive number waits the limit.
    frame: a consume moves the graph's progress counter and ends the wait at once. The bound
        covers a reader whose consume moves no counter this source waits on.
    invariant: kRoomWaitLimitS bounds how late a stop or a staged setting reaches a source
        whose edge stays full.
    verified-by: publish.a-full-edge-waits-for-the-reader
*/
inline constexpr double kRoomWaitLimitS = 0.1;

inline std::chrono::nanoseconds roomWaitBound(std::size_t edgeSamples, double rateHz) {
    const double limitNs = kRoomWaitLimitS * 1e9;
    const double halfNs  = rateHz > 0.0 ? static_cast<double>(edgeSamples) * 0.5e9 / rateHz : limitNs; // a rate that is not a number fails the test
    return std::chrono::nanoseconds(static_cast<std::int64_t>(std::min(halfNs, limitNs)));
}

/*| contract: wait until the graph's progress counter leaves seen, or roomWaitBound of the edge
        at the block's rate passes, and answer whether the counter moved. The caller reads seen
        before it reads the room it lacks, so a consume that lands between the two ends the wait
        at once.
    frame: where the progress counter has no timed wait, the source reads the counter
        kRoomPollSteps times across the bound, and the wait ends at most one step after the
        consume.
*/
inline constexpr int kRoomPollSteps = 16;

template <typename Block>
bool awaitProgress(Block& blk, std::size_t seen, std::size_t edgeSamples) {
    const std::chrono::nanoseconds bound    = roomWaitBound(edgeSamples, static_cast<double>(blk.sample_rate.value));
    const auto                     deadline = std::chrono::steady_clock::now() + bound;
    if constexpr (requires { blk.progress->waitUntil(seen, deadline); }) {
        return blk.progress->waitUntil(seen, deadline);
    } else {
        const std::chrono::nanoseconds step = std::max(bound / kRoomPollSteps, std::chrono::nanoseconds(1));
        while (blk.progress->value() == seen) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::min<std::chrono::nanoseconds>(step, deadline - now));
        }
        return true;
    }
}

/*| role: publish one block of samples a source's streaming thread holds into its output edge,
        waiting for room as long as it takes.
    contract: no delivered sample is dropped. On a full edge the call waits through
        awaitProgress, applies the staged settings, and retries until the block is out or
        keepGoing() answers false. keepGoing is the caller's stop test. tWallNs is read once by
        the caller and stamps the whole block. The first sample of each block carries the owed
        gap marker and the tags publishBlockHeadTags places. The answer is the samples
        published, which falls short of the block only where keepGoing() answered false.
    invariant: staged settings are applied here, on the streaming thread, and never on the
        scheduler's. A device source holds this only while its work() override returns without
        reaching the base class's work path.
    trap: a request is capped at the room the writer has now. A reservation is all or nothing.
        A request longer than the edge is therefore never granted.
    invariant: the progress counter moves after the span is released, which is when the edge
        takes its samples.
    verified-by: publish.block-longer-than-the-edge
*/
template <typename Block, typename KeepGoing>
std::size_t holdAndPublish(Block& blk, std::span<const std::complex<float>> samples, std::uint64_t tWallNs, KeepGoing keepGoing) {
    auto&             writer = blk.out.streamWriter();
    const std::size_t n      = samples.size();
    std::size_t       done   = 0UZ;
    while (done < n && keepGoing()) {
        const std::size_t want = std::min(n - done, writer.available());
        if (want == 0UZ) {
            const std::size_t seen = blk.progress->value();
            if (writer.available() == 0UZ) {
                awaitProgress(blk, seen, writer.buffer().size());
            }
            blk.applyChangedSettings();
            continue;
        }
        std::size_t nCopy = 0UZ;
        {
            auto span = writer.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(want);
            if (!span.empty()) {
                nCopy = std::min(n - done, span.size());
                std::memcpy(span.data(), samples.data() + done, nCopy * sizeof(std::complex<float>));
                if (done == 0UZ) {
                    placeOwedOverflowTag(blk);
                    publishBlockHeadTags(blk, tWallNs);
                }
                span.publish(nCopy);
            }
        } // the edge takes the samples when the span is released
        if (nCopy == 0UZ) {
            awaitProgress(blk, blk.progress->value(), writer.buffer().size());
            blk.applyChangedSettings();
            continue;
        }
        done += nCopy;
        blk.progress->incrementAndGet();
        blk.progress->notify_all();
    }
    return done;
}

} // namespace capture
