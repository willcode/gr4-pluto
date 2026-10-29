/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Source.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <numbers>
#include <span>
#include <utility>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

/*| role: the host end of a hardware case: a sink that takes every sample, counts what arrived
        and adds up what it was worth.
    contract: one process runs one case, so these counters are one set per process. A case calls
        resetCounters() before it starts a graph and reads the counters after.
    frame: the numbers a case prints come from here — the sample count, the tags by kind, the
        wall clock at the first sample, and the mean power and mean sample over whatever window
        the case reset around.
    why: shared by every hardware binary so that a figure one driver's case prints means the same
        thing as the figure another one prints.
*/
namespace capture::test {

using HwComplex = std::complex<float>;
using CF32      = HwComplex;
using Clock     = std::chrono::steady_clock;

inline std::atomic<std::uint64_t> g_samples{0};
inline std::atomic<std::uint64_t> g_overflowTags{0};
inline std::atomic<std::uint64_t> g_eosTags{0};
inline std::atomic<std::uint64_t> g_timingTags{0};
inline std::atomic<std::uint64_t> g_rateTags{0};
inline std::atomic<std::uint64_t> g_firstTriggerNs{0};
inline std::atomic<std::uint64_t> g_firstSampleWallNs{0};
inline std::atomic<std::int64_t>  g_stallMs{0};
inline std::atomic<bool>          g_stallTaken{false};

/*| role: every timing tag that arrived, with the sample count the stream stood at when the
        block carrying it was handed over.
    frame: the tag sits at offset zero of its block, so that count is the tag's own sample
        offset in the stream.
*/
struct TagStamp {
    std::uint64_t atSample  = 0;
    std::uint64_t triggerNs = 0;
};
inline std::mutex            g_tagLock;
inline std::vector<TagStamp> g_tagStamps;

/*| role: a discontinuity marker with the sample offset it arrived at and the cause it named.
    frame: cause is empty for a marker carrying gr:rx_overflow alone, as a source that
        names no cause publishes.
*/
struct OverflowStamp {
    std::uint64_t atSample = 0;
    std::string   cause;
};
inline std::vector<OverflowStamp> g_overflowStamps;

/*| role: the sample rate each rate tag carried, with the offset it arrived at.
    why: a rate change puts the new rate at the current write position, so a case checking that
        no old-rate samples follow the tag needs the offset as well as the value.
*/
struct RateStamp {
    std::uint64_t atSample = 0;
    double        rateHz   = 0.0;
};
inline std::vector<RateStamp> g_rateStamps;

/*| role: what the arriving samples add up to, over whatever window a case resets around.
    frame: the mean power and the mean sample, the second being the DC component where an
        oscillator leakage spike sits.
    trap: a mutex rather than atomics, since a complex accumulator is two numbers that have
        to move together.
*/
struct Accumulator {
    std::mutex           lock;
    double               power = 0.0;
    std::complex<double> dc{};
    std::uint64_t        n = 0;

    void reset() {
        std::lock_guard g(lock);
        power = 0.0;
        dc    = {};
        n     = 0;
    }
    void add(std::span<const HwComplex> v) {
        std::lock_guard g(lock);
        for (const HwComplex& c : v) {
            const std::complex<double> s(c.real(), c.imag());
            power += s.real() * s.real() + s.imag() * s.imag();
            dc += s;
        }
        n += v.size();
    }
};
inline Accumulator g_accum;

/*| role: the energy at one offset above the center and at the same offset below it.
    contract: a case calls resetBins(offsetHz, rateHz, groupSamples) and reads binsDb() after a
        window. The answer is the mean power of the stream mixed down by that offset and by its
        negative, each averaged over groupSamples first, so the measurement covers a band about
        rate/groupSamples wide around each of the two places. The group sets that width:
        2048 samples at 2 MS/s is about a kilohertz, and 32 is about sixty.
    frame: one unit phasor steps per sample and the two mixes are that phasor and its conjugate,
        so the cost is two complex multiplies a sample. The phasor is reseeded at every group
        from an absolute sample index, which keeps the phase exact at group boundaries.
    why: which side of center a known signal lands on is the only way to tell a receiver that
        inverts its spectrum in a band from one that does not, and a level alone cannot say it.
*/
inline constexpr std::size_t kBinGroupDefault = 2048;

inline std::atomic<bool>    g_binOn{false};
inline std::mutex           g_binLock;
inline std::size_t          g_binGroup      = kBinGroupDefault;
inline double               g_binOffsetHz   = 0.0;
inline double               g_binRateHz     = 0.0;
inline double               g_binPowerPlus  = 0.0;
inline double               g_binPowerMinus = 0.0;
inline std::uint64_t        g_binGroups     = 0;
inline std::complex<double> g_binAccPlus{};
inline std::complex<double> g_binAccMinus{};
inline std::size_t          g_binFill = 0;

inline void resetBins(double offsetHz, double rateHz, std::size_t groupSamples = kBinGroupDefault) {
    std::lock_guard g(g_binLock);
    g_binGroup      = std::max<std::size_t>(groupSamples, 1UZ);
    g_binOffsetHz   = offsetHz;
    g_binRateHz     = rateHz;
    g_binPowerPlus  = 0.0;
    g_binPowerMinus = 0.0;
    g_binGroups     = 0;
    g_binAccPlus    = {};
    g_binAccMinus   = {};
    g_binFill       = 0;
    g_binOn.store(offsetHz != 0.0 && rateHz > 0.0, std::memory_order_release);
}

// The two mean powers in dB below full scale: the one above center first.
inline std::pair<double, double> binsDb() {
    std::lock_guard g(g_binLock);
    if (g_binGroups == 0UL) {
        return {-300.0, -300.0};
    }
    const double n = static_cast<double>(g_binGroups);
    return {10.0 * std::log10(g_binPowerPlus / n + 1e-30), 10.0 * std::log10(g_binPowerMinus / n + 1e-30)};
}

inline void addToBins(std::span<const HwComplex> v) {
    std::lock_guard g(g_binLock);
    if (g_binOffsetHz == 0.0 || g_binRateHz <= 0.0) {
        return;
    }
    // One unit phasor per sample, its phase taken from the position inside the group, so
    // each group is an independent pair of bins and the two powers add across groups.
    const std::complex<double> step = std::polar(1.0, -2.0 * std::numbers::pi * g_binOffsetHz / g_binRateHz);
    std::complex<double>       p    = std::polar(1.0, -2.0 * std::numbers::pi * g_binOffsetHz / g_binRateHz * static_cast<double>(g_binFill));
    for (const HwComplex& c : v) {
        const std::complex<double> s(c.real(), c.imag());
        g_binAccPlus += s * p;
        g_binAccMinus += s * std::conj(p);
        p *= step;
        if (++g_binFill == g_binGroup) {
            const double k = static_cast<double>(g_binGroup);
            g_binPowerPlus += std::norm(g_binAccPlus / k);
            g_binPowerMinus += std::norm(g_binAccMinus / k);
            ++g_binGroups;
            g_binAccPlus  = {};
            g_binAccMinus = {};
            g_binFill     = 0UZ;
            p             = {1.0, 0.0};
        }
    }
}

inline void resetCounters() {
    g_samples.store(0);
    g_overflowTags.store(0);
    g_eosTags.store(0);
    g_timingTags.store(0);
    g_rateTags.store(0);
    g_firstTriggerNs.store(0);
    g_firstSampleWallNs.store(0);
    g_stallMs.store(0);
    g_stallTaken.store(false);
    g_accum.reset();
    resetBins(0.0, 0.0);
    std::lock_guard g(g_tagLock);
    g_tagStamps.clear();
    g_overflowStamps.clear();
    g_rateStamps.clear();
}

inline void resetPower() { g_accum.reset(); }

// Mean power of what has arrived since the last reset, in dB below full scale.
inline double meanDbfs() {
    std::lock_guard g(g_accum.lock);
    return g_accum.n == 0UL ? -300.0 : 10.0 * std::log10(g_accum.power / static_cast<double>(g_accum.n) + 1e-30);
}

// The DC component of what has arrived, which is where an oscillator leakage spike sits.
inline double dcDbfs() {
    std::lock_guard g(g_accum.lock);
    return g_accum.n == 0UL ? -300.0 : 20.0 * std::log10(std::abs(g_accum.dc) / static_cast<double>(g_accum.n) + 1e-30);
}

/*| contract: the power in what arrived with its own mean taken out, in dB below full scale,
        which on a quiet port is the receiver's noise floor.
    trap: the mean power of a direct-conversion stream is not the noise floor. The frontend
        leaves a standing offset at zero frequency that can sit tens of dB above the noise, and
        only the baseband element amplifies it, so a mean-power reading can hold still while a
        front-end element moves. The variance is the reading that follows every element.
*/
inline double acDbfs() {
    std::lock_guard g(g_accum.lock);
    if (g_accum.n == 0UL) {
        return -300.0;
    }
    const double               n  = static_cast<double>(g_accum.n);
    const std::complex<double> mu = g_accum.dc / n;
    return 10.0 * std::log10(std::max(g_accum.power / n - std::norm(mu), 0.0) + 1e-30);
}

/*| contract: count one tag by kind, and record what a discontinuity, a timing tag or a rate tag
        carried.
    frame: each kind is read under its gr: key. The cause beside a marker is
        rx_overflow_cause.
*/
inline void countTag(const gr::property_map& map) {
    // One pass gathers the keys, because a property map is unordered: the marker and the
    // cause it carries sit in the same map, and reading them in arrival order put a cause
    // on the marker before it.
    bool                   overflow = false;
    bool                   eos      = false;
    bool                   haveRate = false;
    double                 rateHz   = 0.0;
    bool                   haveTime = false;
    std::uint64_t          timeNs   = 0;
    std::string_view       cause;

    for (const auto& [key, value] : map) {
        const std::string_view k(key.data(), key.size());
        if (k == gr::tag::RX_OVERFLOW.key()) {
            overflow = true;
        } else if (k == "rx_overflow_cause") {
            cause = value.value_or(std::string_view{});
        } else if (k == gr::tag::END_OF_STREAM.key()) {
            eos = true;
        } else if (k == gr::tag::SAMPLE_RATE.key()) {
            haveRate = true;
            rateHz   = value.value_or(0.0);
        } else if (k == gr::tag::TRIGGER_TIME.key()) {
            if (const std::uint64_t* t = value.get_if<std::uint64_t>(); t != nullptr) {
                haveTime = true;
                timeNs   = *t;
            }
        }
    }

    const std::uint64_t at = g_samples.load(std::memory_order_relaxed);
    if (eos) {
        g_eosTags.fetch_add(1, std::memory_order_relaxed);
    }
    if (overflow) {
        g_overflowTags.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard g(g_tagLock);
        g_overflowStamps.push_back({.atSample = at, .cause = std::string(cause)});
    }
    if (haveRate) {
        g_rateTags.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard g(g_tagLock);
        g_rateStamps.push_back({.atSample = at, .rateHz = rateHz});
    }
    if (haveTime) {
        std::uint64_t none = 0UL;
        g_firstTriggerNs.compare_exchange_strong(none, timeNs);
        g_timingTags.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard g(g_tagLock);
        g_tagStamps.push_back({.atSample = at, .triggerNs = timeNs});
    }
}

/*| role: the host, as a block: it takes everything and counts what it took.
    trap: the one-shot stall sleeps on the scheduler thread, which is the thread the source's
        work() runs on, so the source's publish blocks for exactly as long as a host that has
        stopped draining would block it.
*/
struct hw_sink : gr::Block<hw_sink> {
    gr::PortIn<HwComplex> in;
    GR_MAKE_REFLECTABLE(hw_sink, in);

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            countTag(tag.map);
        }
        const std::size_t n = inSpan.size();
        if (n == 0UZ) {
            return gr::work::Status::OK;
        }
        if (g_samples.load(std::memory_order_relaxed) == 0UL) {
            g_firstSampleWallNs.store(capture::wallClockNs(), std::memory_order_relaxed);
        }
        g_accum.add(std::span<const HwComplex>(inSpan.data(), n));
        if (g_binOn.load(std::memory_order_acquire)) {
            addToBins(std::span<const HwComplex>(inSpan.data(), n));
        }
        g_samples.fetch_add(n, std::memory_order_relaxed);

        if (const std::int64_t ms = g_stallMs.load(std::memory_order_relaxed); ms > 0 && !g_stallTaken.exchange(true)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
        return gr::work::Status::OK;
    }
};

/*| contract: how long the delivered rate takes to come back inside tol of wantHz, in seconds,
        and the rate of the window that came back. seconds stays negative when no window inside
        limitMs came back.
    why: a host that stopped draining leaves the ring holding a backlog, and a window opened
        while that backlog is still moving reads above the device's own rate. The guarantee is
        that the rate comes back once the ring has drained, so the reading is the time it took.
    frame: consecutive windows of windowMs, each counted from the sink's own sample total, so a
        case states seconds of draining rather than assuming a settling time.
*/
struct RateClosure {
    double seconds = -1.0;
    double rateHz  = 0.0;
};

inline RateClosure rateClosure(double wantHz, double tol, int windowMs, int limitMs) {
    const auto  start = std::chrono::steady_clock::now();
    RateClosure out;
    while (std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() < static_cast<double>(limitMs)) {
        const std::uint64_t at0 = g_samples.load(std::memory_order_relaxed);
        const auto          t0  = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(windowMs));
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        out.rateHz           = seconds > 0.0 ? static_cast<double>(g_samples.load(std::memory_order_relaxed) - at0) / seconds : 0.0;
        if (std::abs(out.rateHz - wantHz) <= tol * wantHz) {
            out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            return out;
        }
    }
    return out;
}

inline bool waitForSamples(std::chrono::milliseconds limit) {
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline) {
        if (g_samples.load(std::memory_order_relaxed) > 0UL) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

inline void settle(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Mean power over one window, in dB below full scale.
inline double dbfsOver(double seconds) {
    resetPower();
    settle(static_cast<int>(seconds * 1000.0));
    return meanDbfs();
}

// The noise floor and the offset sitting on it, over one window.
inline std::pair<double, double> floorOver(double seconds) {
    resetPower();
    settle(static_cast<int>(seconds * 1000.0));
    return {acDbfs(), dcDbfs()};
}

/*| contract: read one value off the open device through the block's own handle, under the
        block's own control mutex, so the read cannot race a teardown that resets the handle.
    why: what the device holds and what the block was asked for are different numbers, and a
        guarantee about the first is only testable against the first.
*/
template <typename Src, typename R, typename Fn>
R deviceRead(Src& src, R fallback, Fn&& fn) {
    std::lock_guard lock(src._ctrlMutex);
    if (src._dev == nullptr) {
        return fallback;
    }
    return fn(src._dev);
}

/*| contract: the accessors a family may or may not carry, answered as a sentinel where it does
        not. -1 means the block states no such number at all, which is a different answer from
        zero.
    why: the seam a receiver binds detects each of these the same way, so a case that asks for
        one through the same detection measures what a receiver would get.
*/
template <typename Src>
long long droppedOf(Src& s) {
    if constexpr (requires { s.droppedSamples(); }) {
        return static_cast<long long>(s.droppedSamples());
    } else {
        return -1;
    }
}

template <typename Src>
double tunedOf(Src& s) {
    if constexpr (requires { s.tunedRfHz(); }) {
        return s.tunedRfHz();
    } else {
        return -1.0;
    }
}

template <typename Src>
double bandwidthOf(Src& s) {
    if constexpr (requires { s.analogBandwidthHz(); }) {
        return s.analogBandwidthHz();
    } else {
        return -1.0;
    }
}

// 1 when the sweep answered, 0 when it refused, -1 when the block publishes no sensors.
template <typename Src>
int sensorsOf(Src& s, std::vector<capture::SensorReading>& out) {
    if constexpr (requires { s.readSensors(out); }) {
        return s.readSensors(out) ? 1 : 0;
    } else {
        return -1;
    }
}

inline std::vector<std::string> ownThreadNames() {
    std::vector<std::string> names;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) {
        std::ifstream comm(entry.path() / "comm");
        std::string   name;
        if (std::getline(comm, name)) {
            names.push_back(name);
        }
    }
    return names;
}

/*| frame: where the converter's own resolution takes over from the analog path. Both radios
        these cases drive carry an eight-bit converter, so a setting whose analog output falls
        below one code reads a variance set by where the standing offset sits between two codes
        rather than by the gain in front of it.
    measured: readings cluster between -48 and -52 dBFS wherever the path is below one code, and
        repeat to about 0.03 dB, so -46 separates the two regimes with room either side.
*/
inline constexpr double kConverterFloorDbfs = -46.0;

/*| contract: non-decreasing within a tolerance among the readings that are above the converter's
        own resolution, which is where a reading says anything about the element that produced
        it. A reading at or below that floor is skipped rather than compared.
*/
inline bool risesAboveFloor(const std::vector<double>& v, double tolDb) {
    double last = -1.0e9;
    for (const double x : v) {
        if (x <= kConverterFloorDbfs) {
            continue;
        }
        if (x < last - tolDb) {
            return false;
        }
        last = x;
    }
    return true;
}

inline std::size_t countAboveFloor(const std::vector<double>& v) {
    return static_cast<std::size_t>(std::ranges::count_if(v, [](double x) { return x > kConverterFloorDbfs; }));
}

/*| contract: run fn count times and answer the median and the worst, in microseconds.
    why: what a sensor read costs is a property of the unit and not of the model, so it is
        measured rather than assumed. The median says what a repeating poll pays and the worst
        says what the poll's own window has to survive.
*/
template <typename Fn>
std::pair<double, double> timeCall(int count, Fn&& fn) {
    std::vector<double> us;
    us.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const auto t0 = Clock::now();
        fn();
        us.push_back(std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
    }
    std::ranges::sort(us);
    return {us[us.size() / 2UZ], us.back()};
}

/*| contract: put this thread on one named processor, and say whether it went there.
    why: this box is an i7-12700H. CPUs 0 to 11 are performance cores and 12 to 19 are efficiency
        cores at about two thirds of the speed, so an unpinned timing lands wherever the
        scheduler puts it and two arms of one comparison can land on different kinds of core.
*/
inline bool pinToCpu(int cpu) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<std::size_t>(cpu), &set);
    return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
}

/*| contract: count the end-of-stream markers sitting in a stopped sink's own tag ring, added to
        whatever the stream already reported.
    trap: a consumer never sees the end-of-stream marker through its input span. The marker sits
        at the index the last sample ended on, so the scheduler finds zero samples ahead of it
        and ends the block without one more call. The consumer's own tag ring is where the marker
        can be counted.
*/
inline void countEndOfStreamTags(hw_sink& sink) {
    auto tags   = sink.in.tagReader().get();
    std::ignore = tags.consume(0UZ);
    for (const gr::Tag& tag : tags) {
        for (const auto& [key, value] : tag.map) {
            const std::string_view k(key.data(), key.size());
            if (k == gr::tag::END_OF_STREAM.key() && value.get_if<bool>() != nullptr) {
                g_eosTags.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
}

} // namespace capture::test
