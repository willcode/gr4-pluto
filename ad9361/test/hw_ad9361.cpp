/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the AD9361 source's guarantees, measured against an attached radio.
    contract: one case per guarantee, selected by name on the command line like every other
        binary here, and every case prints the numbers it measured. A case opens the radio
        itself and releases it before it returns, so one process runs one case.
    frame: the graph is the source and a counting sink on the scheduler a receiver uses, so
        start, the reader thread, the decoder, the ring, the publish, the tags, stop and the
        teardown guard run as a receiver runs them.
    trap: these cases open a radio and are registered under the label hardware, built only
        under CAPTURE_HW_AD9361, so the hardware-free suite never reaches one. The selector
        comes from CAPTURE_HW_AD9361_ARGS, uri=usb: by default, and names the image the radio
        runs.
    invariant: the transmit cases send a tone at the attenuation CAPTURE_HW_AD9361_TX_GAIN
        names, -30 dB by default, and a receive graph on the same radio hears it through the
        radio's own leakage between its ports. Nothing is transmitted at full power.
    trap: an ADALM-Pluto on USB 2 sustains a few megasamples per second. A rate above that
        overruns the link, which a case reports and does not fail.
*/
#include <capture/ad9361/Sink.hpp>
#include <capture/ad9361/Source.hpp>

#include <gnuradio-4.0/Runtime.hpp>

#include "support/Cases.hpp"
#include "support/Chain.hpp"
#include "support/HwSink.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using namespace capture::test;
using capture::ad9361::Source;

const std::string& deviceArgs() {
    static const std::string args = [] {
        const char* v = std::getenv("CAPTURE_HW_AD9361_ARGS");
        return v != nullptr && *v != '\0' ? std::string(v) : std::string("uri=usb:");
    }();
    return args;
}

gr::property_map sourceSettings(double rateHz, double freqHz, double gainDb) {
    return {
        {"name", std::string("src")},
        {"device", std::string("ad9361")},
        {"device_parameter", deviceArgs()},
        {"sample_rate", rateHz},
        {"frequency", std::vector<double>{freqHz}},
        {"rx_gains", std::vector<double>{gainDb}},
        {"tag_interval", 0.5f},
    };
}

template <typename Body>
bool withGraph(gr::property_map settings, Body&& body) {
    resetCounters();
    auto flow = capture::test::chain<Source, hw_sink>(std::move(settings), {{"name", std::string("host")}});
    if (!flow) {
        std::fprintf(stderr, "the source does not wire to the sink: %s\n", flow.error().message.c_str());
        return false;
    }
    auto& src     = *flow->up;
    auto& sink    = *flow->down;
    auto  runtime = gr::Runtime::create(std::move(flow->graph), "gr::scheduler::Simple<singleThreadedBlocking>");
    if (!runtime.has_value()) {
        std::fprintf(stderr, "the graph did not build\n");
        return false;
    }
    if (const auto started = capture::test::startRun(*runtime); !started.accepted) {
        std::fprintf(stderr, "the graph did not start: %s\n", started.reason.c_str());
        return false;
    }
    body(src, *runtime);
    runtime->stop();
    countEndOfStreamTags(sink);
    return true;
}

/*| contract: true where a receive buffer can be created on the radio, which after a stop
        means the block released its own. iiod gives a device's buffer to one client at a time.
*/
bool radioReleased() {
    std::string                 error;
    const auto                  ctx = capture::ad9361::openContext(capture::ad9361::contextUriFor(deviceArgs()), error);
    iio_device*                 rx  = ctx ? iio_context_find_device(ctx.get(), capture::ad9361::kRxName) : nullptr;
    if (rx == nullptr) {
        std::fprintf(stderr, "reopen failed: %s\n", error.c_str());
        return false;
    }
    iio_channel* i = iio_device_find_channel(rx, "voltage0", false);
    iio_channel* q = iio_device_find_channel(rx, "voltage1", false);
    if (i == nullptr || q == nullptr) {
        return false;
    }
    int         code        = 0;
    iio_channel* channels[] = {i, q};
    const auto  stream      = capture::ad9361::iio::Stream::open(rx, channels, 4096, false, code);
    return stream && stream->refill() > 0;
}

double txGainDb() {
    const char* v = std::getenv("CAPTURE_HW_AD9361_TX_GAIN");
    return v != nullptr && *v != '\0' ? std::strtod(v, nullptr) : -30.0;
}

/*| role: the graph end of a transmit case: a complex tone at a fixed offset from the center,
        for burst_samples samples with gr:tx_eob = true on the last, or for ever where that is 0.
*/
struct hw_burst_tone : gr::Block<hw_burst_tone> {
    gr::PortOut<CF32> out;

    gr::Annotated<double, "cycles_per_sample", gr::Doc<"tone offset in cycles per sample">> cycles_per_sample = 0.1;
    gr::Annotated<float, "amplitude", gr::Doc<"peak amplitude of each part">>               amplitude         = 0.5f;
    gr::Annotated<gr::Size_t, "burst_samples", gr::Doc<"samples before the burst end; 0 sends the tone continuously">> burst_samples = 0U;

    GR_MAKE_REFLECTABLE(hw_burst_tone, out, cycles_per_sample, amplitude, burst_samples);

    std::uint64_t _n = 0;

    gr::work::Status processBulk(gr::OutputSpanLike auto& output) {
        std::size_t n = output.size();
        if (burst_samples.value > 0U) {
            if (_n >= burst_samples.value) {
                output.publish(0UZ);
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // the burst is over; the graph stays up
                return gr::work::Status::OK;
            }
            n = std::min<std::size_t>(n, burst_samples.value - _n);
        }
        for (std::size_t i = 0; i < n; ++i) {
            const double phase = 2.0 * std::numbers::pi * cycles_per_sample * static_cast<double>(_n + i);
            output[i]          = {static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase))};
        }
        if (burst_samples.value > 0U && n > 0UZ && _n + n == burst_samples.value) {
            output.publishTag(gr::property_map{{std::pmr::string(gr::tag::TX_EOB.key()), true}}, n - 1UZ);
        }
        output.publish(n);
        _n += n;
        return gr::work::Status::OK;
    }
};

gr::property_map sinkSettings(double rateHz, double freqHz) {
    return {
        {"name", std::string("tx")},
        {"device", std::string("ad9361")},
        {"device_parameter", deviceArgs()},
        {"sample_rate", rateHz},
        {"frequency", std::vector<double>{freqHz}},
        {"tx_gains", std::vector<double>{txGainDb()}},
    };
}

template <typename Body>
bool withTxGraph(gr::property_map settings, gr::Size_t burstSamples, Body&& body) {
    auto flow = capture::test::chain<hw_burst_tone, capture::ad9361::Sink>({{"name", std::string("tone")}, {"burst_samples", burstSamples}}, std::move(settings));
    if (!flow) {
        std::fprintf(stderr, "the tone does not wire to the sink: %s\n", flow.error().message.c_str());
        return false;
    }
    auto& sink    = *flow->down;
    auto  runtime = gr::Runtime::create(std::move(flow->graph), "gr::scheduler::Simple<singleThreadedBlocking>");
    if (!runtime.has_value()) {
        std::fprintf(stderr, "the transmit graph did not build\n");
        return false;
    }
    if (const auto started = capture::test::startRun(*runtime); !started.accepted) {
        std::fprintf(stderr, "the transmit graph did not start: %s\n", started.reason.c_str());
        return false;
    }
    body(sink, *runtime);
    runtime->stop();
    return true;
}

// The power above and below center at the tone's offset, over a window, in dBFS.
std::pair<double, double> toneOver(double offsetHz, double rateHz, int windowMs) {
    resetBins(offsetHz, rateHz);
    settle(windowMs);
    return binsDb();
}

double achievedRate(int windowMs) {
    const std::uint64_t at0 = g_samples.load();
    const auto          t0  = Clock::now();
    settle(windowMs);
    return static_cast<double>(g_samples.load() - at0) / std::chrono::duration<double>(Clock::now() - t0).count();
}

} // namespace

int main(int argc, char** argv) {
    using namespace boost::ut;
    capture::test::Cases cases(argc, argv);

    cases("ad9361.open-stream-close", [] {
        constexpr double kRate = 2.5e6;
        double           rate  = 0.0;
        double           levelDbfs = 0.0;
        double           setRate   = 0.0;
        const bool       ran   = withGraph(sourceSettings(kRate, 915.0e6, 30.0), [&](Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(1000);
            resetPower();
            rate      = achievedRate(2000);
            levelDbfs = meanDbfs();
            setRate   = src.sample_rate.value;
        });
        expect(ran) << "the graph ran";
        std::printf("open-stream-close: %.1f S/s achieved (%.3f %% off %.0f), the setting reads %.1f, %.2f dBFS, timing tags %llu, overflow tags %llu, eos tags %llu\n", rate,
                    100.0 * (rate - kRate) / kRate, kRate, setRate, levelDbfs, static_cast<unsigned long long>(g_timingTags.load()), static_cast<unsigned long long>(g_overflowTags.load()),
                    static_cast<unsigned long long>(g_eosTags.load()));
        expect(std::abs(rate - kRate) <= 0.02 * kRate) << "the delivered rate is within 2 percent of the requested one";
        expect(std::abs(setRate - kRate) <= 1.0) << "the setting states the rate the radio read back";
        expect(levelDbfs < 0.0 && levelDbfs > -120.0) << "the samples are scaled into full scale and carry something";
        expect(g_rateTags.load() >= 1UL && g_timingTags.load() >= 1UL) << "the rate tag and a timing tag";
        expect(g_overflowTags.load() == 0UL) << "a stream at a rate the link carries starts with no gap";
        expect(g_eosTags.load() == 1UL) << "exactly one end-of-stream tag after stop";
        expect(radioReleased()) << "the radio's receive buffer was released";
    });

    cases("ad9361.rate-ladder", [] {
        for (const double want : Source::sampleRates()) {
            double     got     = 0.0;
            double     setting = 0.0;
            const bool ran     = withGraph(sourceSettings(want, 915.0e6, 30.0), [&](Source& src, auto&) {
                if (waitForSamples(std::chrono::milliseconds(6000))) {
                    settle(300);
                    got     = achievedRate(1000);
                    setting = src.sample_rate.value;
                }
            });
            std::printf("rate-ladder: %.0f asked, the radio reads back %.1f, %.1f S/s achieved (%.2f %%), overflow tags %llu\n", want, setting, got, 100.0 * (got - want) / want,
                        static_cast<unsigned long long>(g_overflowTags.load()));
            expect(ran && got > 0.0) << want << " streams";
            expect(std::abs(setting - want) <= 1.0) << want << " is the rate the radio reads back";
        }
    });

    cases("ad9361.rate-change-in-stream", [] {
        /*| frame: 1 MS/s is under 25e6/12 S/s, so the change loads a decimating FIR through
                libad9361 while the buffer streams.
        */
        constexpr double kFrom = 2.5e6;
        constexpr double kTo   = capture::ad9361::kHaveAd9361 ? 1.0e6 : 3.0e6;
        RateClosure      closed;
        const bool       ran = withGraph(sourceSettings(kFrom, 915.0e6, 30.0), [&](Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(500);
            std::ignore = src.settings().setStaged({{"sample_rate", kTo}});
            closed      = rateClosure(kTo, 0.02, 500, 6000);
        });
        expect(ran);
        bool rateChangeMarked = false;
        bool newRateTagged    = false;
        {
            std::lock_guard g(g_tagLock);
            rateChangeMarked = std::ranges::any_of(g_overflowStamps, [](const auto& s) { return s.cause == "rate change"; });
            newRateTagged    = std::ranges::any_of(g_rateStamps, [](const auto& s) { return std::abs(s.rateHz - kTo) <= 1.0; });
        }
        std::printf("rate-change-in-stream: %.0f to %.0f, the stream reached %.1f S/s after %.2f s\n", kFrom, kTo, closed.rateHz, closed.seconds);
        expect(closed.seconds >= 0.0) << "the stream reaches the new rate";
        expect(rateChangeMarked) << "the gap carries the cause rate change";
        expect(newRateTagged) << "a rate tag states the new rate as the radio read it back";
    });

    cases("ad9361.tuning-and-range", [] {
        double held = 0.0;
        double low  = 0.0;
        const bool ran = withGraph(sourceSettings(2.5e6, 433.92e6, 30.0), [&](Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            const auto ranges = Source::frequencyRanges();
            low               = ranges.front().first;
            std::ignore       = src.settings().setStaged({{"frequency", std::vector<double>{915.0e6}}});
            settle(400);
            std::ignore = src.settings().setStaged({{"frequency", std::vector<double>{low / 2.0}}});
            settle(400);
            held = src.frequency->empty() ? 0.0 : src.frequency->front();
        });
        std::printf("tuning-and-range: the radio states coverage from %.0f Hz; a request at half of it leaves the setting at %.0f\n", low, held);
        expect(ran);
        expect(held == 915.0e6) << "a request outside the coverage leaves the LO and the setting where they were";
    });

    cases("ad9361.gain-and-agc", [] {
        double lowDb  = 0.0;
        double highDb = 0.0;
        double backDb = 0.0;
        const bool ran = withGraph(sourceSettings(2.5e6, 915.0e6, 10.0), [&](Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(500);
            resetPower();
            settle(500);
            lowDb       = meanDbfs();
            std::ignore = src.settings().setStaged({{"rx_gains", std::vector<double>{60.0}}});
            settle(500);
            resetPower();
            settle(500);
            highDb      = meanDbfs();
            std::ignore = src.settings().setStaged({{"gain_mode", true}});
            settle(800);
            std::ignore = src.settings().setStaged({{"gain_mode", false}});
            settle(500);
            resetPower();
            settle(500);
            backDb = meanDbfs();
        });
        std::printf("gain-and-agc: %.2f dBFS at 10 dB, %.2f dBFS at 60 dB, %.2f dBFS after the AGC hands 60 dB back\n", lowDb, highDb, backDb);
        expect(ran);
        expect(highDb > lowDb + 6.0) << "50 dB more gain raises the level";
        expect(std::abs(backDb - highDb) < 3.0) << "leaving the AGC writes the held gain again";
    });

    cases("ad9361.teardown-and-restart", [] {
        std::uint64_t first = 0;
        std::uint64_t second = 0;
        const bool    a     = withGraph(sourceSettings(2.5e6, 915.0e6, 30.0), [&](Source&, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000)));
            settle(500);
            first = g_samples.load();
        });
        const bool b = withGraph(sourceSettings(2.5e6, 915.0e6, 30.0), [&](Source&, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000)));
            settle(500);
            second = g_samples.load();
        });
        std::printf("teardown-and-restart: %llu then %llu samples\n", static_cast<unsigned long long>(first), static_cast<unsigned long long>(second));
        expect(a && b && first > 0U && second > 0U) << "the second graph opens the radio the first released";
        expect(radioReleased());
    });

    cases("ad9361.every-spelling-shares-one-context", [] {
        /*| frame: the radio's spellings are "usb:", each full URI the USB scan lists, and each URI
                the family's listing gives, the network ones among them. A second USB context in
                one process is refused, so a spelling that missed the shared context fails here.
        */
        using capture::ad9361::acquireContext;
        std::vector<std::string> spellings{"usb:"};
        for (const auto& uri : capture::ad9361::usbContextUris()) {
            spellings.push_back(uri);
        }
        std::string error;
        auto        first = acquireContext("usb:", 2000U, error);
        expect(fatal(first != nullptr)) << "usb: opens: " << error;
        for (const auto& [label, device] : capture::ad9361::listRadios()) {
            spellings.push_back(capture::kwargsValue(device, "uri"));
        }
        bool all = true;
        for (const auto& uri : spellings) {
            const auto ctx  = acquireContext(uri, 2000U, error);
            const bool same = ctx != nullptr && ctx.get() == first.get();
            std::printf("every-spelling-shares-one-context: %-24s %s\n", uri.c_str(), same ? "the same context" : (ctx ? "another context" : error.c_str()));
            all = all && same;
        }
        expect(spellings.size() >= 3UZ) << "usb:, the scanned URI and at least one listed URI";
        expect(all) << "every spelling reaches the one context";

        // A source streaming on "usb:" while the listing asks for the full URIs.
        first.reset();
        std::size_t listed = 0;
        const bool  ran    = withGraph(sourceSettings(2.5e6, 915.0e6, 30.0), [&](Source&, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            listed = capture::ad9361::listRadios().size();
            settle(300);
        });
        std::printf("every-spelling-shares-one-context: %zu listings while the source streamed on %s\n", listed, deviceArgs().c_str());
        expect(ran && listed >= 1UZ) << "the listing finds the radio the source streams from";
        expect(g_overflowTags.load() == 0UL) << "and the stream has no gap from it";
    });

    cases("ad9361.a-sink-start-keeps-the-source-whole", [] {
        /*| frame: a source streams the radio at 2.5 MS/s. A sink asks for another rate, then for
                a frequency under the coverage; each start fails, and the source's stream keeps its
                rate. The refused sink is reset and started again, and is refused again. The
                source's gaps across the starts are printed, not judged.
        */
        constexpr double kRate = 2.5e6;
        const auto startSink = [](double rateHz, double freqHz) {
            capture::ad9361::Sink blk{};
            blk.device_parameter = deviceArgs();
            blk.sample_rate      = rateHz;
            blk.frequency        = std::vector<double>{freqHz};
            blk.tx_gains         = std::vector<double>{txGainDb()};
            std::ignore          = blk.changeStateTo(gr::lifecycle::State::INITIALISED);
            const auto started   = blk.changeStateTo(gr::lifecycle::State::RUNNING);
            const auto why       = [](const auto& r) { return r.has_value() ? std::string("the start went through") : std::string(r.error().message); };
            if (started.has_value()) {
                return std::pair{why(started), why(started)};
            }
            expect(!blk.frequency->empty() && blk.frequency->front() == freqHz) << "a refused first tune leaves the setting as asked";
            std::ignore      = blk.changeStateTo(gr::lifecycle::State::INITIALISED);
            const auto again = blk.changeStateTo(gr::lifecycle::State::RUNNING);
            return std::pair{why(started), why(again)};
        };
        std::string   rateRefusal;
        std::string   tuneRefusal;
        std::string   tuneAgain;
        double        rate    = 0.0;
        double        setting = 0.0;
        std::uint64_t gaps    = 0;
        const bool    ran     = withGraph(sourceSettings(kRate, 915.0e6, 30.0), [&](Source& src, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the stream starts";
            settle(300);
            const std::uint64_t before = g_overflowTags.load();
            rateRefusal                = startSink(5.0e6, 915.0e6).first;
            std::tie(tuneRefusal, tuneAgain) = startSink(kRate, Source::frequencyRanges().front().first / 2.0);
            settle(300);
            gaps    = g_overflowTags.load() - before;
            rate    = achievedRate(1000);
            setting = src.sample_rate.value;
        });
        std::printf("a-sink-start-keeps-the-source-whole: the source streams %.1f S/s, its setting reads %.1f, %llu gaps across the starts\n  rate: %s\n  tune: %s\n  again: %s\n", rate, setting,
                    static_cast<unsigned long long>(gaps), rateRefusal.c_str(), tuneRefusal.c_str(), tuneAgain.c_str());
        expect(ran);
        expect(rateRefusal.contains("sample_rate asked 5000000 S/s, and another block in this process streams this radio at")) << "a second rate on the radio fails the sink's start";
        expect(tuneRefusal.ends_with("Hz is outside the radio's coverage; the LO stays where it is")) << "a frequency outside the coverage fails the sink's start";
        expect(tuneAgain.ends_with("Hz is outside the radio's coverage; the LO stays where it is")) << "and a reset and second start of the same sink are refused again";
        expect(std::abs(rate - kRate) <= 0.02 * kRate && std::abs(setting - kRate) <= 1.0) << "the source's stream and setting keep their rate";
    });

    cases("ad9361.a-refused-source-start-is-refused-again", [] {
        /*| frame: a source asks for a frequency under the coverage. Its start fails with the
                setting as asked, and a reset and second start of the same block fail the same way.
        */
        const double freqHz = Source::frequencyRanges().front().first / 2.0;
        Source       blk{};
        blk.device_parameter = deviceArgs();
        blk.sample_rate      = 2.5e6;
        blk.frequency        = std::vector<double>{freqHz};
        const auto why       = [](const auto& r) { return r.has_value() ? std::string("the start went through") : std::string(r.error().message); };
        std::ignore          = blk.changeStateTo(gr::lifecycle::State::INITIALISED);
        const std::string first = why(blk.changeStateTo(gr::lifecycle::State::RUNNING));
        const bool        kept  = !blk.frequency->empty() && blk.frequency->front() == freqHz;
        std::ignore             = blk.changeStateTo(gr::lifecycle::State::INITIALISED);
        const std::string again = why(blk.changeStateTo(gr::lifecycle::State::RUNNING));
        std::printf("a-refused-source-start-is-refused-again:\n  tune: %s\n  again: %s\n", first.c_str(), again.c_str());
        expect(first.ends_with("Hz is outside the radio's coverage; the LO stays where it is")) << "a frequency outside the coverage fails the source's start";
        expect(kept) << "a refused first tune leaves the setting as asked";
        expect(again.ends_with("Hz is outside the radio's coverage; the LO stays where it is")) << "and a reset and second start of the same source are refused again";
        expect(radioReleased());
    });

    cases("ad9361.tx-tone-reaches-the-receiver", [] {
        /*| frame: the receiver and the transmitter share the radio's sample clock, so both
                graphs ask for the same rate. The tone sits a tenth of the rate above center,
                and the bin below center is its image.
        */
        constexpr double kRate   = 1.0e6;
        constexpr double kFreq   = 915.0e6;
        constexpr double kOffset = 0.1 * kRate;
        std::pair<double, double> idle, on, off;
        std::uint64_t             underruns = 0;
        const bool ran = withGraph(sourceSettings(kRate, kFreq, 30.0), [&](Source&, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the receiver starts";
            settle(300);
            idle = toneOver(kOffset, kRate, 500);
            const bool sent = withTxGraph(sinkSettings(kRate, kFreq), 0U, [&](capture::ad9361::Sink& sink, auto&) {
                settle(500);
                expect(sink.deviceUp()) << "the sink opened the radio beside the source";
                on        = toneOver(kOffset, kRate, 500);
                underruns = sink.underrunEvents();
            });
            expect(sent) << "the transmit graph ran";
            settle(300);
            off = toneOver(kOffset, kRate, 500);
        });
        std::printf("tx-tone-reaches-the-receiver: at %.2f dB, the tone bin reads %.1f dBFS idle, %.1f transmitting (image %.1f), %.1f after the stop; %llu underrun events\n", txGainDb(),
                    idle.first, on.first, on.second, off.first, static_cast<unsigned long long>(underruns));
        expect(ran);
        expect(on.first > idle.first + 20.0) << "the tone rises out of the idle floor";
        expect(on.first > on.second + 20.0) << "above center, where it was sent, and not at its image";
        expect(off.first < on.first - 20.0) << "the stop silences the transmitter";
    });

    cases("ad9361.tx-eob-ends-the-burst", [] {
        /*| frame: a half-second burst ends with gr:tx_eob = true, and the transmit graph stays
                up after it. The receiver measures while the burst is open and after the sink
                has closed it.
        */
        constexpr double     kRate   = 1.0e6;
        constexpr double     kFreq   = 915.0e6;
        constexpr double     kOffset = 0.1 * kRate;
        constexpr gr::Size_t kBurst  = 500'000U;
        std::pair<double, double> during, after;
        std::uint64_t             queued    = 1;
        std::uint64_t             underruns = 0;
        bool                      opened    = false;
        bool                      closed    = false;
        double                    openS     = 0.0;
        const bool ran = withGraph(sourceSettings(kRate, kFreq, 30.0), [&](Source&, auto&) {
            expect(waitForSamples(std::chrono::milliseconds(6000))) << "the receiver starts";
            settle(300);
            const bool sent = withTxGraph(sinkSettings(kRate, kFreq), kBurst, [&](capture::ad9361::Sink& sink, auto&) {
                const auto t0       = Clock::now();
                const auto waitFor  = [](auto&& done, int limitMs) {
                    const auto deadline = Clock::now() + std::chrono::milliseconds(limitMs);
                    while (!done() && Clock::now() < deadline) {
                        settle(5);
                    }
                    return done();
                };
                opened = waitFor([&] { return sink._burstOpen.load(); }, 5000);
                openS  = std::chrono::duration<double>(Clock::now() - t0).count();
                settle(50);
                during = toneOver(kOffset, kRate, 200);
                closed = waitFor([&] { return !sink._burstOpen.load(); }, 3000);
                settle(100);
                after     = toneOver(kOffset, kRate, 300);
                queued    = sink.queuedSamples();
                underruns = sink.underrunEvents();
            });
            expect(sent) << "the transmit graph ran";
        });
        std::printf("tx-eob-ends-the-burst: the burst opened %.2f s after the start; the tone bin reads %.1f dBFS during it and %.1f after its end; %llu samples queued after it, %llu underrun events\n",
                    openS, during.first, after.first, static_cast<unsigned long long>(queued), static_cast<unsigned long long>(underruns));
        expect(ran && opened && closed) << "the burst opened and the sink closed it while the graph ran";
        expect(during.first > after.first + 20.0) << "the burst's end silences the transmitter";
        expect(queued == 0U) << "the burst was played out";
    });

    return cases.report();
}
