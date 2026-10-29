/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/Block.hpp>
#ifdef CAPTURE_BLOCK_LABELS
#include <gnuradio-4.0/BlockAttributes.hpp>
#endif
#include <gnuradio-4.0/Tag.hpp>

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Properties.hpp>
#include <capture/common/Ring.hpp>
#include <capture/common/Source.hpp>
#include <capture/ad9361/Device.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <format>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace capture::ad9361 {

/*| role: the receive source for AD9361 radios driven by the Linux ad9361 driver, over libiio:
        the ADALM-Pluto, the Red Wire Technologies radios and other Zynq AD9361 designs. It
        opens the radio's IIO context, on the radio itself or across the network or USB from
        iiod, and delivers one or two receive channels as complex float on out and out2.
    contract: device_parameter names the context, uri=<libiio URI> or ip=<address>, and the
        FPGA image's packing, image=adi (the default) or image=rwt. An empty selector opens
        libiio's default context. channels lists the device channels streamed, {0}, {1} or
        {0, 1}; out carries the first and out2 the second. channels, buffer_size and
        device_parameter are taken when the block starts.
    contract: sample_rate is any whole rate from kRateFloorHz to 61.44 MS/s and applies in the
        running stream. A request is rounded to whole hertz, one outside that range is brought
        to the nearer end, and one that is not a number is written at kRateFloorHz; a request
        moved by more than half a hertz is named on standard error. A rate other than the one
        another block of this process holds the radio at is refused.
    contract: frequency is the receive LO over the coverage the radio states: 70 MHz to 6 GHz
        on an AD9361 or AD9364, 325 MHz to 3.8 GHz on an AD9363. A frequency outside the
        coverage, or one the radio refuses, fails the start, and in the running stream leaves
        the LO where it is; gr:frequency is forwarded only once a tune of this start has
        landed. rx_gains is the receive gain in dB on every streamed channel, bounded to the
        range the radio states for the LO's band. gain_mode hands the gain to the
        AD9361's AGC, in the mode the AGC_MODE control picks. rx_antennae names the AD9361
        input, such as A_BALANCED. bandwidth sets the analog filter, and 0 follows the rate.
        frequency_correction is applied to the LO command alone; the sample clock keeps the
        radio's reference.
    frame: the first block after a rate change, a lost buffer or a radio FIFO overflow carries
        gr:rx_overflow, with rx_overflow_cause "rate change", "host ring full" or "radio
        FIFO". A lost buffer's tag also carries gr:n_dropped_samples, the frames the host
        ring refused, per channel. A rate change's tag carries it too: the frames the change
        discarded and the frames the ring refused among them or where they end, per channel.
        An overflow or a lost buffer that falls among the frames a rate change discards, or
        where they end, joins the rate change's tag and adds no tag of its own.
    frame: under image=rwt a pulse-per-second from the radio marks the first sample after
        it with gr:trigger_name "PPS_RADIO", gr:trigger_time the UTC second nearest the host's
        time for that sample, gr:trigger_offset 0, and gr:trigger_meta_info holding the radio's
        own 56-bit sample count as radio_sample_count and that host time as gr:local_time.
        The radio inserts these tags only after a program on the radio enables them; a host
        cannot.
    frame: droppedSamples() counts the frames the host ring refused and the frames a rate
        change discarded, per channel.
        radioOverflows() counts the overflows the radio reported. droppedTags() counts the
        tags the output port had no room for.
    frame: while the radio is open, the sensors property states radioOverflows() as
        rx_overflows and droppedSamples() as rx_dropped_samples, both glance rows, and
        droppedTags() as rx_dropped_markers, a detail row. Once an rf_bandwidth write of this
        start has landed, it also states rx_analog_bandwidth, the width rf_bandwidth reads
        back after the last write.
    frame: a stream that delivers nothing for five seconds, or whose refill fails, ends the
        block with an error on the message port.
    invariant: a reader thread refills the libiio buffer and decodes it into the ring. A
        consumer thread drains the staged settings, places the tags and publishes. Every
        libiio attribute and register call goes through _ctrlMutex.
    trap: the AD9361 runs one sample clock for both directions. While a sink of this process
        holds the same radio, this block refuses every rate but the sink's, at a start and at a
        rate write. A program in another process that reaches the radio through iiod is not
        refused, and its rate write moves this block's stream with no marker.
    trap: a wrong image setting delivers a stream with its samples out of order and no
        error. Every image offers the same IIO devices, and no attribute read here tells an
        RWT image from an ADI one.
*/
struct Source : gr::Block<Source> {
    using Description = gr::Doc<R"(AD9361 receive source via libiio.
Opens the radio's IIO context locally or through iiod and delivers one or two AD9361 receive channels as complex float.
Covers the ADALM-Pluto, the Red Wire Technologies radios and other radios whose AD9361 runs under the Linux ad9361 driver.
Every source block in this library shares one set of setting names, so one caller drives every family identically.)">;

    gr::PortOut<std::complex<float>>               out;
    gr::PortOut<std::complex<float>, gr::Optional> out2;

    static constexpr const char* kDevice             = "ad9361"; // the family name a device listing and a probe carry
    static constexpr bool        kListingOpens       = true;  // the listing opens each scanned context to read its devices
    static constexpr double      kDefaultRateHz      = 5'000'000.0;
    static constexpr double      kDefaultFrequencyHz = 2'400'000'000.0;
#ifdef CAPTURE_BLOCK_LABELS
    static constexpr auto attributes = gr::block::describe(1U, gr::block::labels::family(kDevice, "AD9361 radios driven through the Linux ad9361 driver and libiio."),
        gr::block::labels::role::source, gr::block::labels::holds::device, gr::block::labels::ingests::rf);
#endif

    gr::Annotated<std::string, "device", gr::Visible, gr::Doc<"informational; \"ad9361\"">>                                    device = std::string(kDevice);
    gr::Annotated<std::string, "device_parameter", gr::Visible, gr::Doc<"kwargs; uri= or ip=, and image=adi or image=rwt">>       device_parameter;
    gr::Annotated<double, "sample_rate", gr::Unit<"Hz">, gr::Visible, gr::Doc<"receive sample rate">>                          sample_rate = kDefaultRateHz;
    gr::Annotated<std::vector<double>, "frequency", gr::Unit<"Hz">, gr::Visible, gr::Doc<"receive LO frequency">>              frequency   = std::vector<double>{kDefaultFrequencyHz};
    gr::Annotated<std::vector<double>, "rx_gains", gr::Unit<"dB">, gr::Visible, gr::Doc<"receive gain on every streamed channel">> rx_gains = std::vector<double>{40.0};
    gr::Annotated<bool, "gain_mode", gr::Doc<"AD9361 AGC on/off">>                                                             gain_mode = false;
    gr::Annotated<double, "frequency_correction", gr::Unit<"ppm">, gr::Doc<"reference error applied to the LO">>               frequency_correction = 0.0;
    gr::Annotated<std::vector<std::string>, "rx_antennae", gr::Doc<"AD9361 input, such as A_BALANCED">>                         rx_antennae;
    gr::Annotated<bool, "emit_timing_tags", gr::Doc<"emit wall-clock timing tags">>                                            emit_timing_tags = true;
    gr::Annotated<float, "tag_interval", gr::Unit<"s">, gr::Doc<"minimum interval between timing tags">>                       tag_interval     = 1.0f;
    gr::Annotated<std::string, "trigger_name", gr::Doc<"tag trigger_name">>                                                    trigger_name     = std::string("SDR_WALLCLOCK");
    gr::Annotated<std::vector<gr::Size_t>, "channels", gr::Doc<"device channels streamed on out and out2">>                    channels         = std::vector<gr::Size_t>{0U};
    gr::Annotated<double, "bandwidth", gr::Unit<"Hz">, gr::Doc<"analog filter bandwidth; 0 follows the rate">>                 bandwidth        = 0.0;
    gr::Annotated<gr::Size_t, "buffer_size", gr::Doc<"samples per transfer; 0 derives it from the rate">>                      buffer_size      = 0U;

    GR_MAKE_REFLECTABLE(Source, out, out2, device, device_parameter, sample_rate, frequency, rx_gains, gain_mode, frequency_correction, rx_antennae, emit_timing_tags, tag_interval, trigger_name, channels, bandwidth, buffer_size);

    // How long one libiio call over the network may wait, and how long a stream may deliver
    // nothing before the block gives up on it.
    static constexpr unsigned     kTimeoutMs     = 2000U;
    static constexpr std::int64_t kNoDataDeadNs  = 5'000'000'000;
    // The most one drain takes from the ring, in frames.
    static constexpr std::size_t  kDrainFrames   = 1UZ << 15;
    // The event type a reader records for a buffer the ring refused, outside the 7-bit tag types.
    static constexpr std::uint8_t kHostGapEvent  = 0x80U;

    /*| role: the trigger name of a pulse-per-second tag, and the key in its meta info that
            holds the radio's own sample count at the pulse.
    */
    static constexpr const char* kPpsTriggerName = "PPS_RADIO";
    static constexpr const char* kRadioCountKey  = "radio_sample_count";

    /*| contract: the UTC second a pulse marks, in nanoseconds: the whole second nearest the
            host's time for the sample after it.
        trap: the host's clock decides the second, so a clock more than half a second off
            names the wrong one.
        verified-by: ad9361.a-pulse-names-the-nearest-second
    */
    static constexpr std::uint64_t ppsSecondNs(std::uint64_t hostNs) { return (hostNs + 500'000'000ULL) / 1'000'000'000ULL * 1'000'000'000ULL; }

    static constexpr std::array<const char*, 3> kAgcModes{"slow_attack", "fast_attack", "hybrid"};
    static constexpr std::array<const char*, 3> kTrackingIds{"QUADRATURE_TRACKING", "RF_DC_TRACKING", "BB_DC_TRACKING"};
    static constexpr std::array<const char*, 3> kTrackingAttrs{"quadrature_tracking_en", "rf_dc_offset_tracking_en", "bb_dc_offset_tracking_en"};

    // ---- state -----------------------------------------------------------------------

    SharedContext               _ctx;   // shared with the family's other blocks on this URI
    std::shared_ptr<RadioRates> _rates; // the rate each block of the process holds the radio at

    iio_device*                _phy = nullptr;
    iio_device*                _rx  = nullptr;
    iio_channel*               _lo  = nullptr;              // ad9361-phy output altvoltage0, the receive LO
    std::vector<iio_channel*>  _phyRx;                      // ad9361-phy input voltage<ch>, one per streamed channel
    std::unique_ptr<iio::Stream> _stream;                   // cf-ad9361-lpc's, with the scan channels this block enabled
    std::vector<std::size_t>   _channels{0UZ};              // the device channels streaming, written by start() before any thread runs
    std::size_t                _nch = 1UZ;
    mutable std::mutex         _ctrlMutex;                  // every libiio attribute call, and the handles above
    std::mutex                 _teardownMutex;              // one hardwareTeardown() owner at a time
    std::atomic<bool>          _deviceUp{false};
    std::atomic<bool>          _streamDead{false};
    bool                       _secondPublished = false;    // a second channel streams to a reader on out2; written by start()
    ConsumerThread             _reader;                     // buffer refills into the ring
    ConsumerThread             _consumer;                   // ring -> tags -> output ports

    /*| invariant: the last value each applier wrote, guarded by _ctrlMutex and cleared by
            start(). An applier that finds its value already written writes nothing.
        trap: settingsChanged replays the staged map after streaming starts. Without these
            caches the replay rewrites the rate, which makes the AD9361 recalibrate.
    */
    double                     _appliedRateHz = 0.0;
    double                     _appliedBwHz   = 0.0;
    double                     _appliedFreqHz = 0.0;
    long long                  _appliedLoCmd  = -1;
    std::string                _appliedPort;
    int                        _appliedAgc    = -1;         // -1 none written, 0 manual, 1 the AGC
    double                     _appliedGainDb = std::numeric_limits<double>::quiet_NaN();
    std::atomic<double>        _radioBwHz{0.0};             // the filter the radio holds after the last write, 0 before one

    /*| role: the gain a caller last asked for, written to the radio while the AGC is off.
        trap: the AGC moves the hardware gain and the driver reads back what the AGC chose, so
            leaving the AGC writes this held value again.
    */
    std::atomic<double>        _heldGainDb{40.0};
    std::atomic<int>           _agcMode{0};                 // index into kAgcModes
    std::array<std::atomic<int>, 3> _tracking{1, 1, 1};     // the tracking toggles as last written, 1 on
    AvailableRange             _gainRange = rxGainRangeFor(kDefaultFrequencyHz); // the range the radio states at the LO in force, under _ctrlMutex
    AvailableRange             _bwBounds = kRxBandwidthBounds; // the analog filter's range the radio states, under _ctrlMutex
    std::atomic<double>        _surfaceLoHz{kDefaultFrequencyHz};
    std::atomic<bool>          _surfaceAgc{false};

    std::unique_ptr<SpscRing<CF32>> _ring;                  // frame-interleaved; replaced by start() before any thread runs
    SampleDecoder              _decoder;                    // reader thread
    std::vector<CF32>          _decoded;                    // reader-thread scratch
    std::vector<StreamEvent>   _events;                     // reader-thread scratch
    std::vector<CF32>          _drain0;                     // consumer-thread scratch, out
    std::vector<CF32>          _drain1;                     // consumer-thread scratch, out2

    /*| role: the in-band tags and host gaps the reader found, with the ring position of the
            frame each one precedes, in ring elements.
        frame: a tag arrives about once a second, so the queue's mutex is seldom contended.
    */
    struct Pending {
        std::size_t   position = 0;
        std::uint8_t  type     = 0;
        std::uint64_t value    = 0;
    };
    std::mutex                 _pendingMutex;
    std::deque<Pending>        _pending;
    std::atomic<bool>          _flushOwed{false};           // a rate change discards what the ring holds
    std::optional<std::uint64_t> _rateLossOwed;             // the count of a "rate change" marker not yet published, consumer thread
    std::atomic<std::uint64_t> _radioOverflows{0};
    std::atomic<std::uint64_t> _hostDropped{0};               // frames the ring refused or a rate change discarded
    std::uint64_t              _lastTagTimeNs = 0;

    OutputState           _outputState;
    ControlProperties     _controlProperties{this};
    TeardownGuard<Source> _teardown{this}; // the last member, so its teardown runs first

    std::optional<gr::Message> controlProperty(std::string_view name, gr::Message message) { return answerControlProperty(*this, name, std::move(message)); }

    std::vector<ControlDesc> controlSurface() {
        TruthContext ctx;
        ctx.gainMode   = _surfaceAgc.load(std::memory_order_acquire);
        ctx.centerFreq = _surfaceLoHz.load(std::memory_order_acquire);
        return describeControls(ctx);
    }

    // ---- device truth ----------------------------------------------------------------

    /*| role: what an open radio stated about itself, empty until one has been opened.
        frame: process-wide, because describeControls() is static and a caller lays out its
            controls while the radio is stopped. Passing a value stores it; passing nothing
            reads what is stored.
    */
    static std::optional<AvailableRange> probedLoRange(const std::optional<AvailableRange>* store = nullptr) {
        static std::mutex                    guard;
        static std::optional<AvailableRange> range;
        std::lock_guard                      lock(guard);
        if (store != nullptr) {
            range = *store;
        }
        return range;
    }

    static std::vector<std::pair<double, double>> frequencyRanges() {
        const auto lo = probedLoRange();
        return {lo.has_value() ? std::pair{lo->min, lo->max} : std::pair{70.0e6, 6.0e9}};
    }

    // A ladder of usual rates inside the range this build reaches; any whole rate in the range is taken.
    static std::vector<double> sampleRates() {
        std::vector<double> out;
        for (const double hz : {1.0e6, 2.5e6, 5.0e6, 10.0e6, 20.0e6, 30.72e6, 61.44e6}) {
            if (rateInRange(hz)) {
                out.push_back(hz);
            }
        }
        return out;
    }

    /*| contract: the surface at the context's LO and gain mode: the receive gain "RX" over the
            band's range, the AGC mode, the three tracking loops, GAIN_MODE, and the rate and
            frequency.
    */
    static std::vector<ControlDesc> describeControls(const TruthContext& ctx) {
        static constexpr const char* kAgcLabels[] = {"Slow attack", "Fast attack", "Hybrid"};
        static constexpr ControlSeed kControls[]  = {
            // id, label, unit, kind, min, max, step, def, options, n, list, n, isGain, inopAbove
            {"RX", "RX", "dB", 1, -1.0, 73.0, 1.0, 40.0, nullptr, 0, nullptr, 0, true, 0.0},
            {"AGC_MODE", "AGC mode", "", 3, 0.0, 2.0, 1.0, 0.0, kAgcLabels, 3, nullptr, 0, false, 0.0},
            {"QUADRATURE_TRACKING", "Quadrature tracking", "", 2, 0.0, 1.0, 1.0, 1.0, nullptr, 0, nullptr, 0, false, 0.0},
            {"RF_DC_TRACKING", "RF DC tracking", "", 2, 0.0, 1.0, 1.0, 1.0, nullptr, 0, nullptr, 0, false, 0.0},
            {"BB_DC_TRACKING", "BB DC tracking", "", 2, 0.0, 1.0, 1.0, 1.0, nullptr, 0, nullptr, 0, false, 0.0},
        };
        auto       out   = controlsFromSeeds(kControls);
        const auto range = rxGainRangeFor(ctx.centerFreq > 0.0 ? ctx.centerFreq : kDefaultFrequencyHz);
        for (auto& c : out) {
            if (c.id == "RX") {
                c.min      = range.min;
                c.max      = range.max;
                c.defValue = std::clamp(c.defValue, c.min, c.max);
            }
        }
        out.push_back(gainModeControl(ctx.gainMode));
        appendRateAndFrequency(out, sampleRates(), kDefaultRateHz, std::pair{kRateFloorHz, kMaxRateHz}, frequencyRanges(), kDefaultFrequencyHz);
        return out;
    }

    static std::vector<std::pair<std::string, std::string>> enumerateDevices() { return listRadios(); }

    bool          deviceUp() const { return _deviceUp.load(std::memory_order_acquire); }
    bool          streamDead() const { return _streamDead.load(std::memory_order_acquire); }
    std::uint64_t droppedSamples() const { return _hostDropped.load(std::memory_order_relaxed); }
    std::uint64_t radioOverflows() const { return _radioOverflows.load(std::memory_order_relaxed); }
    std::size_t   droppedTags() const { return droppedTagCount(*this); }

    /*| contract: what the stream has lost and the analog filter the radio holds, while the
            radio is open: the overflows, the dropped samples, the lost markers, and the filter
            once a write has landed. False with nothing published while the radio is closed.
        invariant: the counts are atomics, and the filter is the value the last write read
            back. A sweep makes no libiio call and takes no lock.
        verified-by: ad9361.the-sensors-state-what-the-stream-lost
        verified-by: ad9361.the-filter-stated-is-the-one-the-radio-holds
    */
    bool readSensors(std::vector<SensorReading>& out) const {
        if (!deviceUp()) {
            return false;
        }
        out.push_back({.id = "rx_overflows", .label = "Overflows", .value = std::format("{}", radioOverflows()), .good = true, .brief = true});
        out.push_back(droppedSamplesReading(droppedSamples()));
        out.push_back(droppedMarkersReading(droppedTags()));
        if (const double hz = _radioBwHz.load(std::memory_order_acquire); hz > 0.0) {
            out.push_back(analogBandwidthReading(hz));
        }
        return true;
    }

    // ---- lifecycle -------------------------------------------------------------------

    void start() {
        noteOutputAtStart(*this);
        _lastTagTimeNs = 0;
        _streamDead.store(false, std::memory_order_release);
        _radioOverflows.store(0, std::memory_order_relaxed);
        _hostDropped.store(0, std::memory_order_relaxed);
        _flushOwed.store(false, std::memory_order_relaxed);
        _rateLossOwed.reset();
        {
            std::lock_guard lock(_pendingMutex);
            _pending.clear();
        }
        _channels.assign(channels->begin(), channels->end());
        _nch = std::max<std::size_t>(_channels.size(), 1UZ);

        const auto packing = packingFor(device_parameter.value);
        if (!packing.has_value()) {
            throwStartFailure("ad9361::Source", std::format("image={} names no FPGA image; adi or rwt", kwargsValue(device_parameter.value, "image")));
        }
        _decoder.packing = *packing;

        std::string error;
        SharedContext ctx = acquireContext(contextUriFor(device_parameter.value), kTimeoutMs, error);
        if (!ctx) {
            throwStartFailure("ad9361::Source", error);
        }
        iio_device* phy = iio_context_find_device(ctx.get(), kPhyName);
        iio_device* rx  = iio_context_find_device(ctx.get(), kRxName);
        if (phy == nullptr || rx == nullptr) {
            throwStartFailure("ad9361::Source", std::format("the context holds no {} and {}; the radio has loaded no FPGA image (on the radio: fpgaloader switch default)", kPhyName, kRxName));
        }
        // An AD9364 or a one-receiver image offers voltage0 and voltage1 alone.
        const std::size_t deviceChannels = iio_device_find_channel(rx, "voltage2", false) != nullptr && iio_device_find_channel(rx, "voltage3", false) != nullptr ? 2UZ : 1UZ;
        if (const auto refusal = channelsRefusal(_channels, deviceChannels, out2.isConnected()); refusal.has_value()) {
            throwStartFailure("ad9361::Source", *refusal);
        }

        {
            std::lock_guard lock(_ctrlMutex);
            _ctx   = std::move(ctx);
            _rates = radioRatesFor(_ctx.get());
            _phy   = phy;
            _rx    = rx;
            _lo    = iio_device_find_channel(phy, "altvoltage0", true);
            _phyRx.clear();
            for (const std::size_t ch : _channels) {
                _phyRx.push_back(iio_device_find_channel(phy, std::format("voltage{}", ch).c_str(), false));
            }
            _appliedRateHz = 0.0;
            _appliedBwHz   = 0.0;
            _radioBwHz.store(0.0, std::memory_order_relaxed);
            _appliedFreqHz = 0.0;
            _appliedLoCmd  = -1;
            _appliedPort.clear();
            _appliedAgc    = -1;
            _appliedGainDb = std::numeric_limits<double>::quiet_NaN();
            _gainRange     = rxGainRangeFor(frequency->empty() ? kDefaultFrequencyHz : frequency->front());
            _surfaceAgc.store(gain_mode, std::memory_order_release);
            _bwBounds = kRxBandwidthBounds;
            if (const auto text = readAttr(iio_device_find_channel(phy, "voltage0", false), "rf_bandwidth_available"); text.has_value()) {
                _bwBounds = parseAvailableRange(*text).value_or(kRxBandwidthBounds);
            }
            if (const auto text = readAttr(_lo, "frequency_available"); text.has_value()) {
                const auto range = parseAvailableRange(*text);
                std::ignore      = probedLoRange(&range);
            }
        }

        /*| trap: a stop can arrive while the context opens across the network, and the
                framework's stop() hook fires only from an active state. The teardown can
                then find no handle and return, while this thread holds a context it is about
                to stream from.
        */
        if (!gr::lifecycle::isActive(this->state()) && this->state() != gr::lifecycle::State::INITIALISED) {
            closeDevice();
            return;
        }

        if (const auto refusal = applySampleRate(); refusal.has_value()) {
            closeDevice();
            throwStartFailure("ad9361::Source", *refusal);
        }
        applyBandwidth();
        applyAntenna();
        if (const auto refusal = applyFrequency(); refusal.has_value()) {
            closeDevice();
            throwStartFailure("ad9361::Source", *refusal);
        }
        applyGainMode();
        applyGain();

        const std::size_t transfer = transferSamplesFor(static_cast<double>(sample_rate.value), buffer_size.value);
        int code = 0;
        {
            std::lock_guard           lock(_ctrlMutex);
            std::vector<iio_channel*> scan;
            for (const auto& id : scanChannelsFor(_channels)) {
                if (iio_channel* chn = iio_device_find_channel(_rx, id.c_str(), false); chn != nullptr) {
                    scan.push_back(chn);
                }
            }
            _stream = iio::Stream::open(_rx, scan, transfer, false, code);
        }
        if (!_stream) {
            closeDevice();
            throwStartFailure("ad9361::Source", std::format("opening the receive stream ({} samples): {}", transfer, iioError(code != 0 ? code : -EIO)));
        }

        /*| frame: the ring holds half a second of frames at the rate and at least four
                transfers, so the reader keeps draining the radio while the publish waits on a
                full edge.
        */
        const std::size_t frames = std::max(static_cast<std::size_t>(static_cast<double>(sample_rate.value) / 2.0), 4UZ * transfer);
        _ring                    = std::make_unique<SpscRing<CF32>>(frames * _nch);
        // The flag latches across streams, so one left from before this start is cleared here.
        std::ignore = readOverflowFlag();
        _decoder.reset();
        _secondPublished = _nch > 1UZ && out2.isConnected();

        _deviceUp.store(true, std::memory_order_release);
        _reader.start([this] {
            nameStreamingThread("ad9361", this->name.value);
            readerLoop();
        });
        _consumer.start([this] {
            nameStreamingThread("ad9361pub", this->name.value);
            consumerLoop();
        });
        offerControlProperties(*this);
        publishControlSurface(*this);
    }

    void stop() {
        if (hardwareTeardown()) {
            this->publishEoS();
        }
    }

    /*| contract: join both threads and release the context, answering true to the one caller
            that released an open radio.
        why: the consumer goes first, so nothing is still publishing while the reader is
            unblocked. The stream's cancel ends a refill the reader is parked in, and the
            context's timeout bounds one that started after it.
    */
    bool hardwareTeardown() {
        std::lock_guard teardown(_teardownMutex);
        bool            hadDevice = false;
        {
            std::lock_guard lock(_ctrlMutex);
            hadDevice = _ctx != nullptr;
        }
        _deviceUp.store(false, std::memory_order_release);
        _consumer.requestStop();
        _consumer.joinIfRunning();
        _reader.requestStop();
        {
            std::lock_guard lock(_ctrlMutex);
            if (_stream) {
                _stream->cancel();
            }
        }
        _reader.joinIfRunning();
        closeDevice();
        return hadDevice;
    }

    gr::work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!gr::lifecycle::isActive(this->state()) || stopWhenOutputIsUnconnected(*this)) {
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        if (_deviceUp.load(std::memory_order_acquire) && _streamDead.load(std::memory_order_acquire)) {
            this->emitErrorMessage("work()", "the radio stopped delivering samples (network or iiod failure?)");
            this->requestStop();
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        return {requestedWork, 0UZ, gr::work::Status::OK};
    }

    // ---- staged settings (applied on the consumer thread) ----------------------------

    void settingsChanged(const gr::property_map& /*oldSettings*/, gr::property_map& newSettings, gr::property_map& forwardSettings) {
        const ControlSurfaceCheck<Source> surfaceCheck{*this}; // sends a moved surface once the body has let go of its locks
        if (newSettings.contains("gain_mode")) {
            _surfaceAgc.store(gain_mode, std::memory_order_release);
        }
        if (!_deviceUp.load(std::memory_order_acquire)) {
            return; // start() applies every setting
        }
        if (newSettings.contains("sample_rate")) {
            const double before = _appliedRateHz;
            if (const auto refusal = applySampleRate(); refusal.has_value()) {
                std::fprintf(stderr, "[ad9361::Source] %s\n", refusal->c_str());
            }
            if (_appliedRateHz != before) {
                _flushOwed.store(true, std::memory_order_release);
            }
            applyBandwidth();
            forwardSettings.insert_or_assign(std::pmr::string(gr::tag::SAMPLE_RATE.key()), sample_rate.value);
        }
        if (newSettings.contains("bandwidth")) {
            applyBandwidth();
        }
        if (newSettings.contains("frequency") || newSettings.contains("frequency_correction")) {
            if (const auto refusal = applyFrequency(); refusal.has_value()) {
                std::fprintf(stderr, "[ad9361::Source] %s\n", refusal->c_str());
            }
            if (!frequency->empty() && _appliedFreqHz > 0.0) {
                forwardSettings.insert_or_assign(std::pmr::string(gr::tag::FREQUENCY.key()), frequency->front());
            }
        }
        if (newSettings.contains("rx_antennae")) {
            applyAntenna();
        }
        if (newSettings.contains("gain_mode")) {
            applyGainMode();
        }
        if (newSettings.contains("rx_gains")) {
            applyGain();
        }
    }

    // ---- controls (a caller's thread) ------------------------------------------------

    /*| contract: "RX" is the receive gain on every streamed channel, bounded to the range in
            force. Under the AGC the value is held and written when the AGC lets go.
    */
    bool setElementGain(const std::string& element, double dB) {
        if (element != "RX") {
            return false;
        }
        CtrlGuard guard(_ctrlMutex, _deviceUp);
        if (!guard) {
            return false;
        }
        const double held = std::clamp(dB, _gainRange.min, _gainRange.max);
        _heldGainDb.store(held, std::memory_order_relaxed);
        if (_appliedAgc == 1) {
            return true;
        }
        return writeGainLocked(held);
    }

    double elementGain(const std::string& element) const { return element == "RX" ? _heldGainDb.load(std::memory_order_relaxed) : 0.0; }

    bool setControl(const std::string& id, double value) {
        CtrlGuard guard(_ctrlMutex, _deviceUp);
        if (!guard) {
            return false;
        }
        if (id == "AGC_MODE") {
            const int mode = std::clamp(static_cast<int>(std::lround(value)), 0, static_cast<int>(kAgcModes.size()) - 1);
            _agcMode.store(mode, std::memory_order_relaxed);
            return _appliedAgc != 1 || writeModeLocked(kAgcModes[static_cast<std::size_t>(mode)]);
        }
        for (std::size_t i = 0; i < kTrackingIds.size(); ++i) {
            if (id == kTrackingIds[i]) {
                const int on = value >= 0.5 ? 1 : 0;
                // The three loops are shared by both receivers, and voltage0 carries them.
                if (!writeAttr(iio_device_find_channel(_phy, "voltage0", false), kTrackingAttrs[i], std::to_string(on))) {
                    return false;
                }
                _tracking[i].store(on, std::memory_order_relaxed);
                return true;
            }
        }
        return false;
    }

    std::optional<double> controlValue(const std::string& id) const {
        if (id == "AGC_MODE") {
            return static_cast<double>(_agcMode.load(std::memory_order_relaxed));
        }
        for (std::size_t i = 0; i < kTrackingIds.size(); ++i) {
            if (id == kTrackingIds[i]) {
                return static_cast<double>(_tracking[i].load(std::memory_order_relaxed));
            }
        }
        return std::nullopt;
    }

    // ---- appliers (the device open; each takes _ctrlMutex) ---------------------------

    /*| contract: write sample_rate to the radio, as rateToWrite has it, and leave the setting
            at the rate the radio reads back. A rate another block holds the radio away from,
            or one the radio refuses, leaves the rate in force and answers why.
        frame: with libad9361 compiled in, ad9361_set_bb_rate designs and loads the FIR the
            rate needs. Without it, the FIR is switched off and the rate written as it stands,
            which the AD9361 takes from 25e6/12 S/s up.
    */
    std::optional<std::string> applySampleRate() {
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return std::nullopt;
        }
        const double want = static_cast<double>(sample_rate.value);
        if (want == _appliedRateHz) {
            return std::nullopt;
        }
        const auto ran = applyRadioRate(*_rates, this, _phy, false, want, "ad9361::Source");
        if (!ran.has_value()) {
            if (_appliedRateHz > 0.0) {
                sample_rate = _appliedRateHz;
            }
            return ran.error();
        }
        _appliedRateHz = *ran;
        sample_rate    = *ran;
        return std::nullopt;
    }

    /*| contract: write the analog filter, bandwidth where it is positive and the rate otherwise,
            inside the bounds the radio states, and keep the width rf_bandwidth reads back.
    */
    void applyBandwidth() {
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return;
        }
        const double hz = rxBandwidthFor(bandwidth.value, _appliedRateHz > 0.0 ? _appliedRateHz : static_cast<double>(sample_rate.value), _bwBounds);
        if (hz == _appliedBwHz) {
            return;
        }
        iio_channel* chn = iio_device_find_channel(_phy, "voltage0", false);
        if (!writeAttr(chn, "rf_bandwidth", std::format("{:.0f}", hz))) {
            std::fprintf(stderr, "[ad9361::Source] rf_bandwidth %.0f failed (continuing)\n", hz);
            return;
        }
        _appliedBwHz = hz;
        _radioBwHz.store(heldAfterWrite(readAttr(chn, "rf_bandwidth"), hz), std::memory_order_release);
    }

    /*| contract: tune the receive LO to the first frequency, corrected for the reference
            error. A request outside the coverage, or one the radio refuses, leaves the LO
            where it is, the setting at this start's last tune, or as asked before the first
            tune of a start, and answers why; start() fails on it.
        trap: before the first tune of a start the radio's LO is wherever an earlier program
            left it. The setting keeps the request and is never set from the radio's read-back,
            so every start of the block refuses a refused request again.
        frame: the gain range moves with the LO's band, and is read again after each tune.
    */
    std::optional<std::string> applyFrequency() {
        if (frequency->empty()) {
            return std::nullopt;
        }
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return std::nullopt;
        }
        const double f      = frequency->front();
        const auto   ranges = frequencyRanges();
        const auto   inside = frequencyInRange(f, ranges);
        const auto   revert = [this] {
            if (_appliedFreqHz > 0.0) {
                frequency = std::vector<double>{_appliedFreqHz};
            }
        };
        if (!inside.has_value()) {
            revert();
            return std::format("{:.0f} Hz is outside the radio's coverage; the LO stays where it is", f);
        }
        const long long cmd = std::llround(correctedTuneHz(*inside, frequency_correction));
        if (cmd == _appliedLoCmd) {
            return std::nullopt;
        }
        if (!writeAttr(_lo, "frequency", std::to_string(cmd))) {
            revert();
            return std::format("the radio refused a tune to {} Hz; the LO stays where it is", cmd);
        }
        _appliedLoCmd  = cmd;
        _appliedFreqHz = *inside;
        _surfaceLoHz.store(*inside, std::memory_order_release);
        _gainRange = rxGainRangeFor(*inside);
        if (const auto text = readAttr(_phyRx.empty() ? nullptr : _phyRx.front(), "hardwaregain_available"); text.has_value()) {
            if (const auto range = parseAvailableRange(*text); range.has_value()) {
                _gainRange = *range;
            }
        }
        return std::nullopt;
    }

    void applyAntenna() {
        if (rx_antennae->empty() || rx_antennae->front().empty()) {
            return;
        }
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return;
        }
        const std::string& port = rx_antennae->front();
        if (port == _appliedPort) {
            return;
        }
        iio_channel* chn = iio_device_find_channel(_phy, "voltage0", false);
        if (const auto text = readAttr(chn, "rf_port_select_available"); text.has_value()) {
            const auto names = parseAvailableList(*text);
            if (!names.empty() && std::ranges::find(names, port) == names.end()) {
                std::fprintf(stderr, "[ad9361::Source] the radio has no input %s; it offers %s\n", port.c_str(), text->c_str());
                return;
            }
        }
        if (!writeAttr(chn, "rf_port_select", port)) {
            std::fprintf(stderr, "[ad9361::Source] rf_port_select %s failed (continuing)\n", port.c_str());
            return;
        }
        _appliedPort = port;
    }

    /*| contract: write the gain control mode on every streamed channel: the AGC mode
            AGC_MODE picks where gain_mode is on, manual otherwise. Leaving the AGC writes the
            held gain again.
    */
    void applyGainMode() {
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return;
        }
        const int want = gain_mode ? 1 : 0;
        if (want == _appliedAgc) {
            return;
        }
        const char* mode = want == 1 ? kAgcModes[static_cast<std::size_t>(_agcMode.load(std::memory_order_relaxed))] : "manual";
        if (!writeModeLocked(mode)) {
            std::fprintf(stderr, "[ad9361::Source] gain_control_mode %s failed (continuing)\n", mode);
            return;
        }
        const bool leaving = _appliedAgc == 1 && want == 0;
        _appliedAgc        = want;
        if (leaving) {
            _appliedGainDb = std::numeric_limits<double>::quiet_NaN();
            std::ignore    = writeGainLocked(_heldGainDb.load(std::memory_order_relaxed));
        }
    }

    void applyGain() {
        if (rx_gains->empty()) {
            return;
        }
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return;
        }
        const double want = std::clamp(rx_gains->front(), _gainRange.min, _gainRange.max);
        if (want != rx_gains->front()) {
            rx_gains = std::vector<double>{want};
        }
        _heldGainDb.store(want, std::memory_order_relaxed);
        if (_appliedAgc == 1 || want == _appliedGainDb) {
            return;
        }
        std::ignore = writeGainLocked(want);
    }

    // The caller holds _ctrlMutex with the radio open.
    bool writeGainLocked(double dB) {
        bool ok = true;
        for (iio_channel* chn : _phyRx) {
            ok = writeAttr(chn, "hardwaregain", std::format("{}", dB)) && ok;
        }
        if (ok) {
            _appliedGainDb = dB;
        } else {
            std::fprintf(stderr, "[ad9361::Source] hardwaregain %.1f failed (continuing)\n", dB);
        }
        return ok;
    }

    bool writeModeLocked(const char* mode) {
        bool ok = true;
        for (iio_channel* chn : _phyRx) {
            ok = writeAttr(chn, "gain_control_mode", mode) && ok;
        }
        return ok;
    }

    // Releases everything start() took. Safe to call on a partial open.
    void closeDevice() {
        std::lock_guard lock(_ctrlMutex);
        _stream.reset();
        _phyRx.clear();
        _lo  = nullptr;
        _rx  = nullptr;
        _phy = nullptr;
        if (_rates) {
            releaseRadioRate(*_rates, this);
            _rates.reset();
        }
        _ctx.reset();
    }

    // ---- streaming -------------------------------------------------------------------

    static std::int64_t steadyNowNs() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

    /*| role: buffer refills into the ring, and nothing else.
        invariant: it blocks on nothing but the refill.
        trap: the buffer handle is read without the control mutex. Teardown stops this thread
            before it destroys the buffer, and start() creates it before this thread runs.
    */
    void readerLoop() {
        std::int64_t lastDataNs = steadyNowNs();
        while (!_reader.stopRequested()) {
            const ssize_t n = _stream->refill();
            if (_reader.stopRequested()) {
                return;
            }
            if (n < 0) {
                if (n == -ETIMEDOUT || n == -EAGAIN) {
                    if (steadyNowNs() - lastDataNs >= kNoDataDeadNs) {
                        std::fprintf(stderr, "[ad9361::Source] no samples for %lld s\n", static_cast<long long>(kNoDataDeadNs / 1'000'000'000));
                        _streamDead.store(true, std::memory_order_release);
                        return;
                    }
                    continue;
                }
                std::fprintf(stderr, "[ad9361::Source] refill: %s\n", iioError(n).c_str());
                _streamDead.store(true, std::memory_order_release);
                return;
            }
            lastDataNs              = steadyNowNs();
            const bool overflowed   = readOverflowFlag();
            const auto        data  = _stream->data();
            _decoded.clear();
            _events.clear();
            _decoder.decode({data.data(), data.size()}, _nch, _decoded, _events);
            stageDecoded(overflowed);
        }
    }

    /*| contract: put the decoded buffer into the ring, and queue its events at the positions of
            the frames they precede. A buffer the ring cannot take is dropped whole: its frames
            are counted, and a host gap holding that count is queued at the position the buffer
            would have filled, with its events there too. A gap already queued there is moved
            behind this buffer's events and takes its count. One marker then carries the whole
            loss, and the host ring's cause stays the latest at that position.
        why: buffers refused in a row all fall at one position, and a reader that merges the
            tags of one sample in order would keep one count of several and the cause of the
            event queued last.
    */
    void stageDecoded(bool overflowed) {
        const std::size_t base   = _ring->head.load(std::memory_order_relaxed);
        const bool        fitted = _decoded.empty() || _ring->write(_decoded.data(), _decoded.size());
        if (overflowed) {
            // Ahead of the buffer it was read after: the loss came before its first frame.
            _events.insert(_events.begin(), StreamEvent{.type = static_cast<std::uint8_t>(TagType::Overflow), .value = 0U, .frame = 0UZ});
        }
        if (_events.empty() && fitted) {
            return;
        }
        std::lock_guard lock(_pendingMutex);
        for (const auto& e : _events) {
            _pending.push_back({.position = fitted ? base + e.frame * _nch : base, .type = e.type, .value = e.value});
        }
        if (!fitted) {
            const std::uint64_t frames = _decoded.size() / _nch;
            _hostDropped.fetch_add(frames, std::memory_order_relaxed);
            std::uint64_t joined = frames;
            for (auto it = _pending.rbegin(); it != _pending.rend() && it->position == base; ++it) {
                if (it->type == kHostGapEvent) {
                    joined += it->value;
                    _pending.erase(std::next(it).base());
                    break;
                }
            }
            _pending.push_back({.position = base, .type = kHostGapEvent, .value = joined});
        }
    }

    /*| contract: whether the ADC core flagged an overflow since the last read, and clear the
            flag.
        why: the core latches the overflow its DMA side raises when the radio's kernel buffers
            are full, and libiio's buffer calls report none.
        invariant: the reader takes the control mutex only when it is free, so a slow attribute
            write never holds up a refill. The flag stays latched until the next read takes it.
    */
    bool readOverflowFlag() {
        std::unique_lock lock(_ctrlMutex, std::try_to_lock);
        return lock.owns_lock() && takeStatusFlag(_rx, kRxOverflowFlag);
    }

    /*| trap: this body runs on a thread of this block's own, and an exception leaving a
            std::thread body ends the process. applyChangedSettings() may throw and the scratch
            buffers grow on this thread.
    */
    void consumerLoop() {
        try {
            drainLoop();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[ad9361::Source] the publishing thread stopped: %s\n", e.what());
            _streamDead.store(true, std::memory_order_release);
        } catch (...) {
            std::fprintf(stderr, "[ad9361::Source] the publishing thread stopped on an unknown exception\n");
            _streamDead.store(true, std::memory_order_release);
        }
    }

    bool keepGoing() const { return gr::lifecycle::isActive(this->state()) && !_consumer.stopRequested(); }

    /*| contract: publish the tag one pending event stands for, at the head of the next block.
            hostNs is the host's time for that block's first sample.
        invariant: a pulse's tag carries the three trigger keys the periodic timing tag does,
            so it replaces that block's timing tag and restarts the timing interval. Two
            tags on one sample with the same keys would reach a reader that merges them as one.
    */
    void placeEvent(const Pending& e, std::uint64_t hostNs) {
        switch (e.type) {
        case static_cast<std::uint8_t>(TagType::Overflow):
            _radioOverflows.fetch_add(1, std::memory_order_relaxed);
            emitOverflowTag(*this, "radio FIFO");
            break;
        case kHostGapEvent: emitOverflowTag(*this, "host ring full", e.value); break;
        case static_cast<std::uint8_t>(TagType::Pps): {
            const std::uint64_t second = ppsSecondNs(hostNs);
            auto                tagMap = out.makeTagMap();
            gr::tag::put(tagMap, gr::tag::TRIGGER_NAME, std::string(kPpsTriggerName));
            gr::tag::put(tagMap, gr::tag::TRIGGER_TIME, second);
            gr::tag::put(tagMap, gr::tag::TRIGGER_OFFSET, 0.f);
            gr::property_map meta;
            meta.insert_or_assign(std::pmr::string(kRadioCountKey), e.value);
            meta.insert_or_assign(std::pmr::string(gr::tag::LOCAL_TIME.key()), hostNs);
            gr::tag::put(tagMap, gr::tag::TRIGGER_META_INFO, std::move(meta));
            publishOutputTag(*this, std::move(tagMap), 0UZ);
            _lastTagTimeNs = std::max(_lastTagTimeNs, hostNs);
            break;
        }
        default: break; // the receive path sends no other type
        }
    }

    /*| contract: take every radio overflow and host gap queued at a position up to upTo out of
            the queue. The overflows are counted in radioOverflows(). The answer is the gaps'
            frames, per channel. The caller holds _pendingMutex.
    */
    std::uint64_t foldLosses(std::size_t upTo) {
        std::uint64_t frames = 0U;
        for (auto it = _pending.begin(); it != _pending.end() && it->position <= upTo;) {
            if (it->type == static_cast<std::uint8_t>(TagType::Overflow)) {
                _radioOverflows.fetch_add(1, std::memory_order_relaxed);
            } else if (it->type == kHostGapEvent) {
                frames += it->value;
            } else {
                ++it;
                continue;
            }
            it = _pending.erase(it);
        }
        return frames;
    }

    /*| contract: discard the avail ring elements from tail after a rate change, with the events
            queued among them, and owe one "rate change" marker. Its gr:n_dropped_samples is
            the frames discarded plus the frames of each host gap queued among them or where
            they end, per channel. A radio overflow queued there is counted in radioOverflows()
            and publishes no marker of its own. A pulse queued where they end stays queued.
            A second rate change before the marker goes out adds its count to the one owed.
        verified-by: ad9361.a-rate-change-carries-the-loss-the-ring-refused
    */
    void flushRing(std::size_t tail, std::size_t avail) {
        const std::size_t end  = tail + avail;
        std::uint64_t     lost = avail / _nch;
        _ring->advanceTo(end);
        _hostDropped.fetch_add(lost, std::memory_order_relaxed);
        {
            std::lock_guard lock(_pendingMutex);
            lost += foldLosses(end);
            std::erase_if(_pending, [end](const Pending& p) { return p.position < end; });
        }
        _rateLossOwed = _rateLossOwed.value_or(0U) + lost;
        _outputState.rateTagOwed.store(true, std::memory_order_release);
    }

    /*| contract: place the pending events due at tail on the head of the next block, and
            answer how many of the take ring elements that block may hold, up to the next
            pending event. hostNs is the host's time for the block's first sample.
        contract: an owed "rate change" marker waits for a block to head, and with take below
            one frame nothing is placed. It then folds in every radio overflow and host gap
            queued up to tail, and goes out once, ahead of the events left at tail.
        why: an event queued where the discarded frames end falls on the marker's sample, the
            case stageDecoded() states for refused buffers. The reader can queue one there after
            the flush, from the first buffer it reads.
        verified-by: ad9361.an-overflow-after-a-rate-change-joins-its-marker
    */
    std::size_t placeDueEvents(std::size_t tail, std::size_t take, std::uint64_t hostNs) {
        std::lock_guard lock(_pendingMutex);
        if (_rateLossOwed.has_value()) {
            if (take < _nch) {
                return 0UZ;
            }
            const std::uint64_t lost = *_rateLossOwed + foldLosses(tail);
            _rateLossOwed.reset();
            emitOverflowTag(*this, "rate change", lost);
        }
        while (!_pending.empty() && _pending.front().position <= tail) {
            placeEvent(_pending.front(), hostNs);
            _pending.pop_front();
        }
        if (!_pending.empty() && _pending.front().position < tail + take) {
            take = _pending.front().position - tail;
        }
        return take;
    }

    /*| invariant: a pass takes at most kDrainFrames frames, and stops short at the position of
            the next pending event, so every event's tag lands on the frame it precedes.
        frame: a rate change discards the ring and the events in it. Samples taken at the old
            rate can still arrive in the refill that was running, about one transfer.
    */
    void drainLoop() {
        while (!_consumer.stopRequested()) {
            this->applyChangedSettings();

            std::size_t tail  = 0;
            std::size_t avail = _ring->available(tail);
            if (_flushOwed.exchange(false, std::memory_order_acq_rel)) {
                flushRing(tail, avail);
                continue;
            }

            // The clock is read at the drain, less the whole backlog the ring held.
            const std::uint64_t headNs = blockStartNs(wallClockNs(), avail / _nch, static_cast<double>(sample_rate.value));
            std::size_t         take   = placeDueEvents(tail, std::min(avail, kDrainFrames * _nch), headNs);
            take -= take % _nch;
            if (take == 0UZ) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            const std::size_t nFrames = take / _nch;
            _drain0.resize(nFrames);
            _drain1.resize(_nch > 1UZ ? nFrames : 0UZ);
            for (std::size_t f = 0; f < nFrames; ++f) {
                _drain0[f] = _ring->at(tail + f * _nch);
                if (_nch > 1UZ) {
                    _drain1[f] = _ring->at(tail + f * _nch + 1UZ);
                }
            }
            _ring->advanceTo(tail + take);
            publishFrames(_drain0, _drain1, headNs);
        }
    }

    /*| contract: publish one block of frames on out, and on out2 where a second channel
            streams to a reader there, holding and retrying until the ports take them or the
            consumer stops. Both writers advance by the same count.
        invariant: the progress counter moves after the spans are released, which is when the
            edges take the samples.
    */
    void publishFrames(std::span<const CF32> first, std::span<const CF32> second, std::uint64_t tWallNs) {
        auto&             writer  = out.streamWriter();
        auto&             writer2 = out2.streamWriter();
        const std::size_t n       = first.size();
        std::size_t       done    = 0UZ;
        const auto        roomNow = [&] { return _secondPublished ? std::min(writer.available(), writer2.available()) : writer.available(); };
        while (done < n && keepGoing()) {
            const std::size_t want = std::min(n - done, roomNow());
            std::size_t       k    = 0UZ;
            if (want > 0UZ) {
                auto span  = writer.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(want);
                auto span2 = writer2.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(_secondPublished ? want : 0UZ);
                if (!span.empty() && (!_secondPublished || !span2.empty())) {
                    k = _secondPublished ? std::min(span.size(), span2.size()) : span.size();
                    std::memcpy(span.data(), first.data() + done, k * sizeof(CF32));
                    if (_secondPublished) {
                        std::memcpy(span2.data(), second.data() + done, k * sizeof(CF32));
                    }
                    if (done == 0UZ) {
                        publishBlockHeadTags(*this, tWallNs);
                    }
                    if (_secondPublished) {
                        span2.publish(k);
                    }
                    span.publish(k);
                }
            } // the edges take the samples when the spans are released
            if (k == 0UZ) {
                awaitProgress(*this, this->progress->value(), writer.buffer().size());
                this->applyChangedSettings();
                continue;
            }
            done += k;
            this->progress->incrementAndGet();
            this->progress->notify_all();
        }
    }

    /*| contract: publish on out2 each tag the shared helpers publish on out, at the same
            offset, while a second channel streams to a reader there.
    */
    template <typename TagMap>
    void publishFurtherOutputTag(const TagMap& tagMap, std::size_t offset) {
        if (_secondPublished) {
            out2.publishTag(TagMap(tagMap), offset);
        }
    }
};

} // namespace capture::ad9361
