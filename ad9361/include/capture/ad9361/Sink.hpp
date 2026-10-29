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
#include <capture/common/Device.hpp>
#include <capture/common/Properties.hpp>
#include <capture/common/Ring.hpp>
#include <capture/common/Sink.hpp>
#include <capture/ad9361/Device.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <format>
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

/*| role: the transmit sink for AD9361 radios driven by the Linux ad9361 driver, over libiio.
        It takes complex float on in, and on in2 for a second channel, clips it, stages it in a
        ring, and a drain thread of its own packs it into libiio buffers and pushes them.
    contract: the settings are the source's names, with tx_ for the transmit ones. device_parameter
        names the context and the image as the source's does. channels lists the transmit
        channels, {0}, {1} or {0, 1}; in feeds the first and in2 the second, connected exactly
        when two are listed. sample_rate, frequency, frequency_correction, tx_gains, tx_antennae
        and bandwidth apply in the running stream; channels, buffer_size and device_parameter
        are taken at the start. tx_gains is the attenuation as a gain, -89.75 to 0 dB. A
        frequency outside the coverage, or one the radio refuses, fails the start, and in the
        running stream leaves the LO where it is.
    contract: sample_rate is any whole rate from kRateFloorHz to 61.44 MS/s. A request is
        rounded to whole hertz, one outside that range is brought to the nearer end, and one
        that is not a number is written at kRateFloorHz; a request moved by more than half a
        hertz is named on standard error. A rate other than the one another block of this
        process holds the radio at is refused.
    frame: the first samples staged start a burst: the drain creates the radio's buffer and
        powers the transmit LO up. A sample tagged gr:tx_eob = true is the last of its burst.
        The drain pads its buffer with zeros, waits for the radio to play what it holds, frees
        the buffer and powers the LO down, and the block takes no samples until then. A
        stream with no gr:tx_eob is one burst. With two channels a burst end on either input
        ends the burst on both. gr:tx_time and a gr:tx_eob that is not a bool are ignored, and
        the first of a run is reported on the message port.
    frame: underrunEvents() counts the transfers after which the DAC core flagged an underflow,
        a burst's first excepted: the DAC starves until that transfer arrives, every burst.
        discardedAtRateChangeSamples() counts what a rate change threw away, and
        unsentAtStopSamples() what a stop's deadline left unsent. All restart with a run.
    frame: analogBandwidthHz() is the transmit analog filter the radio holds while the radio
        is open, as rf_bandwidth reads back after the last write of this start. It is 0 while
        the radio is closed and before a write has landed.
    invariant: the drain thread owns the radio's buffer. Every libiio attribute and register
        call goes through _ctrlMutex.
    trap: the AD9361 runs one sample clock for both directions. While a source of this process
        holds the same radio, this block refuses every rate but the source's, at a start and at
        a rate write. A program in another process that reaches the radio through iiod is not
        refused, and its rate write moves this block's stream with no marker.
    trap: libiio and the ADI images carry no timed transmit and no burst marker. A burst's
        edges are as exact as a buffer creation and a play-out wait, milliseconds.
*/
struct Sink : gr::Block<Sink> {
    using Description = gr::Doc<R"(AD9361 sink via libiio.
CF32 in, clipped to full scale 1.0 and packed for the radio's FPGA image.
The setting names are the AD9361 source's, with the tx_ prefix where a transmitter carries the thing, so one caller drives both directions identically.
A sample tagged gr:tx_eob = true ends a burst: the radio plays it out, frees its buffer and powers the transmit LO down, and the next samples start the next burst.
gr:tx_sob asks for nothing more. The block ignores gr:tx_time, since the radio has no timed transmit, and reports the first such tag of a run on its message port.)">;

    gr::PortIn<std::complex<float>>               in;
    gr::PortIn<std::complex<float>, gr::Optional> in2;

    static constexpr const char*      kDevice             = "ad9361"; // the family name a device listing and a probe carry
    static constexpr bool             kListingOpens       = true;
    static constexpr std::string_view kTxEob              = gr::tag::TX_EOB.key();
    static constexpr std::string_view kTxTime             = gr::tag::TX_TIME.key();
    static constexpr double           kDefaultRateHz      = 5'000'000.0;
    static constexpr double           kDefaultFrequencyHz = 2'400'000'000.0;
#ifdef CAPTURE_BLOCK_LABELS
    static constexpr auto attributes = gr::block::describe(1U, gr::block::labels::family(kDevice, "AD9361 radios driven through the Linux ad9361 driver and libiio."),
        gr::block::labels::role::sink, gr::block::labels::holds::device, gr::block::labels::emits::rf);
#endif

    gr::Annotated<std::string, "device", gr::Visible, gr::Doc<"informational; \"ad9361\"">>                              device = std::string(kDevice);
    gr::Annotated<std::string, "device_parameter", gr::Visible, gr::Doc<"kwargs; uri= or ip=, and image=adi or image=rwt">> device_parameter;
    gr::Annotated<double, "sample_rate", gr::Unit<"Hz">, gr::Visible, gr::Doc<"transmit sample rate">>                   sample_rate = kDefaultRateHz;
    gr::Annotated<std::vector<double>, "frequency", gr::Unit<"Hz">, gr::Visible, gr::Doc<"transmit LO frequency">>       frequency   = std::vector<double>{kDefaultFrequencyHz};
    gr::Annotated<std::vector<double>, "tx_gains", gr::Unit<"dB">, gr::Visible, gr::Doc<"transmit gain, -89.75 to 0">>   tx_gains    = std::vector<double>{-30.0};
    gr::Annotated<double, "frequency_correction", gr::Unit<"ppm">, gr::Doc<"reference error applied to the LO">>        frequency_correction = 0.0;
    gr::Annotated<std::vector<std::string>, "tx_antennae", gr::Doc<"AD9361 output, A or B">>                             tx_antennae;
    gr::Annotated<std::vector<gr::Size_t>, "channels", gr::Doc<"device channels fed by in and in2">>                     channels    = std::vector<gr::Size_t>{0U};
    gr::Annotated<double, "bandwidth", gr::Unit<"Hz">, gr::Doc<"analog filter bandwidth; 0 follows the rate">>          bandwidth   = 0.0;
    gr::Annotated<gr::Size_t, "buffer_size", gr::Doc<"samples per transfer; 0 derives it from the rate">>               buffer_size = 0U;

    GR_MAKE_REFLECTABLE(Sink, in, in2, device, device_parameter, sample_rate, frequency, tx_gains, frequency_correction, tx_antennae, channels, bandwidth, buffer_size);

    static constexpr unsigned kTimeoutMs = 2000U;

    /*| frame: the buffers the radio's kernel queues ahead of the DAC: libiio's default of four.
            A burst's end waits for that many transfers to play.
    */
    static constexpr std::size_t kKernelBuffers = 4UZ;

    // ---- state -----------------------------------------------------------------------

    SharedContext               _ctx;   // shared with the family's other blocks on this URI
    std::shared_ptr<RadioRates> _rates; // the rate each block of the process holds the radio at

    iio_device*        _phy   = nullptr;
    iio_device*        _tx    = nullptr;
    iio_channel*       _lo    = nullptr; // ad9361-phy output altvoltage1, the transmit LO
    std::vector<iio_channel*> _phyTx;    // ad9361-phy output voltage<ch>, one per streamed channel
    std::vector<iio_channel*> _scan;     // cf-ad9361-dds-core-lpc scan channels this block enabled
    std::unique_ptr<iio::Stream> _stream; // the drain's while a burst runs
    bool               _firstPush = false; // the drain's: the next push is its burst's first
    std::atomic<bool>  _burstOpen{false}; // _stream is held, for the threads that may not read it
    Packing            _packing  = Packing::Adi;
    std::size_t        _transfer = 0UZ;  // frames per buffer, written by start()
    std::size_t        _nch      = 1UZ;  // channels streamed, written by start()
    mutable std::mutex _ctrlMutex;
    std::mutex         _teardownMutex;
    std::atomic<bool>  _deviceUp{false};
    std::atomic<bool>  _streamDead{false};
    ConsumerThread     _drain;

    double             _appliedRateHz = 0.0; // under _ctrlMutex, cleared by start()
    double             _appliedBwHz   = 0.0;
    std::atomic<double> _radioBwHz{0.0}; // the filter the radio holds after the last write, 0 before one
    long long          _appliedLoCmd  = -1;
    double             _appliedFreqHz = 0.0;
    std::string        _appliedPort;
    double             _appliedGainDb = std::numeric_limits<double>::quiet_NaN();
    AvailableRange     _bwBounds      = kRxBandwidthBounds;
    std::atomic<double> _heldGainDb{-30.0};

    /*| role: the frames staged for the drain, and where the burst that is ending ends.
        invariant: the ring holds frames interleaved, _nch entries a frame, and every write and
            read moves whole frames. _burstEnd is kNoEnd or the ring's head position of the
            frame after the burst's last. While it is set, the work call stages nothing.
    */
    static constexpr std::size_t    kNoEnd = std::numeric_limits<std::size_t>::max();
    std::unique_ptr<SpscRing<CF32>> _ring;
    std::atomic<std::size_t>        _burstEnd{kNoEnd};
    std::atomic<bool>               _flushOwed{false};
    std::vector<CF32>               _chunk; // drain-thread scratch
    TxCounters                      _counters;
    bool                            _burstTagReported = false;

    ControlProperties   _controlProperties{this};
    TeardownGuard<Sink> _teardown{this}; // the last member, so its teardown runs first

    std::optional<gr::Message> controlProperty(std::string_view name, gr::Message message) { return answerControlProperty(*this, name, std::move(message)); }

    std::vector<ControlDesc> controlSurface() { return describeControls({}); }

    // ---- device truth ----------------------------------------------------------------

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
        return {lo.has_value() ? std::pair{lo->min, lo->max} : std::pair{47.0e6, 6.0e9}};
    }

    static std::vector<double> sampleRates() {
        std::vector<double> out;
        for (const double hz : {1.0e6, 2.5e6, 5.0e6, 10.0e6, 20.0e6, 30.72e6, 61.44e6}) {
            if (rateInRange(hz)) {
                out.push_back(hz);
            }
        }
        return out;
    }

    // The transmit gain "TX" over the AD9361's attenuation range, and the rate and frequency.
    static std::vector<ControlDesc> describeControls(const TruthContext&) {
        static constexpr ControlSeed kControls[] = {
            // id, label, unit, kind, min, max, step, def, options, n, list, n, isGain, inopAbove
            {"TX", "TX", "dB", 1, kTxGainMinDb, kTxGainMaxDb, kTxGainStepDb, -30.0, nullptr, 0, nullptr, 0, true, 0.0},
        };
        auto out = controlsFromSeeds(kControls);
        appendRateAndFrequency(out, sampleRates(), kDefaultRateHz, std::pair{kRateFloorHz, kMaxRateHz}, frequencyRanges(), kDefaultFrequencyHz);
        return out;
    }

    static std::vector<std::pair<std::string, std::string>> enumerateDevices() { return listRadios(); }

    bool          deviceUp() const { return _deviceUp.load(std::memory_order_acquire); }
    bool          streamDead() const { return _streamDead.load(std::memory_order_acquire); }
    std::uint64_t underrunEvents() const { return _counters.underrunEvents.load(std::memory_order_relaxed); }
    std::uint64_t discardedAtRateChangeSamples() const { return _counters.discardedAtRateChange.load(std::memory_order_relaxed); }
    std::uint64_t unsentAtStopSamples() const { return _counters.unsentAtStop.load(std::memory_order_relaxed); }
    double        analogBandwidthHz() const { return deviceUp() ? _radioBwHz.load(std::memory_order_acquire) : 0.0; }

    // Frames staged and not yet pushed.
    std::uint64_t queuedSamples() const {
        if (!_ring) {
            return 0U;
        }
        std::size_t tail = 0;
        return _ring->available(tail) / _nch;
    }

    // ---- lifecycle -------------------------------------------------------------------

    void start() {
        _counters.reset();
        _streamDead.store(false, std::memory_order_release);
        _flushOwed.store(false, std::memory_order_relaxed);
        _burstEnd.store(kNoEnd, std::memory_order_relaxed);
        _burstTagReported = false;

        const auto packing = packingFor(device_parameter.value);
        if (!packing.has_value()) {
            throwStartFailure("ad9361::Sink", std::format("image={} names no FPGA image; adi or rwt", kwargsValue(device_parameter.value, "image")));
        }
        _packing = *packing;
        const std::vector<std::size_t> chans(channels->begin(), channels->end());

        std::string error;
        SharedContext ctx = acquireContext(contextUriFor(device_parameter.value), kTimeoutMs, error);
        if (!ctx) {
            throwStartFailure("ad9361::Sink", error);
        }
        iio_device* phy = iio_context_find_device(ctx.get(), kPhyName);
        iio_device* tx  = iio_context_find_device(ctx.get(), kTxName);
        if (phy == nullptr || tx == nullptr) {
            throwStartFailure("ad9361::Sink", std::format("the context holds no {} and {}; the radio has loaded no FPGA image", kPhyName, kTxName));
        }
        const std::size_t deviceChannels = iio_device_find_channel(tx, "voltage2", true) != nullptr && iio_device_find_channel(tx, "voltage3", true) != nullptr ? 2UZ : 1UZ;
        if (const auto refusal = channelsRefusal(chans, deviceChannels, in2.isConnected(), "in2", "transmits"); refusal.has_value()) {
            throwStartFailure("ad9361::Sink", *refusal);
        }
        _nch = chans.size();

        {
            std::lock_guard lock(_ctrlMutex);
            _ctx           = std::move(ctx);
            _rates         = radioRatesFor(_ctx.get());
            _phy           = phy;
            _tx            = tx;
            _lo            = iio_device_find_channel(phy, "altvoltage1", true);
            _phyTx.clear();
            for (const std::size_t ch : chans) {
                _phyTx.push_back(iio_device_find_channel(phy, std::format("voltage{}", ch).c_str(), true));
            }
            _appliedRateHz = 0.0;
            _appliedBwHz   = 0.0;
            _radioBwHz.store(0.0, std::memory_order_relaxed);
            _appliedLoCmd  = -1;
            _appliedFreqHz = 0.0;
            _appliedPort.clear();
            _appliedGainDb = std::numeric_limits<double>::quiet_NaN();
            _bwBounds      = kRxBandwidthBounds;
            if (const auto text = readAttr(iio_device_find_channel(phy, "voltage0", true), "rf_bandwidth_available"); text.has_value()) {
                _bwBounds = parseAvailableRange(*text).value_or(kRxBandwidthBounds);
            }
            if (const auto text = readAttr(_lo, "frequency_available"); text.has_value()) {
                const auto range = parseAvailableRange(*text);
                std::ignore      = probedLoRange(&range);
            }
            /*| why: the ADI DAC core plays its tone generators whenever no buffer feeds it,
                    which is the time between two bursts. Their scale is set to zero.
            */
            for (unsigned i = 0; i < iio_device_get_channels_count(tx); ++i) {
                iio_channel* chn = iio_device_get_channel(tx, i);
                if (iio_channel_is_output(chn) && std::string_view(iio_channel_get_id(chn)).starts_with("altvoltage")) {
                    std::ignore = writeAttr(chn, "scale", "0");
                }
            }
            std::ignore = writeAttr(_lo, "powerdown", "1"); // up for each burst
        }

        if (!gr::lifecycle::isActive(this->state()) && this->state() != gr::lifecycle::State::INITIALISED) {
            closeDevice();
            return;
        }

        if (const auto refusal = applySampleRate(); refusal.has_value()) {
            closeDevice();
            throwStartFailure("ad9361::Sink", *refusal);
        }
        applyBandwidth();
        applyAntenna();
        if (const auto refusal = applyFrequency(); refusal.has_value()) {
            closeDevice();
            throwStartFailure("ad9361::Sink", *refusal);
        }
        applyGain();

        {
            std::lock_guard lock(_ctrlMutex);
            _scan.clear();
            for (const auto& id : scanChannelsFor(chans)) {
                if (iio_channel* chn = iio_device_find_channel(_tx, id.c_str(), true); chn != nullptr) {
                    _scan.push_back(chn);
                }
            }
        }
        _transfer = transferSamplesFor(static_cast<double>(sample_rate.value), buffer_size.value);
        _chunk.assign(_transfer * _nch, CF32{});
        _ring = std::make_unique<SpscRing<CF32>>(std::max(ringSamplesFor(static_cast<double>(sample_rate.value)), 4UZ * _transfer) * _nch);

        _deviceUp.store(true, std::memory_order_release);
        _drain.start([this] {
            nameStreamingThread("ad9361tx", this->name.value);
            drainLoop();
        });
        offerControlProperties(*this);
    }

    void stop() { std::ignore = hardwareTeardown(); }

    /*| contract: play out what is staged within the play-out bound, stop the drain, and leave
            the radio muted: the buffer freed, the attenuation at its most and the LO powered
            down. Answers true to the one caller that released an open radio.
        why: a stopped radio keeps its LO and gain, and an LO left up radiates its leakage.
    */
    bool hardwareTeardown() {
        std::lock_guard teardown(_teardownMutex);
        bool            hadDevice = false;
        {
            std::lock_guard lock(_ctrlMutex);
            hadDevice = _ctx != nullptr;
        }
        _deviceUp.store(false, std::memory_order_release);
        if (hadDevice && _ring) {
            std::size_t tail   = 0;
            const auto  queued = _ring->available(tail);
            if (queued > 0UZ || _burstOpen.load(std::memory_order_acquire)) {
                _burstEnd.store(_ring->head.load(std::memory_order_acquire), std::memory_order_release);
                const auto deadline = std::chrono::steady_clock::now() + playOutMs(queued / _nch + kKernelBuffers * _transfer, _appliedRateHz);
                while (_burstEnd.load(std::memory_order_acquire) != kNoEnd && std::chrono::steady_clock::now() < deadline && !_streamDead.load(std::memory_order_acquire)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                _counters.unsentAtStop.store(_ring->available(tail) / _nch, std::memory_order_relaxed);
            }
        }
        _drain.requestStop();
        _drain.joinIfRunning();
        closeDevice();
        return hadDevice;
    }

    gr::work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!gr::lifecycle::isActive(this->state())) {
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        if (_deviceUp.load(std::memory_order_acquire) && _streamDead.load(std::memory_order_acquire)) {
            this->emitErrorMessage("work()", "the radio stopped taking samples (network or iiod failure?)");
            this->requestStop();
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        return gr::Block<Sink>::work(requestedWork);
    }

    /*| contract: stage what the ring has room for, up to and including the first sample tagged
            tx_eob = true on either input, and consume exactly that from each. With two channels
            the frames take one sample from in and one from in2. A full ring, or a burst still
            ending, consumes nothing, so the scheduler offers the same samples again.
    */
    gr::work::Status processBulk(gr::InputSpanLike auto& input, gr::InputSpanLike auto& input2) noexcept {
        if (input2.isConnected) {
            return stageBulk(input, input2);
        }
        return stageBulk(input);
    }

    gr::work::Status stageBulk(gr::InputSpanLike auto& input, gr::InputSpanLike auto&... second) noexcept {
        if (!_deviceUp.load(std::memory_order_acquire) || !_ring) {
            std::ignore = input.consume(input.size());
            ((std::ignore = second.consume(second.size())), ...);
            return gr::work::Status::OK;
        }
        if (_burstEnd.load(std::memory_order_acquire) != kNoEnd) {
            std::ignore = input.consume(0UZ);
            ((std::ignore = second.consume(0UZ)), ...);
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        const std::size_t want = std::min({input.size(), second.size()...});
        if (want == 0UZ) {
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        std::size_t burst     = want;
        bool        endsBurst = false;
        for (const auto& [length, ends] : {burstLength(input, want), burstLength(second, want)...}) {
            if (ends && length <= burst) {
                burst     = length;
                endsBurst = true;
            }
        }
        std::size_t tail = 0;
        const auto  room = (_ring->buf.size() - _ring->available(tail)) / _nch;
        const auto  take = std::min(burst, room);
        if (take == 0UZ) {
            std::ignore = input.consume(0UZ);
            ((std::ignore = second.consume(0UZ)), ...);
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        const std::array<const CF32*, 1UZ + sizeof...(second)> data{input.data(), second.data()...};
        const std::size_t                                     nch = data.size();
        std::ignore = _ring->writeWith(take * nch, [&data, nch](CF32& slot, std::size_t i) { slot = clipUnit(data[i % nch][i / nch]); });
        reportIgnoredBurstTags(input, take);
        (reportIgnoredBurstTags(second, take), ...);
        std::ignore = input.consume(take);
        ((std::ignore = second.consume(take)), ...);
        if (endsBurst && take == burst) {
            _burstEnd.store(_ring->head.load(std::memory_order_acquire), std::memory_order_release);
        }
        return gr::work::Status::OK;
    }

    // The samples of input up to and including the first tagged tx_eob = true, and true; or want and false.
    std::pair<std::size_t, bool> burstLength(gr::InputSpanLike auto& input, std::size_t want) {
        std::pair<std::size_t, bool> length{want, false};
        for (const auto& [relIndex, tagMap] : input.tags(want)) {
            if (relIndex < 0) {
                continue;
            }
            if (endsBurstIn(tagMap.get()) && static_cast<std::size_t>(relIndex) < length.first) {
                length = {static_cast<std::size_t>(relIndex) + 1UZ, true};
            }
        }
        return length;
    }

    static bool endsBurstIn(const gr::property_map& tagMap) {
        const gr::pmt::Value* value = capture::detail::field(tagMap, kTxEob);
        const bool*           ends  = value == nullptr ? nullptr : value->get_if<bool>();
        return ends != nullptr && *ends;
    }

    /*| contract: the reason a tag is ignored: a tx_time, which this radio cannot transmit at,
            or a tx_eob that is not a bool. Empty for every other tag.
        verified-by: ad9361.sink-names-the-tags-it-ignores
    */
    static std::string ignoredBurstTagReason(const gr::property_map& tagMap, std::size_t streamIndex) {
        if (capture::detail::field(tagMap, kTxTime) != nullptr) {
            return std::format("{} at sample {} is ignored: this radio has no timed transmit", kTxTime, streamIndex);
        }
        if (const gr::pmt::Value* value = capture::detail::field(tagMap, kTxEob); value != nullptr && value->get_if<bool>() == nullptr) {
            return std::format("{} at sample {} is not a bool and is ignored", kTxEob, streamIndex);
        }
        return {};
    }

    void reportIgnoredBurstTags(gr::InputSpanLike auto& input, std::size_t nStaged) {
        if (_burstTagReported) {
            return;
        }
        for (const auto& [relIndex, tagMap] : input.tags(nStaged)) {
            if (relIndex < 0) {
                continue;
            }
            if (auto why = ignoredBurstTagReason(tagMap.get(), input.streamIndex + static_cast<std::size_t>(relIndex)); !why.empty()) {
                _burstTagReported = true;
                this->emitMessage("processBulk()", {{"error", why}});
                return;
            }
        }
    }

    /*| contract: a tx_eob = true one past the last sample, on in or in2, with the end of the
            stream, ends the burst after the last frame staged. Nothing under a stop request:
            the stop ends it.
    */
    gr::work::Status processEpilogue(gr::InputSpanLike auto& input, gr::InputSpanLike auto& input2) noexcept {
        const bool ends = endsAtStreamEnd(in, input.streamIndex) || endsAtStreamEnd(in2, input2.streamIndex);
        if (ends && _ring && _burstEnd.load(std::memory_order_acquire) == kNoEnd) {
            _burstEnd.store(_ring->head.load(std::memory_order_acquire), std::memory_order_release);
        }
        return gr::work::Status::OK;
    }

    template <typename Port>
    bool endsAtStreamEnd(Port& port, std::size_t from) {
        if (gr::lifecycle::isShuttingDown(this->state()) || !_deviceUp.load(std::memory_order_acquire) || !port.isConnected()) {
            return false;
        }
        bool endsBurst = false;
        auto tags      = port.tagReader().get();
        for (const gr::Tag& tag : tags) {
            endsBurst = endsBurst || (tag.index >= from && endsBurstIn(tag.map));
        }
        std::ignore = tags.consume(0UZ);
        return endsBurst;
    }

    // ---- staged settings (the scheduler's thread) ------------------------------------

    void settingsChanged(const gr::property_map& /*oldSettings*/, gr::property_map& newSettings, gr::property_map& /*forwardSettings*/) {
        const ControlSurfaceCheck<Sink> surfaceCheck{*this};
        if (!_deviceUp.load(std::memory_order_acquire)) {
            return;
        }
        if (newSettings.contains("sample_rate")) {
            const double before = _appliedRateHz;
            if (const auto refusal = applySampleRate(); refusal.has_value()) {
                std::fprintf(stderr, "[ad9361::Sink] %s\n", refusal->c_str());
            }
            if (_appliedRateHz != before) {
                _flushOwed.store(true, std::memory_order_release); // queued samples were taken at the old rate
            }
            applyBandwidth();
        }
        if (newSettings.contains("bandwidth")) {
            applyBandwidth();
        }
        if (newSettings.contains("frequency") || newSettings.contains("frequency_correction")) {
            if (const auto refusal = applyFrequency(); refusal.has_value()) {
                std::fprintf(stderr, "[ad9361::Sink] %s\n", refusal->c_str());
            }
        }
        if (newSettings.contains("tx_antennae")) {
            applyAntenna();
        }
        if (newSettings.contains("tx_gains")) {
            applyGain();
        }
    }

    // ---- controls --------------------------------------------------------------------

    bool setElementGain(const std::string& element, double dB) {
        if (element != "TX") {
            return false;
        }
        CtrlGuard guard(_ctrlMutex, _deviceUp);
        if (!guard) {
            return false;
        }
        const double held = std::clamp(dB, kTxGainMinDb, kTxGainMaxDb);
        _heldGainDb.store(held, std::memory_order_relaxed);
        return writeGainLocked(held);
    }

    double elementGain(const std::string& element) const { return element == "TX" ? _heldGainDb.load(std::memory_order_relaxed) : 0.0; }

    bool setControl(const std::string&, double) { return false; }

    // ---- appliers (each takes _ctrlMutex) --------------------------------------------

    std::optional<std::string> applySampleRate() {
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return std::nullopt;
        }
        const double want = static_cast<double>(sample_rate.value);
        if (want == _appliedRateHz) {
            return std::nullopt;
        }
        const auto ran = applyRadioRate(*_rates, this, _phy, true, want, "ad9361::Sink");
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

    /*| contract: write the transmit analog filter, bandwidth where it is positive and the rate
            otherwise, inside the bounds the radio states, and keep the width rf_bandwidth reads
            back.
        verified-by: ad9361.sink-keeps-the-filter-the-radio-holds
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
        iio_channel* chn = iio_device_find_channel(_phy, "voltage0", true);
        if (!writeAttr(chn, "rf_bandwidth", std::format("{:.0f}", hz))) {
            std::fprintf(stderr, "[ad9361::Sink] rf_bandwidth %.0f failed (continuing)\n", hz);
            return;
        }
        _appliedBwHz = hz;
        _radioBwHz.store(heldAfterWrite(readAttr(chn, "rf_bandwidth"), hz), std::memory_order_release);
    }

    /*| contract: tune the transmit LO to the first frequency, corrected for the reference
            error. A request outside the coverage, or one the radio refuses, leaves the LO
            where it is, the setting at this start's last tune, or as asked before the first
            tune of a start, and answers why; start() fails on it.
        trap: before the first tune of a start the radio's LO is wherever an earlier program
            left it. The setting keeps the request and is never set from the radio's read-back,
            so every start of the block refuses a refused request again.
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
        return std::nullopt;
    }

    void applyAntenna() {
        if (tx_antennae->empty() || tx_antennae->front().empty()) {
            return;
        }
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr || tx_antennae->front() == _appliedPort) {
            return;
        }
        const std::string& port = tx_antennae->front();
        iio_channel*       chn  = iio_device_find_channel(_phy, "voltage0", true);
        if (const auto text = readAttr(chn, "rf_port_select_available"); text.has_value()) {
            const auto names = parseAvailableList(*text);
            if (!names.empty() && std::ranges::find(names, port) == names.end()) {
                std::fprintf(stderr, "[ad9361::Sink] the radio has no output %s; it offers %s\n", port.c_str(), text->c_str());
                return;
            }
        }
        if (!writeAttr(chn, "rf_port_select", port)) {
            std::fprintf(stderr, "[ad9361::Sink] rf_port_select %s failed (continuing)\n", port.c_str());
            return;
        }
        _appliedPort = port;
    }

    void applyGain() {
        if (tx_gains->empty()) {
            return;
        }
        std::lock_guard lock(_ctrlMutex);
        if (_phy == nullptr) {
            return;
        }
        const double want = std::clamp(tx_gains->front(), kTxGainMinDb, kTxGainMaxDb);
        if (want != tx_gains->front()) {
            tx_gains = std::vector<double>{want};
        }
        _heldGainDb.store(want, std::memory_order_relaxed);
        if (want != _appliedGainDb) {
            std::ignore = writeGainLocked(want);
        }
    }

    // The caller holds _ctrlMutex with the radio open.
    bool writeGainLocked(double dB) {
        bool ok = true;
        for (iio_channel* chn : _phyTx) {
            ok = writeAttr(chn, "hardwaregain", std::format("{}", dB)) && ok;
        }
        if (!ok) {
            std::fprintf(stderr, "[ad9361::Sink] hardwaregain %.2f failed (continuing)\n", dB);
            return false;
        }
        _appliedGainDb = dB;
        return true;
    }

    // Releases everything start() took, muting the radio on the way. Safe on a partial open.
    void closeDevice() {
        std::lock_guard lock(_ctrlMutex);
        _stream.reset();
        for (iio_channel* chn : _phyTx) {
            std::ignore = writeAttr(chn, "hardwaregain", std::format("{}", kTxGainMinDb));
        }
        if (_lo != nullptr) {
            std::ignore = writeAttr(_lo, "powerdown", "1");
        }
        _scan.clear();
        _phyTx.clear();
        _lo    = nullptr;
        _tx    = nullptr;
        _phy   = nullptr;
        if (_rates) {
            releaseRadioRate(*_rates, this);
            _rates.reset();
        }
        _ctx.reset();
    }

    // ---- the drain -------------------------------------------------------------------

    /*| contract: create the radio's buffer and power the LO up; the burst's first push is
            marked for pushFromRing.
    */
    bool openBurst() {
        std::lock_guard lock(_ctrlMutex);
        std::ignore = writeAttr(_lo, "powerdown", "0");
        int code    = 0;
        _stream     = iio::Stream::open(_tx, _scan, _transfer, true, code);
        if (!_stream) {
            std::fprintf(stderr, "[ad9361::Sink] opening the transmit stream (%zu samples): %s\n", _transfer, iioError(code != 0 ? code : -EIO).c_str());
            return false;
        }
        _burstOpen.store(true, std::memory_order_release);
        _firstPush = true;
        return true;
    }

    // Free the buffer and power the LO down. The radio drops whatever it still held.
    void closeBurst() {
        std::lock_guard lock(_ctrlMutex);
        _stream.reset();
        _burstOpen.store(false, std::memory_order_release);
        std::ignore = writeAttr(_lo, "powerdown", "1");
    }

    /*| contract: pack n ring entries, whole frames, from tail into the buffer, zero behind
            them, push it, and count an underflow the DAC flagged.
        trap: the buffer's memory moves with each push, so its start is read for every one.
        trap: the DAC flags an underflow whenever it holds no samples, and it holds none until
            a burst's first push arrives; that push's flag is cleared, not counted, waiting for
            the lock so a later push cannot count it.
        measured: a Pluto, firmware v0.39, 2.5 MS/s, 64k-sample transfers: the flag
            was set after the first push of every burst, cleared just before it or not, and
            clear after each later push.
    */
    bool pushFromRing(std::size_t tail, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            _chunk[i] = _ring->at(tail + i);
        }
        _ring->advanceTo(tail + n);
        encodeTx(std::span<const CF32>(_chunk.data(), n), _packing, _nch, _stream->data());
        const ssize_t pushed = _stream->push();
        if (pushed < 0) {
            std::fprintf(stderr, "[ad9361::Sink] push: %s\n", iioError(pushed).c_str());
            return false;
        }
        if (std::exchange(_firstPush, false)) {
            std::lock_guard lock(_ctrlMutex);
            std::ignore = takeStatusFlag(_tx, kTxUnderflowFlag);
            return true;
        }
        std::unique_lock lock(_ctrlMutex, std::try_to_lock);
        if (lock.owns_lock() && takeStatusFlag(_tx, kTxUnderflowFlag)) {
            _counters.underrunEvents.fetch_add(1, std::memory_order_relaxed);
        }
        return true;
    }

    /*| trap: the body runs on a thread of this block's own, and an exception leaving it ends
            the process.
    */
    void drainLoop() {
        try {
            drainSteps();
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[ad9361::Sink] the drain stopped: %s\n", e.what());
            _streamDead.store(true, std::memory_order_release);
        } catch (...) {
            std::fprintf(stderr, "[ad9361::Sink] the drain stopped on an unknown exception\n");
            _streamDead.store(true, std::memory_order_release);
        }
    }

    /*| invariant: a buffer goes to the radio whole, so the drain waits for a transfer's worth of
            samples while a burst runs; the radio underflows and says so where the graph falls
            behind. A burst's end goes out short and padded with zeros.
        frame: the end waits for the radio's kernel buffers to play before it frees the buffer,
            because freeing it drops what the radio still holds.
    */
    void drainSteps() {
        while (!_drain.stopRequested()) {
            std::size_t tail  = 0;
            std::size_t avail = _ring->available(tail);
            if (_flushOwed.exchange(false, std::memory_order_acq_rel)) {
                _ring->advanceTo(tail + avail);
                _counters.discardedAtRateChange.fetch_add(avail, std::memory_order_relaxed);
                if (_stream) {
                    closeBurst();
                }
                _burstEnd.store(kNoEnd, std::memory_order_release);
                continue;
            }
            const std::size_t end = _burstEnd.load(std::memory_order_acquire);
            if (!_stream) {
                if (avail == 0UZ) {
                    if (end != kNoEnd) {
                        _burstEnd.store(kNoEnd, std::memory_order_release); // an end with nothing staged
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                if (!openBurst()) {
                    _streamDead.store(true, std::memory_order_release);
                    return;
                }
            }
            const std::size_t transfer = _transfer * _nch;
            const bool        last     = end != kNoEnd && end - tail <= transfer;
            if (!last && avail < transfer) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!pushFromRing(tail, last ? end - tail : transfer)) {
                _streamDead.store(true, std::memory_order_release);
                return;
            }
            if (last) {
                const double rate     = _appliedRateHz > 0.0 ? _appliedRateHz : static_cast<double>(sample_rate.value);
                const auto   deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(static_cast<std::int64_t>(1e6 * static_cast<double>(kKernelBuffers * _transfer) / rate)) + std::chrono::milliseconds(20);
                while (std::chrono::steady_clock::now() < deadline && !_drain.stopRequested()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                closeBurst();
                _burstEnd.store(kNoEnd, std::memory_order_release);
            }
        }
    }
};

} // namespace capture::ad9361
