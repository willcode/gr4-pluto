/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the AD9361 source without a radio: the sample decoder for each FPGA image, the
        ranges, the selector, the surface, and starts that fail against IIO contexts read from
        XML.
    why: its own binary because libiio is the one vendor header the other cases do not need.
*/
#include <capture/ad9361/Sink.hpp>
#include <capture/ad9361/Source.hpp>

#include "support/Cases.hpp"
#include "support/Labels.hpp"
#include "support/Losses.hpp"
#include "support/Surface.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <expected>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <unistd.h>

namespace {

/*| contract: what the calls print on standard error, the descriptor taken aside for the
        duration.
*/
template <typename Calls>
std::string stderrOf(Calls&& calls) {
    std::fflush(stderr);
    std::FILE* capture = std::tmpfile();
    const int  saved   = ::dup(2);
    ::dup2(::fileno(capture), 2);
    calls();
    std::fflush(stderr);
    ::dup2(saved, 2);
    ::close(saved);
    std::rewind(capture);
    std::string text;
    for (int c = std::fgetc(capture); c != EOF; c = std::fgetc(capture)) {
        text.push_back(static_cast<char>(c));
    }
    std::fclose(capture);
    return text;
}

/*| frame: the bytes of 64-bit words as the radio's buffer holds them, little-endian. */
std::vector<std::byte> wordsLe(std::initializer_list<std::uint64_t> words) {
    std::vector<std::byte> out;
    for (const std::uint64_t w : words) {
        for (int b = 0; b < 8; ++b) {
            out.push_back(static_cast<std::byte>((w >> (8 * b)) & 0xFFU));
        }
    }
    return out;
}

constexpr std::uint64_t lanes(std::int16_t l3, std::int16_t l2, std::int16_t l1, std::int16_t l0) {
    const auto u = [](std::int16_t v) { return static_cast<std::uint64_t>(static_cast<std::uint16_t>(v)); };
    return (u(l3) << 48) | (u(l2) << 32) | (u(l1) << 16) | u(l0);
}

constexpr std::uint64_t tagWord(std::uint8_t type, std::uint64_t value, bool more = false) { return (more ? (1ULL << 63) : 0ULL) | (static_cast<std::uint64_t>(type & 0x7FU) << 56) | (value & 0x00FF'FFFF'FFFF'FFFFULL); }

capture::ad9361::CF32 scaled(int i, int q) { return {static_cast<float>(i) / 2048.0f, static_cast<float>(q) / 2048.0f}; }

/*| role: an IIO context description in libiio's XML form, written to a file a case opens
        as xml:<path> and removed when the case ends.
*/
struct XmlContext {
    std::filesystem::path path;

    explicit XmlContext(const std::string& name, const std::string& body) : path(std::filesystem::temp_directory_path() / name) { std::ofstream(path) << body; }
    ~XmlContext() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    std::string selector() const { return "uri=xml:" + path.string(); }
};

/*| contract: ad9361-phy and cf-ad9361-lpc as every image offers them, with receiveChannels
        receivers.
*/
std::string radioXml(int receiveChannels, int transmitChannels = 0, const std::string& serial = {}) {
    std::string rx;
    for (int i = 0; i < 2 * receiveChannels; ++i) {
        rx += std::format(R"(<channel id="voltage{0}" type="input"><scan-element index="{0}" format="le:S12/16&gt;&gt;0" /></channel>)", i);
    }
    std::string tx;
    for (int i = 0; i < 2 * transmitChannels; ++i) {
        tx += std::format(R"(<channel id="voltage{0}" type="output"><scan-element index="{0}" format="le:S16/16&gt;&gt;0" /></channel>)", i);
    }
    if (transmitChannels > 0) {
        tx = R"(<device id="iio:device2" name="cf-ad9361-dds-core-lpc">)" + tx + R"(</device>)";
    }
    const std::string serialAttr = serial.empty() ? std::string{} : std::format(R"(<context-attribute name="hw_serial" value="{}" />)", serial);
    return std::string(R"(<?xml version="1.0" encoding="utf-8"?><context name="xml" description="ad9361 case">)") + serialAttr
         + R"(<device id="iio:device0" name="ad9361-phy">)"
         + R"(<channel id="voltage0" type="input"><attribute name="sampling_frequency" filename="in_voltage_sampling_frequency" /><attribute name="hardwaregain" filename="in_voltage0_hardwaregain" /></channel>)"
         + R"(<channel id="altvoltage0" name="RX_LO" type="output"><attribute name="frequency" filename="out_altvoltage0_RX_LO_frequency" /></channel>)"
         + R"(<channel id="voltage0" type="output"><attribute name="sampling_frequency" filename="out_voltage_sampling_frequency" /><attribute name="hardwaregain" filename="out_voltage0_hardwaregain" /></channel>)"
         + R"(<channel id="altvoltage1" name="TX_LO" type="output"><attribute name="frequency" filename="out_altvoltage1_TX_LO_frequency" /><attribute name="powerdown" filename="out_altvoltage1_TX_LO_powerdown" /></channel>)"
         + R"(</device><device id="iio:device1" name="cf-ad9361-lpc">)" + rx + R"(</device>)" + tx + R"(</context>)";
}

template <typename Block = capture::ad9361::Source, typename Body>
std::string startExpectingFailure(const std::string& selector, std::vector<gr::Size_t> channels, std::string_view endsWith, Body&& more) {
    using namespace boost::ut;
    Block blk{};
    blk.device_parameter = selector;
    blk.channels         = channels;
    std::ignore          = blk.changeStateTo(gr::lifecycle::State::INITIALISED);
    const auto started   = blk.changeStateTo(gr::lifecycle::State::RUNNING);
    expect(!started.has_value()) << "the start throws, and the transition answers the error";
    expect(!started.has_value() && started.error().message.ends_with(endsWith)) << "whose last words are the reason: " << (started.has_value() ? std::string{} : started.error().message);
    expect(blk.state() == gr::lifecycle::State::ERROR) << "and the block is left in ERROR";
    expect(!blk.deviceUp() && blk._ctx == nullptr) << "holding no context";
    expect(!blk.hardwareTeardown()) << "with nothing left for a teardown to release";
    more(blk);
    return started.has_value() ? std::string{} : std::string(started.error().message);
}

using capture::test::Marker;
using capture::test::takeMarkers;

} // namespace

int main(int argc, char** argv) {
    using namespace boost::ut;
    using capture::ad9361::Source;
    using capture::ad9361::StreamEvent;
    using capture::ad9361::Packing;
    using capture::ad9361::SampleDecoder;
    capture::test::Cases cases(argc, argv);

    cases("ad9361.adi-words-follow-the-channel-order", [] {
        // libiio's order: each 16-bit lane in turn from bit 0, I before Q, channel by channel.
        SampleDecoder                     dec;
        std::vector<capture::ad9361::CF32> out;
        std::vector<StreamEvent>          events;
        dec.decode(wordsLe({lanes(-400, 300, -200, 100)}), 1UZ, out, events);
        expect(fatal(out.size() == 2UZ)) << "two samples a word";
        expect(out[0] == scaled(100, -200)) << "I in 15:0 and Q in 31:16 come first";
        expect(out[1] == scaled(300, -400));
        out.clear();
        dec.decode(wordsLe({lanes(5, 6, 7, 8)}), 2UZ, out, events);
        expect(fatal(out.size() == 2UZ)) << "one frame of two channels";
        expect(out[0] == scaled(8, 7)) << "the first channel in lanes 0 and 1";
        expect(out[1] == scaled(6, 5)) << "the second channel in lanes 2 and 3";

        // The ADI images insert no tags, and an escape-valued word is a sample.
        out.clear();
        dec.decode(wordsLe({capture::ad9361::kEscape, 0ULL}), 1UZ, out, events);
        expect(out.size() == 4UZ && events.empty()) << "two sample words";
    });

    cases("ad9361.rwt-one-channel-words-put-the-earlier-sample-high", [] {
        // The RWT packer holds the first sample of a pair and places it in bits 63:32.
        SampleDecoder                     dec{.packing = Packing::Rwt};
        std::vector<capture::ad9361::CF32> out;
        std::vector<StreamEvent>        events;
        dec.decode(wordsLe({lanes(100, -200, 300, -400), lanes(-2048, 2047, 0, 1)}), 1UZ, out, events);
        expect(fatal(out.size() == 4UZ)) << "two samples a word";
        expect(out[0] == scaled(100, -200)) << "I in 63:48 and Q in 47:32 come first";
        expect(out[1] == scaled(300, -400));
        expect(out[2] == scaled(-2048, 2047)) << "12-bit full scale reads as one";
        expect(out[3] == scaled(0, 1));
        expect(events.empty());
    });

    cases("ad9361.rwt-two-channel-words-put-the-first-channel-low", [] {
        SampleDecoder                     dec{.packing = Packing::Rwt};
        std::vector<capture::ad9361::CF32> out;
        std::vector<StreamEvent>        events;
        dec.decode(wordsLe({lanes(5, 6, 7, 8)}), 2UZ, out, events);
        expect(fatal(out.size() == 2UZ)) << "one frame of two channels";
        expect(out[0] == scaled(7, 8)) << "the first channel in bits 31:0";
        expect(out[1] == scaled(5, 6)) << "the second channel in bits 63:32";
    });

    cases("ad9361.a-tag-split-across-buffers-decodes-once", [] {
        using capture::ad9361::kEscape;
        using capture::ad9361::TagType;
        SampleDecoder                     dec{.packing = Packing::Rwt};
        std::vector<capture::ad9361::CF32> out;
        std::vector<StreamEvent>        events;
        dec.decode(wordsLe({lanes(1, 1, 1, 1), kEscape}), 1UZ, out, events);
        expect(out.size() == 2UZ && events.empty()) << "the escape ending a buffer is held, not decoded";
        out.clear();
        dec.decode(wordsLe({tagWord(0x02, 0x123456789ULL), lanes(2, 2, 2, 2)}), 1UZ, out, events);
        expect(fatal(events.size() == 1UZ)) << "the tag in the next buffer is one event";
        expect(events[0] == StreamEvent{.type = static_cast<std::uint8_t>(TagType::Pps), .value = 0x123456789ULL, .frame = 0UZ}) << "ahead of the buffer's first frame";
        expect(out.size() == 2UZ) << "the tag word carries no samples";

        // A tag word with bit 63 set is followed by another tag word, with no escape between.
        out.clear();
        events.clear();
        dec.decode(wordsLe({lanes(3, 3, 3, 3), kEscape, tagWord(0x03, 7, true), tagWord(0x02, 9), lanes(4, 4, 4, 4)}), 1UZ, out, events);
        expect(fatal(events.size() == 2UZ));
        expect(events[0].type == 0x03U && events[0].value == 7U && events[0].frame == 2UZ) << "after the first word's two samples";
        expect(events[1].type == 0x02U && events[1].value == 9U && events[1].frame == 2UZ);
        expect(out.size() == 4UZ);

        // The frame index counts frames, two entries apiece with two channels.
        out.clear();
        events.clear();
        dec.decode(wordsLe({lanes(1, 2, 3, 4), lanes(1, 2, 3, 4), kEscape, tagWord(0x02, 1)}), 2UZ, out, events);
        expect(events.size() == 1UZ && events[0].frame == 2UZ) << "the third frame";
    });

    cases("ad9361.an-escaped-escape-is-a-sample", [] {
        using capture::ad9361::kEscape;
        SampleDecoder                     dec{.packing = Packing::Rwt};
        std::vector<capture::ad9361::CF32> out;
        std::vector<StreamEvent>        events;
        dec.decode(wordsLe({kEscape, 0ULL}), 1UZ, out, events);
        expect(fatal(out.size() == 2UZ)) << "an escape and a zero word stand for one sample word";
        expect(out[0] == scaled(-21846, -21846)) << "every lane of the escape value, outside the ADC's range";
        expect(events.empty());
        expect(!dec.inEscape);
    });

    cases("ad9361.available-ranges-parse", [] {
        using capture::ad9361::parseAvailableRange;
        const auto lo = parseAvailableRange("[70000000 1 6000000000]");
        expect(lo.has_value() && *lo == capture::ad9361::AvailableRange{.min = 70.0e6, .step = 1.0, .max = 6.0e9});
        const auto gain = parseAvailableRange("[-3 1 71]\n");
        expect(gain.has_value() && gain->min == -3.0 && gain->max == 71.0) << "a negative end and a trailing newline";
        expect(!parseAvailableRange("A_BALANCED B_BALANCED").has_value()) << "a list is not a range";
        expect(!parseAvailableRange("[5 1 2]").has_value()) << "an inverted range is refused";
        expect(!parseAvailableRange("[1 2]").has_value());
        const auto ports = capture::ad9361::parseAvailableList("A_BALANCED B_BALANCED C_BALANCED A_N\n");
        expect(ports == std::vector<std::string>{"A_BALANCED", "B_BALANCED", "C_BALANCED", "A_N"});
    });

    cases("ad9361.the-gain-range-follows-the-band", [] {
        using capture::ad9361::rxGainRangeFor;
        expect(rxGainRangeFor(100.0e6).max == 73.0 && rxGainRangeFor(100.0e6).min == -1.0);
        expect(rxGainRangeFor(2.4e9).max == 71.0 && rxGainRangeFor(2.4e9).min == -3.0);
        expect(rxGainRangeFor(5.8e9).max == 62.0 && rxGainRangeFor(5.8e9).min == -10.0);
        const auto surface = Source::describeControls(capture::TruthContext{.deviceParams = {}, .sampleRate = 5.0e6, .centerFreq = 5.8e9});
        const auto rx      = capture::test::descriptorOf(surface, "RX");
        expect(rx.has_value() && rx->isGain && rx->max == 62.0 && rx->defValue <= 62.0) << "the surface states the band's range";
    });

    cases("ad9361.the-bandwidth-follows-the-rate", [] {
        using capture::ad9361::rxBandwidthFor;
        expect(rxBandwidthFor(0.0, 5.0e6) == 5.0e6) << "0 follows the rate";
        expect(rxBandwidthFor(1.0e6, 5.0e6) == 1.0e6) << "a request holds";
        expect(rxBandwidthFor(0.0, 100.0e3) == 200.0e3) << "the filter's floor";
        expect(rxBandwidthFor(60.0e6, 61.44e6) == 56.0e6) << "the filter's ceiling";
        const capture::ad9361::AvailableRange ad9363{.min = 200.0e3, .step = 1.0, .max = 20.0e6};
        expect(rxBandwidthFor(0.0, 30.72e6, ad9363) == 20.0e6) << "the ceiling an AD9363 states";
    });

    cases("ad9361.a-transfer-holds-whole-words", [] {
        using capture::ad9361::transferSamplesFor;
        expect(transferSamplesFor(5.0e6, 0UZ) == 100000UZ) << "a fiftieth of a second";
        expect(transferSamplesFor(1.0e6, 0UZ) == 20000UZ);
        expect(transferSamplesFor(0.0, 0UZ) == 16384UZ) << "the floor";
        expect(transferSamplesFor(5.0e6, 1001UZ) == 1004UZ) << "a request rounds up to whole words";
        for (const double hz : {520834.0, 2083334.0, 30.72e6, 61.44e6}) {
            expect(transferSamplesFor(hz, 0UZ) % 4UZ == 0UZ) << hz;
        }
    });

    cases("ad9361.the-selector-names-a-context", [] {
        using capture::ad9361::contextUriFor;
        expect(contextUriFor("uri=ip:192.168.2.1") == "ip:192.168.2.1");
        expect(contextUriFor("driver=ad9361,ip=10.0.0.2") == "ip:10.0.0.2");
        expect(contextUriFor("uri=usb:1.2.3,ip=10.0.0.2") == "usb:1.2.3") << "uri wins over ip";
        expect(contextUriFor("driver=ad9361").empty()) << "nothing named opens the default context";
        expect(contextUriFor("").empty());

        using capture::ad9361::packingFor;
        expect(packingFor("uri=ip:192.168.2.1") == Packing::Adi) << "no image names the ADI packing";
        expect(packingFor("image=adi") == Packing::Adi);
        expect(packingFor("ip=10.0.0.2,image=rwt") == Packing::Rwt);
        expect(!packingFor("image=oxygen").has_value()) << "an image this family does not know is refused";
    });

    cases("ad9361.a-channel-list-is-checked", [] {
        using capture::ad9361::channelsRefusal;
        const std::vector<std::size_t> zero{0UZ}, one{1UZ}, both{0UZ, 1UZ}, twice{0UZ, 0UZ}, three{0UZ, 1UZ, 0UZ}, none{};
        expect(!channelsRefusal(zero, 2UZ, false).has_value());
        expect(!channelsRefusal(both, 2UZ, true).has_value());
        expect(channelsRefusal(none, 2UZ, false).has_value());
        expect(channelsRefusal(three, 2UZ, false).has_value());
        expect(channelsRefusal(twice, 2UZ, false).has_value());
        expect(channelsRefusal(one, 1UZ, false).has_value()) << "a one-receiver radio has no channel 1";
        expect(channelsRefusal(zero, 2UZ, true).has_value()) << "out2 connected with one channel listed";
        expect(!channelsRefusal(both, 2UZ, false).has_value()) << "a source may leave out2 unconnected";
        expect(channelsRefusal(both, 2UZ, false, "in2", "transmits") == std::optional<std::string>("channels lists two device channels, and in2 is not connected")) << "a sink may not leave in2 unconnected";
        expect(capture::ad9361::scanChannelsFor(one) == std::vector<std::string>{"voltage2", "voltage3"});
        expect(capture::ad9361::scanChannelsFor(both) == std::vector<std::string>{"voltage0", "voltage1", "voltage2", "voltage3"});
    });

    cases("ad9361.rates-follow-the-build", [] {
        using namespace capture::ad9361;
        expect(kRateFloorHz == (kHaveAd9361 ? kMinRateWithFirHz : kMinRateHz)) << "libad9361 decides the floor";
        expect(rateInRange(kRateFloorHz) && rateInRange(kMaxRateHz));
        expect(!rateInRange(kRateFloorHz - 1.0) && !rateInRange(kMaxRateHz + 1.0));
        expect(!rateInRange(5.0e6 + 0.5)) << "a whole number of samples per second";
        expect(rateToWrite(5.0e6 + 0.4) == 5.0e6 && rateToWrite(1.0e9) == kMaxRateHz && rateToWrite(1.0) == kRateFloorHz) << "a request is written as whole hertz, outside the range at the nearer end";
        expect(rateToWrite(std::numeric_limits<double>::quiet_NaN()) == kRateFloorHz && rateToWrite(-1.0e9) == kRateFloorHz) << "and one that is not a number at the floor";
        expect(sameRate(2499999.0, 2.5e6) && sameRate(61.44e6, 61.44e6)) << "the driver's read-back of the rate asked for";
        expect(!sameRate(2.4e6, 2.5e6) && !sameRate(61.43e6, 61.44e6));
        const auto ladder = Source::sampleRates();
        expect(!ladder.empty() && std::ranges::all_of(ladder, rateInRange)) << "the ladder lies inside the range";
    });

    cases("ad9361.an-unconnected-output-ends-the-block", [] {
        Source src{};
        expect(!src.out.isConnected()) << "a block standing on its own reaches nothing";
        expect(capture::stopWhenOutputIsUnconnected(src)) << "so the work preamble answers that this call cannot go on";
        expect(src.state() == gr::lifecycle::State::REQUESTED_STOP) << "and the block is asked to stop rather than spinning";
        expect(src.droppedTags() == 0UZ && src.droppedSamples() == 0U && src.radioOverflows() == 0U) << "with nothing counted";
    });

    cases("ad9361.a-context-that-cannot-open-fails-the-start", [] {
        const auto missing = std::filesystem::temp_directory_path() / "capture-ad9361-no-such-context.xml";
        const std::string why = startExpectingFailure("uri=xml:" + missing.string(), {0U}, "", [](Source&) {});
        expect(why.contains("xml:" + missing.string() + ": ")) << "the reason names the URI, then libiio's text: " << why;
        startExpectingFailure("image=oxygen", {0U}, "image=oxygen names no FPGA image; adi or rwt", [](Source&) {});
    });

    cases("ad9361.a-context-without-the-radio-fails-the-start", [] {
        const XmlContext other("capture-ad9361-other.xml", R"(<?xml version="1.0" encoding="utf-8"?><context name="xml" description="other"><device id="iio:device0" name="xadc"><channel id="temp0" type="input" /></device></context>)");
        startExpectingFailure(other.selector(), {0U}, "(on the radio: fpgaloader switch default)", [](Source&) {});

        // A one-receiver radio, such as an AD9364 image, offers no channel 1.
        const XmlContext single("capture-ad9361-single.xml", radioXml(1));
        startExpectingFailure(single.selector(), {1U}, "channels lists device channel 1, and the radio receives on 1", [](Source&) {});

        // A context that takes no writes reaches the rate and refuses there, releasing it.
        const XmlContext dual("capture-ad9361-dual.xml", radioXml(2));
        startExpectingFailure(dual.selector(), {0U, 1U}, "the radio refused a rate of 5000000 S/s", [](Source& blk) { expect(blk._stream == nullptr && blk._phy == nullptr && blk._rates == nullptr) << "the handles are cleared"; });
    });

    cases("ad9361.a-pulse-names-the-nearest-second", [] {
        constexpr std::uint64_t kSecond = 1'000'000'000ULL;
        expect(Source::ppsSecondNs(1700000000ULL * kSecond) == 1700000000ULL * kSecond) << "a pulse on the second";
        expect(Source::ppsSecondNs(1700000000ULL * kSecond + 499'000'000ULL) == 1700000000ULL * kSecond) << "the host clock late by under half a second";
        expect(Source::ppsSecondNs(1700000000ULL * kSecond - 499'000'000ULL) == 1700000000ULL * kSecond) << "the host clock early by under half a second";
        expect(Source::ppsSecondNs(1700000000ULL * kSecond + 501'000'000ULL) == 1700000001ULL * kSecond) << "past half a second names the next";
    });

    cases("ad9361.a-radio-has-one-key", [] {
        using capture::ad9361::canonicalUri;
        const std::vector<std::string> one{"usb:1.78.5"};
        const std::vector<std::string> two{"usb:1.78.5", "usb:3.12.5"};
        const auto lookup = [](const std::string& host) -> std::optional<std::string> {
            if (host == "pluto.local") {
                return std::string("192.168.22.1");
            }
            return std::nullopt;
        };
        expect(canonicalUri("usb:", one, lookup) == "usb:1.78.5") << "the one radio attached";
        expect(canonicalUri("usb:", two, lookup) == "usb:") << "two radios: no guess";
        expect(canonicalUri("usb:1.78", two, lookup) == "usb:1.78.5") << "a bus and address name one of two";
        expect(canonicalUri("usb:1.7", two, lookup) == "usb:1.7") << "a prefix that is not a whole field names none";
        expect(canonicalUri("usb:1.78.5", two, lookup) == "usb:1.78.5");
        expect(canonicalUri("ip:pluto.local", one, lookup) == "ip:192.168.22.1") << "a host name becomes its address";
        expect(canonicalUri("ip:10.0.0.9", one, lookup) == "ip:10.0.0.9") << "an address the lookup does not know stays";
        expect(canonicalUri("xml:/tmp/a.xml", one, lookup) == "xml:/tmp/a.xml");
        expect(canonicalUri("", one, lookup).empty()) << "the default context";
    });

    cases("ad9361.one-context-per-radio", [] {
        // Two URIs that reach radios stating one hw_serial share a context; another serial does not.
        using capture::ad9361::acquireContext;
        const XmlContext a("capture-ad9361-serial-a.xml", radioXml(1, 1, "104473dc59930007"));
        const XmlContext b("capture-ad9361-serial-b.xml", radioXml(1, 1, "104473dc59930007"));
        const XmlContext c("capture-ad9361-serial-c.xml", radioXml(1, 1, "ffff00000000"));
        std::string      error;
        auto             first  = acquireContext("xml:" + a.path.string(), 1000U, error);
        auto             second = acquireContext("xml:" + b.path.string(), 1000U, error);
        auto             third  = acquireContext("xml:" + c.path.string(), 1000U, error);
        expect(fatal(first != nullptr && second != nullptr && third != nullptr));
        expect(first.get() == second.get()) << "one radio by its serial, whichever URI reached it";
        expect(first.get() != third.get()) << "another serial is another radio";
        expect(first.use_count() == 2) << "the second open was dropped, not kept";
    });

    cases("ad9361.one-context-per-uri", [] {
        using capture::ad9361::acquireContext;
        const XmlContext radio("capture-ad9361-shared.xml", radioXml(1, 1));
        const XmlContext other("capture-ad9361-shared-other.xml", radioXml(1, 1));
        const std::string uri = "xml:" + radio.path.string();
        std::string       error;
        auto              first  = acquireContext(uri, 1000U, error);
        auto              second = acquireContext(uri, 1000U, error);
        auto              third  = acquireContext("xml:" + other.path.string(), 1000U, error);
        expect(fatal(first != nullptr && second != nullptr && third != nullptr)) << "each URI opens";
        expect(first.get() == second.get()) << "a second block on one URI shares the first block's context";
        expect(first.get() != third.get()) << "another URI has a context of its own";
        first.reset();
        expect(second.use_count() == 1) << "the context outlives the block that opened it";
        second.reset();
        auto again = acquireContext(uri, 1000U, error);
        expect(again != nullptr && again.use_count() == 1) << "the last release destroys it, and the next block opens a new one";
        expect(acquireContext("xml:/nonexistent/capture-ad9361.xml", 1000U, error) == nullptr && !error.empty()) << "a failed open answers no handle and the reason";
    });

    cases("ad9361.a-second-rate-on-one-radio-is-refused", [] {
        using namespace capture::ad9361;
        const XmlContext radio("capture-ad9361-rates.xml", radioXml(1, 1));
        std::string      error;
        auto             ctx = acquireContext("xml:" + radio.path.string(), 1000U, error);
        expect(fatal(ctx != nullptr)) << error;
        auto rates = radioRatesFor(ctx.get());
        expect(radioRatesFor(ctx.get()) == rates) << "the blocks of one radio share one record";
        iio_device* phy    = iio_context_find_device(ctx.get(), "ad9361-phy");
        const int   source = 0;
        const int   sink   = 0;
        rates->held[&source] = 2.5e6; // a source streams the radio at 2.5 MS/s

        const auto other = applyRadioRate(*rates, &sink, phy, true, 5.0e6, "case");
        expect(!other.has_value() && other.error() == "sample_rate asked 5000000 S/s, and another block in this process streams this radio at 2500000 S/s; the AD9361 runs one sample clock for both directions") << "the later block is refused, naming both rates, before anything is written: " << (other.has_value() ? std::string{} : other.error());
        std::expected<double, std::string> past;
        const std::string                  line = stderrOf([&] { past = applyRadioRate(*rates, &sink, phy, true, 1.0e9, "case"); });
        expect(!past.has_value() && past.error().starts_with("sample_rate asked 1000000000 S/s,")) << "the refusal names the request, not the rate it snaps to";
        expect(line.starts_with("[case] rate 1000000000.0 snapped to 61440000,")) << "the snap is named on standard error: " << line;
        expect(stderrOf([&] { std::ignore = applyRadioRate(*rates, &sink, phy, true, 2.5e6 + 0.4, "case"); }).find("snapped") == std::string::npos) << "a move of half a hertz or less is not named";
        const auto same = applyRadioRate(*rates, &sink, phy, true, 2.5e6, "case");
        expect(!same.has_value() && same.error() == "the radio refused a rate of 2500000 S/s") << "the rate in force passes the check and reaches the radio, which takes no writes here";
        expect(!rates->held.contains(&sink)) << "a refused block holds no rate";

        releaseRadioRate(*rates, &source);
        const auto alone = applyRadioRate(*rates, &sink, phy, true, 5.0e6, "case");
        expect(!alone.has_value() && alone.error() == "the radio refused a rate of 5000000 S/s") << "once the source lets go, any rate reaches the radio";
        const auto above = applyRadioRate(*rates, &sink, phy, true, 1.0e9, "case");
        expect(!above.has_value() && above.error() == "the radio refused a rate of 61440000 S/s") << "a rate past the top reaches the radio as the top";
    });

    cases("ad9361.declares-its-labels", [] {
        capture::test::expectLabels<Source>({"family/ad9361", "role/source", "holds/device", "ingests/rf"});
        capture::test::expectLabels<capture::ad9361::Sink>({"family/ad9361", "role/sink", "holds/device", "emits/rf"});
    });

    cases("ad9361.transmit-words-mirror-the-receive-words", [] {
        using capture::ad9361::encodeTx;
        const std::vector<capture::ad9361::CF32> samples{{0.5f, -0.25f}, {2.0f, -1.0f}, {std::numeric_limits<float>::quiet_NaN(), 0.125f}};
        std::vector<std::byte>                   adi(3 * 8), rwt(3 * 8);
        encodeTx(samples, Packing::Adi, 1UZ, adi);
        encodeTx(samples, Packing::Rwt, 1UZ, rwt);
        // 0.5 and -0.25 of 32767, rounded; 2.0 clips to full scale; NaN sends zero; the odd sample's partner is zero.
        expect(adi == wordsLe({lanes(-32767, 32767, -8192, 16384), lanes(0, 0, 4096, 0), 0ULL})) << "ADI: the earlier sample low, I before Q";
        expect(rwt == wordsLe({lanes(16384, -8192, 32767, -32767), lanes(0, 4096, 0, 0), 0ULL})) << "RWT: the earlier sample high, I above Q";

        // Two channels: one frame a word, first channel then second.
        std::vector<std::byte> adi2(2 * 8), rwt2(2 * 8);
        encodeTx(samples, Packing::Adi, 2UZ, adi2);
        encodeTx(samples, Packing::Rwt, 2UZ, rwt2);
        expect(adi2 == wordsLe({lanes(-32767, 32767, -8192, 16384), lanes(0, 0, 4096, 0)})) << "ADI: the first channel in lanes 0 and 1";
        expect(rwt2 == wordsLe({lanes(32767, -32767, 16384, -8192), lanes(0, 0, 0, 4096)})) << "RWT: the first channel in bits 31:0";

        // Each layout is the receive decoder's, so a round trip at 12-bit scale gives the frames back.
        for (const auto packing : {Packing::Adi, Packing::Rwt}) {
            for (const std::size_t channels : {1UZ, 2UZ}) {
                const std::vector<capture::ad9361::CF32> sent{{0.5f, -0.25f}, {0.125f, 0.75f}, {-0.5f, 0.0f}, {0.0625f, -1.0f}};
                std::vector<std::byte>                   words(2 * 8); // two entries a word either way
                encodeTx(sent, packing, channels, words);
                // The DAC's 16-bit lanes read as the ADC's 12-bit ones are 16 times larger.
                SampleDecoder                      dec{.packing = packing};
                std::vector<capture::ad9361::CF32> got;
                std::vector<StreamEvent>           events;
                dec.decode(words, channels, got, events);
                bool same = got.size() == sent.size();
                for (std::size_t i = 0; same && i < sent.size(); ++i) {
                    same = std::abs(got[i] / 16.0f - sent[i]) < 1e-3f;
                }
                expect(same) << (packing == Packing::Adi ? "ADI" : "RWT") << " with " << channels << " channels decodes back to what was packed";
            }
        }
    });

    cases("ad9361.sink-names-the-tags-it-ignores", [] {
        using capture::ad9361::Sink;
        gr::property_map timed{{std::pmr::string(gr::tag::TX_TIME.key()), std::uint64_t{1}}};
        gr::property_map eobInt{{std::pmr::string(gr::tag::TX_EOB.key()), 1}};
        gr::property_map eob{{std::pmr::string(gr::tag::TX_EOB.key()), true}};
        gr::property_map sob{{std::pmr::string(gr::tag::TX_SOB.key()), true}};
        expect(Sink::ignoredBurstTagReason(timed, 7UZ).ends_with("this radio has no timed transmit")) << "a time the radio cannot transmit at";
        expect(Sink::ignoredBurstTagReason(eobInt, 7UZ).ends_with("is not a bool and is ignored"));
        expect(Sink::ignoredBurstTagReason(eob, 7UZ).empty() && Sink::endsBurstIn(eob)) << "a burst end is taken";
        expect(Sink::ignoredBurstTagReason(sob, 7UZ).empty() && !Sink::endsBurstIn(sob)) << "a burst start asks for nothing";
    });

    cases("ad9361.sink-states-its-transmit-range", [] {
        using capture::ad9361::Sink;
        const auto surface = Sink::describeControls({});
        const auto tx      = capture::test::descriptorOf(surface, "TX");
        expect(tx.has_value() && tx->isGain && tx->min == -89.75 && tx->max == 0.0 && tx->step == 0.25) << "the attenuation stated as a gain";
        capture::test::expectRateAndFrequency(surface, Sink::sampleRates(), std::pair{capture::ad9361::kRateFloorHz, capture::ad9361::kMaxRateHz}, Sink::frequencyRanges(), "an AD9361 transmitter");
    });

    cases("ad9361.sink-refuses-a-start-it-cannot-make", [] {
        using capture::ad9361::Sink;
        startExpectingFailure<Sink>("image=oxygen", {0U}, "image=oxygen names no FPGA image; adi or rwt", [](Sink&) {});
        const XmlContext receiveOnly("capture-ad9361-rx-only.xml", radioXml(1));
        startExpectingFailure<Sink>(receiveOnly.selector(), {0U}, "the radio has loaded no FPGA image", [](Sink&) {});
        const XmlContext single("capture-ad9361-tx-single.xml", radioXml(1, 1));
        startExpectingFailure<Sink>(single.selector(), {1U}, "channels lists device channel 1, and the radio transmits on 1", [](Sink&) {});
        const XmlContext dual("capture-ad9361-tx-dual.xml", radioXml(2, 2));
        startExpectingFailure<Sink>(dual.selector(), {0U, 1U}, "channels lists two device channels, and in2 is not connected", [](Sink&) {});
        startExpectingFailure<Sink>(dual.selector(), {0U, 0U}, "channels lists device channel 0 twice", [](Sink&) {});
        startExpectingFailure<Sink>(dual.selector(), {1U}, "the radio refused a rate of 5000000 S/s", [](Sink& blk) { expect(blk._stream == nullptr && blk._phy == nullptr && blk._rates == nullptr) << "the handles are cleared"; });
    });

    cases("ad9361.states-its-rate-and-frequency", [] {
        const capture::TruthContext ctx{.deviceParams = {}, .sampleRate = 5.0e6, .centerFreq = 2.4e9};
        capture::test::expectRateAndFrequency(Source::describeControls(ctx), Source::sampleRates(), std::pair{capture::ad9361::kRateFloorHz, capture::ad9361::kMaxRateHz}, Source::frequencyRanges(), "an AD9361 radio");
    });

    cases("ad9361.states-its-gain-mode-and-its-stream-state", [] {
        capture::test::expectGainMode<Source>(true, "an AD9361 radio, whose AD9361 carries an AGC");
        Source blk{};
        capture::test::expectStreamStateAlone(blk, capture::stream_state::kStopped, "an unstarted AD9361 source");
        blk.device_parameter = std::string("uri=xml:/nonexistent/capture-rwt.xml");
        std::ignore          = blk.changeStateTo(gr::lifecycle::State::INITIALISED);
        std::ignore          = blk.changeStateTo(gr::lifecycle::State::RUNNING);
        capture::test::expectStreamStateAlone(blk, capture::stream_state::kFailed, "an AD9361 source whose start threw");
    });

    cases("ad9361.a-refused-buffer-is-counted-in-its-marker", [] {
        // Two channels: a count in ring elements reads twice the frames lost.
        using capture::ad9361::CF32;
        Source                          src{};
        gr::PortIn<std::complex<float>> reader;
        expect(fatal(src.out.connect(reader).has_value())) << "the marker reaches a reader";
        constexpr std::size_t kChannels   = 2UZ;
        constexpr std::size_t kRingFrames = 64UZ;
        src._nch                          = kChannels;
        src._ring                         = std::make_unique<capture::SpscRing<CF32>>(kRingFrames * kChannels);

        constexpr std::size_t kHeldFrames = kRingFrames - 8UZ;
        src._decoded.assign(kHeldFrames * kChannels, CF32{1.0f, 0.0f});
        src.stageDecoded(false);
        expect(src.droppedSamples() == 0U && src._pending.empty()) << "a buffer the ring takes is neither counted nor marked";

        constexpr std::size_t kRefusedFrames = kRingFrames - kHeldFrames + 1UZ; // one frame past the room left
        src._decoded.assign(kRefusedFrames * kChannels, CF32{2.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(false);
        expect(src.droppedSamples() == kRefusedFrames) << "the refused buffer is counted whole, in frames";
        expect(fatal(src._pending.size() == 1UZ)) << "and one gap is queued for it";
        expect(src._pending.front().type == Source::kHostGapEvent && src._pending.front().position == kHeldFrames * kChannels) << "where the buffer would have begun, behind the frames the ring holds";

        // The ring still has room for 8 frames. The next buffer is longer and falls at the same position.
        constexpr std::size_t kNextFrames = 2UZ * kRefusedFrames;
        src._decoded.assign(kNextFrames * kChannels, CF32{3.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(false);
        expect(src.droppedSamples() == kRefusedFrames + kNextFrames) << "a second refused buffer adds its frames";
        expect(fatal(src._pending.size() == 1UZ)) << "to the gap already queued there, not to a second one";
        expect(src._pending.front().value == kRefusedFrames + kNextFrames) << "which holds both buffers' frames";

        // A radio FIFO overflow read with the next refused buffer queues its event at the same
        // position. A reader merging the sample's tags in order keeps the cause queued last.
        constexpr std::size_t kLastFrames = kRefusedFrames;
        src._decoded.assign(kLastFrames * kChannels, CF32{4.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(true);
        constexpr std::size_t kLost = kRefusedFrames + kNextFrames + kLastFrames;
        expect(fatal(src._pending.size() == 2UZ)) << "the overflow and one gap";
        expect(src._pending.front().type == static_cast<std::uint8_t>(capture::ad9361::TagType::Overflow) && src._pending.back().type == Source::kHostGapEvent) << "the gap moved behind the overflow: the host ring's cause comes last";
        const Source::Pending gap = src._pending.back();
        expect(gap.value == kLost && gap.position == src._pending.front().position) << "holding every frame refused at that position";

        src.placeEvent(gap, 0U);
        const Marker marker = takeMarkers(reader);
        expect(marker.tags == 1UZ) << "the gap publishes one marker";
        expect(marker.overflow && marker.cause == "host ring full") << "gr:rx_overflow with its cause";
        expect(marker.count == std::optional<gr::Size_t>(static_cast<gr::Size_t>(kLost))) << "and every frame the ring refused there beside them";

        // Once the consumer drains the ring, a later refusal falls at a later position and is a
        // gap of its own, with its own count.
        src._ring->advanceTo(src._ring->head.load());
        src._decoded.assign(kHeldFrames * kChannels, CF32{5.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(false);
        const std::size_t laterBase = src._ring->head.load();
        src._decoded.assign(kRefusedFrames * kChannels, CF32{6.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(false);
        expect(fatal(src._pending.size() == 3UZ)) << "a second gap beside the first";
        expect(src._pending.back().type == Source::kHostGapEvent && src._pending.back().position == laterBase) << "at the position the later buffer would have begun";
        expect(src._pending.back().value == kRefusedFrames && src._pending[1].value == kLost) << "each gap with its own count";
    });

    cases("ad9361.the-filter-stated-is-the-one-the-radio-holds", [] {
        using capture::ad9361::heldAfterWrite;
        constexpr double kWritten = 56.0e6;
        constexpr double kHeld    = 20.0e6; // the top of an AD9363's filter, below the one written
        expect(heldAfterWrite(std::format("{:.0f}\n", kHeld), kWritten) == kHeld) << "the width the radio reads back, where it holds another";
        expect(heldAfterWrite(std::nullopt, kWritten) == kWritten) << "the width written, where the read fails";
        expect(heldAfterWrite(std::string("wide"), kWritten) == kWritten && heldAfterWrite(std::string("0"), kWritten) == kWritten && heldAfterWrite(std::string("nan"), kWritten) == kWritten) << "and where the read-back is no positive number";

        // The radio's state as a landed write leaves it, with no radio behind it.
        Source blk{};
        blk._radioBwHz.store(heldAfterWrite(std::format("{:.0f}", kHeld), kWritten));
        expect(!capture::analogBandwidthIn(capture::test::sensorsAnswer(blk)).has_value()) << "a closed radio states no filter";
        blk._deviceUp.store(true);
        const auto readings = capture::test::sensorsAnswer(blk);
        blk._deviceUp.store(false);
        expect(fatal(readings.size() == 5UZ)) << "an open radio states its stream, its three counts and its filter";
        expect(readings.front().id == capture::kStreamStateReading) << "the stream state first";
        expect(readings.back().id == capture::kAnalogBandwidthReading) << "the filter last";
        expect(capture::analogBandwidthIn(readings) == std::optional<double>(kHeld)) << "the filter the radio holds, not the one written";

        blk._radioBwHz.store(0.0);
        blk._deviceUp.store(true);
        const auto unwritten = capture::test::sensorsAnswer(blk);
        blk._deviceUp.store(false);
        expect(unwritten.size() == 4UZ && !capture::analogBandwidthIn(unwritten).has_value()) << "and no filter before a write has landed";
    });

    cases("ad9361.sink-keeps-the-filter-the-radio-holds", [] {
        // The offline context takes no attribute write. The applier meets a radio that refuses
        // one here. The rule for a write that lands is heldAfterWrite, which the source's case
        // pins.
        using capture::ad9361::Sink;
        const XmlContext radio("capture-ad9361-tx-filter.xml", radioXml(1, 1));
        std::string      error;
        auto             ctx = capture::ad9361::acquireContext("xml:" + radio.path.string(), 1000U, error);
        expect(fatal(ctx != nullptr)) << error;
        Sink blk{};
        expect(blk.analogBandwidthHz() == 0.0) << "an unstarted sink holds no filter";
        blk._phy      = iio_context_find_device(ctx.get(), "ad9361-phy");
        blk.bandwidth = 10.0e6;
        const std::string line = stderrOf([&] { blk.applyBandwidth(); });
        blk._phy               = nullptr;
        expect(line.starts_with("[ad9361::Sink] rf_bandwidth 10000000 failed")) << "the refused write is named on standard error: " << line;
        expect(blk.analogBandwidthHz() == 0.0 && blk._appliedBwHz == 0.0) << "and keeps no width: the next write is tried again";

        // The width a landed write keeps, read by a caller of analogBandwidthHz().
        constexpr double kHeld = 20.0e6;
        blk._radioBwHz.store(kHeld);
        expect(blk.analogBandwidthHz() == 0.0) << "a closed radio states no filter, whatever an earlier run kept";
        blk._deviceUp.store(true);
        const double open = blk.analogBandwidthHz();
        blk._deviceUp.store(false);
        expect(open == kHeld) << "an open one states the width it keeps";
    });

    cases("ad9361.a-rate-change-carries-the-loss-the-ring-refused", [] {
        // Two channels and a 64-frame ring. The ring refuses a buffer and the consumer drains
        // it. The ring then takes frames behind that gap, refuses a buffer among them, and
        // refuses again where they end.
        using capture::ad9361::CF32;
        using capture::ad9361::TagType;
        Source                          src{};
        gr::PortIn<std::complex<float>> reader;
        expect(fatal(src.out.connect(reader).has_value())) << "the marker reaches a reader";
        constexpr std::size_t kChannels   = 2UZ;
        constexpr std::size_t kRingFrames = 64UZ;
        src._nch                          = kChannels;
        src._ring                         = std::make_unique<capture::SpscRing<CF32>>(kRingFrames * kChannels);
        const auto stage                  = [&src](std::size_t frames, std::vector<StreamEvent> events, bool overflowed) {
            src._decoded.assign(frames * kChannels, CF32{1.0f, 0.0f});
            src._events = std::move(events);
            src.stageDecoded(overflowed);
        };
        const auto drainAll = [&src] { src._ring->advanceTo(src._ring->head.load()); };

        constexpr std::size_t kFirstRefused = 5UZ;
        stage(kRingFrames, {}, false);
        stage(kFirstRefused, {}, false);
        drainAll();
        constexpr std::size_t kHeldFrames   = 20UZ;
        constexpr std::size_t kInnerRefused = kRingFrames - kHeldFrames + 6UZ; // more than the room the held frames leave
        stage(kHeldFrames, {}, false);
        stage(kInnerRefused, {}, false);
        stage(kHeldFrames, {}, false);
        // One frame past the room left, read after a radio FIFO overflow, with a pulse inside it.
        constexpr std::size_t kLastRefused = kRingFrames - 2UZ * kHeldFrames + 1UZ;
        const StreamEvent     pulse{.type = static_cast<std::uint8_t>(TagType::Pps), .value = 7U, .frame = 3UZ};
        stage(kLastRefused, {pulse}, true);

        std::size_t       tail  = 0;
        const std::size_t avail = src._ring->available(tail);
        const std::size_t end   = tail + avail;
        expect(fatal(avail == 2UZ * kHeldFrames * kChannels && src._pending.size() == 5UZ)) << "the ring holds the frames it took, and five events wait";
        expect(src._pending[0].position == tail && src._pending[1].position > tail && src._pending[1].position < end && src._pending.back().position == end) << "a gap where the held frames begin, one among them, and the later events where they end";

        src.flushRing(tail, avail);
        constexpr std::size_t kLost = kFirstRefused + 2UZ * kHeldFrames + kInnerRefused + kLastRefused;
        expect(src.droppedSamples() == kLost) << "the flush counts the frames it discarded beside those refused";
        expect(src.radioOverflows() == 1U) << "and the overflow queued where they end";
        expect(fatal(src._pending.size() == 1UZ)) << "no gap and no overflow stays queued to mark that sample again";
        expect(src._pending.front().type == pulse.type && src._pending.front().position == end) << "the pulse stays queued for the first frame after the flush";
        std::size_t after = 0;
        expect(src._ring->available(after) == 0UZ && after == end) << "the ring is empty";
        expect(src.placeDueEvents(after, 0UZ, 0U) == 0UZ && reader.tagReader().available() == 0UZ) << "the marker waits for a block to head";

        constexpr std::size_t kNextFrames = 4UZ;
        stage(kNextFrames, {}, false);
        const std::size_t nextAvail = src._ring->available(after);
        expect(src.placeDueEvents(after, nextAvail, 0U) == nextAvail) << "the next block holds every frame the ring took";
        const Marker marker = takeMarkers(reader);
        expect(marker.tags == 2UZ) << "the marker and the pulse head it";
        expect(marker.markers == 1UZ && marker.overflow && marker.cause == "rate change") << "one marker, gr:rx_overflow with its cause";
        expect(marker.count == std::optional<gr::Size_t>(static_cast<gr::Size_t>(kLost))) << "and every frame lost before that sample";

        // A rate change over an empty ring still marks the gap, with nothing lost.
        drainAll();
        const std::size_t emptyAvail = src._ring->available(tail);
        expect(fatal(emptyAvail == 0UZ && src._pending.empty()));
        src.flushRing(tail, emptyAvail);
        stage(kNextFrames, {}, false);
        std::ignore        = src.placeDueEvents(tail, src._ring->available(tail), 0U);
        const Marker empty = takeMarkers(reader);
        expect(empty.tags == 1UZ && empty.markers == 1UZ && empty.cause == "rate change") << "one marker";
        expect(empty.count == std::optional<gr::Size_t>(0U)) << "whose count is 0";
    });

    cases("ad9361.an-overflow-after-a-rate-change-joins-its-marker", [] {
        // The reader takes the radio's overflow flag with the first buffer after the flush. The
        // overflow falls where the discarded frames end, on the marker's sample.
        using capture::ad9361::CF32;
        Source                          src{};
        gr::PortIn<std::complex<float>> reader;
        expect(fatal(src.out.connect(reader).has_value())) << "the markers reach a reader";
        constexpr std::size_t kChannels   = 2UZ;
        constexpr std::size_t kRingFrames = 64UZ;
        src._nch                          = kChannels;
        src._ring                         = std::make_unique<capture::SpscRing<CF32>>(kRingFrames * kChannels);
        constexpr std::size_t kDiscarded  = 30UZ;
        src._decoded.assign(kDiscarded * kChannels, CF32{1.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(false);
        std::size_t tail  = 0;
        std::size_t avail = src._ring->available(tail);
        src.flushRing(tail, avail);

        constexpr std::size_t kNextFrames = 10UZ;
        src._decoded.assign(kNextFrames * kChannels, CF32{2.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(true);
        avail = src._ring->available(tail);
        expect(fatal(src._pending.size() == 1UZ && src._pending.front().position == tail)) << "the overflow waits at the first frame after the flush";
        std::ignore = src.placeDueEvents(tail, avail, 0U);

        const Marker marker = takeMarkers(reader);
        expect(marker.markers == 1UZ) << "one marker on that sample";
        expect(marker.cause == "rate change" && marker.count == std::optional<gr::Size_t>(static_cast<gr::Size_t>(kDiscarded))) << "the rate change's, with the frames it discarded";
        expect(src.radioOverflows() == 1U && src._pending.empty()) << "and the overflow counted in it";
    });

    cases("ad9361.the-sensors-state-what-the-stream-lost", [] {
        // The overflows and the dropped samples on glance rows, the lost markers on a detail
        // row, and the filter after them.
        using capture::ad9361::CF32;
        Source                          src{};
        gr::PortIn<std::complex<float>> reader;
        expect(fatal(src.out.connect(reader).has_value())) << "the markers reach a reader";
        constexpr std::size_t kRingFrames = 64UZ;
        src._ring                         = std::make_unique<capture::SpscRing<CF32>>(kRingFrames);
        const auto openAnswer             = [&src] {
            src._deviceUp.store(true);
            auto readings = capture::test::sensorsAnswer(src);
            src._deviceUp.store(false);
            return readings;
        };
        const auto expectCounts = [](const std::vector<capture::SensorReading>& readings, std::uint64_t overflows, std::uint64_t dropped, std::size_t markers, std::string_view when) {
            expect(fatal(readings.size() >= 4UZ)) << when << ": the stream state and three counts";
            const auto row = [&](std::size_t i, std::string_view id, std::string_view label, const std::string& value, bool brief) {
                const capture::SensorReading& r = readings[i];
                expect(r.id == id && r.label == label && r.value == value && r.good && r.brief == brief) << when << ": row " << i << " reads " << r.id << ", " << r.label << ", " << r.value;
            };
            row(1UZ, "rx_overflows", "Overflows", std::format("{}", overflows), true);
            row(2UZ, "rx_dropped_samples", "Dropped samples", std::format("{}", dropped), true);
            row(3UZ, "rx_dropped_markers", "Lost markers", std::format("{}", markers), false);
        };
        expectCounts(openAnswer(), 0U, 0U, 0UZ, "before anything is lost");

        // A buffer one frame longer than the ring, read after a radio FIFO overflow. The case
        // sets the port's refused-tag count to three. Each marker the block publishes copies it
        // into droppedTags().
        constexpr std::size_t kRefusedFrames = kRingFrames + 1UZ;
        constexpr std::size_t kRefusedTags   = 3UZ;
        src._decoded.assign(kRefusedFrames, CF32{1.0f, 0.0f});
        src._events.clear();
        src.stageDecoded(true);
        src.out.nTagsDropped = kRefusedTags;
        for (const Source::Pending& e : src._pending) {
            src.placeEvent(e, 0U);
        }
        src._pending.clear();
        auto tags   = reader.tagReader().get();
        std::ignore = tags.consume(tags.size());
        expect(src.radioOverflows() == 1U && src.droppedSamples() == kRefusedFrames && src.droppedTags() == kRefusedTags) << "each counter moved";
        const auto lost = openAnswer();
        expectCounts(lost, 1U, kRefusedFrames, kRefusedTags, "after the losses");
        expect(lost.size() == 4UZ) << "no filter before a write has landed";

        src._radioBwHz.store(20.0e6);
        const auto filtered = openAnswer();
        expect(fatal(filtered.size() == 5UZ)) << "the filter beside the counts";
        expect(filtered.back().id == capture::kAnalogBandwidthReading) << "after them";
        expect(capture::test::sensorsAnswer(src).size() == 1UZ) << "a closed radio states its stream alone";
    });

    return cases.report();
}
