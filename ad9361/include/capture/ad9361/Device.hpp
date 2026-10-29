/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Sink.hpp>

#include <capture/ad9361/Iio.hpp>

#if defined(CAPTURE_AD9361_LIBAD9361)
#include <ad9361.h>
#endif

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>

#include <algorithm>
#include <bit>
#include <cerrno>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <format>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

/*| role: what an AD9361 radio driven by the Linux ad9361 driver states through libiio, apart
        from any block: the sample word of each FPGA image, the RWT in-band tag protocol, the IIO
        names, the rate and gain ranges, and the selector.
    frame: the radio runs Linux on a Zynq beside the AD9361. Its FPGA image puts the IIO devices
        ad9361-phy, cf-ad9361-lpc and cf-ad9361-dds-core-lpc in place, and iiod serves them to
        a host over the network or USB. The ADALM-Pluto and the Red Wire Technologies radios are
        two of them.
    invariant: every image offers the same IIO devices and attributes, and the images differ
        in how the FPGA packs the receive words. A block reads the raw buffer bytes through
        SampleDecoder, with the packing the selector names.
*/
namespace capture::ad9361 {

using CF32 = std::complex<float>;

inline constexpr const char* kPhyName = "ad9361-phy";
inline constexpr const char* kRxName  = "cf-ad9361-lpc";
inline constexpr const char* kTxName  = "cf-ad9361-dds-core-lpc";

/*| contract: the receive full scale. The ADC delivers 12-bit samples sign-extended in each
        16-bit lane, and a part at +/-2048 reads +/-1.0.
*/
inline constexpr float kRxScale = 1.0f / 2048.0f;

/*| role: how an FPGA image packs the receive words.
    frame: Adi is the ADI reference designs' packing, the ADALM-Pluto's among them. Rwt is the
        Red Wire Technologies images' packing, with its in-band tags.
*/
enum class Packing : std::uint8_t { Adi, Rwt };

/*| contract: the packing a device string names with image=adi or image=rwt; adi where it names
        none, and nothing for any other value.
    verified-by: ad9361.the-selector-names-a-context
*/
inline std::optional<Packing> packingFor(const std::string& kwargs) {
    const auto image = capture::kwargsValue(kwargs, "image");
    if (image.empty() || image == "adi") {
        return Packing::Adi;
    }
    if (image == "rwt") {
        return Packing::Rwt;
    }
    return std::nullopt;
}

/*| role: the escape word an RWT FPGA puts in front of an in-band tag, at its reset value.
    trap: the escape value sits in a register of the RWT FPGA's own block, which the radio maps
        into its own address space and iiod does not serve. A host cannot read or change it,
        so a radio whose escape was changed on the radio itself delivers tags this decoder
        reads as samples.
*/
inline constexpr std::uint64_t kEscape = 0xAAAA'AAAA'AAAA'AAAAULL;

/*| role: the type field of an RWT in-band tag word.
    frame: the receive path sends Pps at each pulse-per-second edge and Overflow when its
        FIFO nears full, each with the 56-bit sample index of the word it precedes. The
        transmit path takes Hold, StartOfBurst and EndOfBurst.
*/
enum class TagType : std::uint8_t { Time = 0x01, Pps = 0x02, Overflow = 0x03, Hold = 0x04, StartOfBurst = 0x05, EndOfBurst = 0x06 };

/*| role: one in-band tag the decoder took out of the stream.
    contract: frame is the index, within the frames the same decode() call appended, of the
        first frame after the tag.
*/
struct StreamEvent {
    std::uint8_t  type  = 0;
    std::uint64_t value = 0;
    std::size_t   frame = 0;

    bool operator==(const StreamEvent&) const = default;
};

/*| role: the receive decoder: buffer bytes in, complex frames and in-band tags out.
    frame: the buffer is a run of 64-bit little-endian words, each holding four signed 16-bit
        lanes, lane 0 in bits 15:0. A frame is one sample of every streaming channel, and the
        output is frame-interleaved, the first channel first.
    frame: Adi packs in libiio's channel order. One channel: I in lane 0, Q in lane 1, and the
        next sample in lanes 2 and 3. Two channels: the first channel in lanes 0 and 1, the
        second in lanes 2 and 3.
    frame: Rwt packs the earlier sample high. One channel: I in lane 3, Q in lane 2, and the
        next sample in lanes 1 and 0. Two channels: the first channel's I in lane 1 and Q in
        lane 0, the second's I in lane 3 and Q in lane 2.
    contract: under Rwt, a word equal to the escape value starts a tag. The word after it is
        the tag: bit 63 says another tag word follows, bits 62:56 are the type and bits 55:0
        the value. An escape followed by a zero word is a sample word equal to the escape
        value. The state between the two words is held across calls, so a tag split between
        two buffers decodes as one.
    trap: the decoder reads escapes whether or not the radio inserts tags, because a host
        cannot read that setting. A sample word equal to the escape value carries -21846 in
        every lane, outside the ADC's 12-bit range, so a stream without tags never holds one.
    verified-by: ad9361.adi-words-follow-the-channel-order
    verified-by: ad9361.rwt-one-channel-words-put-the-earlier-sample-high
    verified-by: ad9361.rwt-two-channel-words-put-the-first-channel-low
    verified-by: ad9361.a-tag-split-across-buffers-decodes-once
*/
struct SampleDecoder {
    Packing       packing  = Packing::Adi;
    std::uint64_t escape   = kEscape;
    bool          inEscape = false;

    void reset() { inEscape = false; }

    static constexpr std::int16_t lane(std::uint64_t word, int index) { return static_cast<std::int16_t>(static_cast<std::uint16_t>(word >> (16 * index))); }

    static constexpr CF32 part(std::uint64_t word, int iLane, int qLane) { return {static_cast<float>(lane(word, iLane)) * kRxScale, static_cast<float>(lane(word, qLane)) * kRxScale}; }

    void decode(std::span<const std::byte> bytes, std::size_t channels, std::vector<CF32>& out, std::vector<StreamEvent>& events) {
        const std::size_t words  = bytes.size() / sizeof(std::uint64_t);
        const std::size_t frame0 = out.size() / channels;
        const bool        tags   = packing == Packing::Rwt;
        out.reserve(out.size() + 2UZ * words);
        for (std::size_t i = 0; i < words; ++i) {
            std::uint64_t word = 0;
            std::memcpy(&word, bytes.data() + i * sizeof(std::uint64_t), sizeof word);
            if constexpr (std::endian::native == std::endian::big) {
                word = std::byteswap(word);
            }
            if (tags && inEscape) {
                if (word == 0ULL) {
                    inEscape = false;
                    append(escape, out, channels);
                    continue;
                }
                events.push_back({.type = static_cast<std::uint8_t>((word >> 56) & 0x7FU), .value = word & 0x00FF'FFFF'FFFF'FFFFULL, .frame = out.size() / channels - frame0});
                inEscape = (word >> 63) != 0ULL;
                continue;
            }
            if (tags && word == escape) {
                inEscape = true;
                continue;
            }
            append(word, out, channels);
        }
    }

    void append(std::uint64_t word, std::vector<CF32>& out, std::size_t channels) const {
        if (packing == Packing::Adi) {
            out.push_back(part(word, 0, 1));
            out.push_back(part(word, 2, 3));
        } else if (channels == 1UZ) {
            out.push_back(part(word, 3, 2));
            out.push_back(part(word, 1, 0));
        } else {
            out.push_back(part(word, 1, 0));
            out.push_back(part(word, 3, 2));
        }
    }
};

/*| contract: the transmit full scale: a part at +/-1.0 is +/-32767 in its 16-bit lane, which
        the DAC takes the top 12 bits of. A part is clipped first and a NaN sends zero.
*/
inline constexpr float kTxScale = 32767.0f;

inline std::int16_t txLane(float v) { return static_cast<std::int16_t>(std::lround(capture::clipUnit(v) * kTxScale)); }

/*| contract: transmit frames as the FPGA unpacks them, in the lane order SampleDecoder states
        for the same channel count: with one channel two samples a 64-bit little-endian word,
        ADI with the earlier sample low and RWT with it high; with two channels one frame a
        word, ADI with the first channel in lanes 0 and 1 and RWT with it in bits 31:0. frames
        is frame-interleaved, the first channel first. out holds the words; the frames fill it
        from the start, and the rest is zero.
    frame: zero is silence, so a burst's last buffer is padded with it.
    verified-by: ad9361.transmit-words-mirror-the-receive-words
*/
inline void encodeTx(std::span<const CF32> frames, Packing packing, std::size_t channels, std::span<std::byte> out) {
    const std::size_t words = out.size() / sizeof(std::uint64_t);
    const auto        lane  = [](float v) { return static_cast<std::uint64_t>(static_cast<std::uint16_t>(txLane(v))); };
    for (std::size_t w = 0; w < words; ++w) {
        const std::size_t i     = 2UZ * w; // two entries a word, one channel or two
        const CF32        first = i < frames.size() ? frames[i] : CF32{};
        const CF32        next  = i + 1UZ < frames.size() ? frames[i + 1UZ] : CF32{};
        std::uint64_t     word  = 0;
        if (packing == Packing::Adi) {
            word = lane(first.real()) | (lane(first.imag()) << 16) | (lane(next.real()) << 32) | (lane(next.imag()) << 48);
        } else if (channels == 1UZ) {
            word = (lane(first.real()) << 48) | (lane(first.imag()) << 32) | (lane(next.real()) << 16) | lane(next.imag());
        } else {
            word = (lane(first.real()) << 16) | lane(first.imag()) | (lane(next.real()) << 48) | (lane(next.imag()) << 32);
        }
        if constexpr (std::endian::native == std::endian::big) {
            word = std::byteswap(word);
        }
        std::memcpy(out.data() + w * sizeof(std::uint64_t), &word, sizeof word);
    }
}

/*| contract: the AD9361 transmit gain range: an attenuation stated as a gain from -89.75 to 0
        dB in 0.25 dB steps.
*/
inline constexpr double kTxGainMinDb  = -89.75;
inline constexpr double kTxGainMaxDb  = 0.0;
inline constexpr double kTxGainStepDb = 0.25;

/*| role: the ADI core status register both data devices state through libiio's direct
        register access, which iiod serves. On cf-ad9361-lpc bit 2 latches a receive overflow;
        on cf-ad9361-dds-core-lpc bit 0 latches a transmit underflow. Writing a bit back clears
        it.
*/
inline constexpr std::uint32_t kCoreStatusReg    = 0x80000088U;
inline constexpr std::uint32_t kRxOverflowFlag   = 0x4U;
inline constexpr std::uint32_t kTxUnderflowFlag  = 0x1U;

/*| contract: whether the core flagged the condition since the last read, clearing the flag.
        False where the register cannot be read.
*/
inline bool takeStatusFlag(iio_device* dev, std::uint32_t flag) {
    std::uint32_t status = 0;
    if (dev == nullptr || iio_device_reg_read(dev, kCoreStatusReg, &status) < 0 || (status & flag) == 0U) {
        return false;
    }
    std::ignore = iio_device_reg_write(dev, kCoreStatusReg, flag);
    return true;
}

/*| role: a range libiio states in an attribute ending _available, "[min step max]".
    contract: nothing where the text is not three numbers inside brackets, or where min is
        above max.
    verified-by: ad9361.available-ranges-parse
*/
struct AvailableRange {
    double min  = 0.0;
    double step = 0.0;
    double max  = 0.0;

    bool operator==(const AvailableRange&) const = default;
};

inline std::optional<AvailableRange> parseAvailableRange(std::string_view text) {
    const auto open  = text.find('[');
    const auto close = text.find(']');
    if (open == std::string_view::npos || close == std::string_view::npos || close < open) {
        return std::nullopt;
    }
    const std::string inner(text.substr(open + 1, close - open - 1));
    double            v[3]{};
    const char*       p = inner.c_str();
    for (double& x : v) {
        char* end = nullptr;
        x         = std::strtod(p, &end);
        if (end == p) {
            return std::nullopt;
        }
        p = end;
    }
    if (v[0] > v[2]) {
        return std::nullopt;
    }
    return AvailableRange{.min = v[0], .step = v[1], .max = v[2]};
}

// The words of a space-separated _available list, such as the RF port names.
inline std::vector<std::string> parseAvailableList(std::string_view text) {
    std::vector<std::string> out;
    std::size_t              pos = 0;
    while (pos < text.size()) {
        while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n' || text[pos] == '\t')) {
            ++pos;
        }
        std::size_t end = pos;
        while (end < text.size() && text[end] != ' ' && text[end] != '\n' && text[end] != '\t') {
            ++end;
        }
        if (end > pos) {
            out.emplace_back(text.substr(pos, end - pos));
        }
        pos = end;
    }
    return out;
}

/*| contract: the AD9361 receive gain range in dB at a local-oscillator frequency, from the
        full gain table the vendor driver uses: -1 to 73 dB up to 1300 MHz, -3 to 71 dB up to
        4000 MHz and -10 to 62 dB above.
    frame: an open radio states its own range in in_voltage0_hardwaregain_available, which
        the driver moves with the LO. This table answers before a radio is open.
    verified-by: ad9361.the-gain-range-follows-the-band
*/
inline AvailableRange rxGainRangeFor(double loHz) {
    if (loHz <= 1300.0e6) {
        return {.min = -1.0, .step = 1.0, .max = 73.0};
    }
    if (loHz <= 4000.0e6) {
        return {.min = -3.0, .step = 1.0, .max = 71.0};
    }
    return {.min = -10.0, .step = 1.0, .max = 62.0};
}

/*| role: the AD9361's sample-rate bounds.
    frame: the converter clock reaches 61.44 MS/s at the top. Below 25e6/12 S/s the
        transceiver's FIR has to decimate, and libad9361 designs that filter; with the FIR
        decimating by 4 the floor is 25e6/48 S/s.
    contract: a build that compiles libad9361 in reaches kMinRateWithFirHz, and one without
        it kMinRateHz.
*/
inline constexpr double kMaxRateHz        = 61.44e6;
inline constexpr double kMinRateHz        = 2'083'334.0;
inline constexpr double kMinRateWithFirHz = 520'834.0;

#if defined(CAPTURE_AD9361_LIBAD9361)
inline constexpr bool kHaveAd9361 = true;
#else
inline constexpr bool kHaveAd9361 = false;
#endif

inline constexpr double kRateFloorHz = kHaveAd9361 ? kMinRateWithFirHz : kMinRateHz;

/*| contract: whether the AD9361 takes this rate: a whole number of samples per second
        inside [kRateFloorHz, kMaxRateHz].
*/
inline bool rateInRange(double hz) { return hz >= kRateFloorHz && hz <= kMaxRateHz && hz == std::floor(hz); }

/*| contract: whether a rate the radio reads back is the rate asked for: within 1 Hz, or a
        millionth of the rate where that is more. The driver reads back 2499999 for 2500000.
    verified-by: ad9361.rates-follow-the-build
*/
inline bool sameRate(double readBackHz, double wantHz) { return std::abs(readBackHz - wantHz) <= std::max(1.0, wantHz * 1e-6); }

/*| contract: the analog bandwidth to set: the caller's where it is positive, the rate
        otherwise, held inside the bounds. The default bounds are the AD9361's 200 kHz to 56
        MHz; an open radio states its own in rf_bandwidth_available, 20 MHz at the top on an
        AD9363.
    verified-by: ad9361.the-bandwidth-follows-the-rate
*/
inline constexpr AvailableRange kRxBandwidthBounds{.min = 200.0e3, .step = 1.0, .max = 56.0e6};

inline double rxBandwidthFor(double requestHz, double rateHz, const AvailableRange& bounds = kRxBandwidthBounds) { return std::clamp(requestHz > 0.0 ? requestHz : rateHz, bounds.min, bounds.max); }

/*| contract: the value an attribute holds after a write: its read-back where that starts with
        a positive finite number, the value written otherwise.
    frame: the driver can hold a value other than the one written, and states the one it holds.
    verified-by: ad9361.the-filter-stated-is-the-one-the-radio-holds
*/
inline double heldAfterWrite(const std::optional<std::string>& readBack, double written) {
    if (readBack.has_value()) {
        if (const double read = std::strtod(readBack->c_str(), nullptr); std::isfinite(read) && read > 0.0) {
            return read;
        }
    }
    return written;
}

/*| contract: the samples one buffer transfer carries: the caller's count where it is
        positive, a fiftieth of a second at the rate otherwise, and never under 16384. The
        count is a multiple of four.
    why: with one channel streaming a 64-bit word holds two samples, so an odd count would
        cut a word. A network transfer costs a round trip, which a fiftieth of a second
        amortizes.
    verified-by: ad9361.a-transfer-holds-whole-words
*/
inline std::size_t transferSamplesFor(double rateHz, std::size_t requested) {
    const double      derived = rateHz > 0.0 ? rateHz / 50.0 : 0.0;
    const std::size_t want    = requested > 0UZ ? requested : std::max<std::size_t>(16384UZ, static_cast<std::size_t>(derived));
    return std::max<std::size_t>(4UZ, (want + 3UZ) & ~std::size_t{3});
}

/*| contract: the libiio context URI a device string names: uri=<uri> as given, ip=<address>
        as "ip:<address>". Empty where it names neither, which opens libiio's default
        context: IIO_CONTEXT_URI where the environment sets it, and the radio's own devices
        on the radio itself.
    verified-by: ad9361.the-selector-names-a-context
*/
inline std::string contextUriFor(const std::string& kwargs) {
    if (auto uri = capture::kwargsValue(kwargs, "uri"); !uri.empty()) {
        return uri;
    }
    if (auto ip = capture::kwargsValue(kwargs, "ip"); !ip.empty()) {
        return "ip:" + ip;
    }
    return {};
}

/*| contract: the scan channels of a data device a list of device channels enables:
        voltage0 and voltage1 (I and Q) for channel 0, voltage2 and voltage3 for channel 1.
*/
inline std::vector<std::string> scanChannelsFor(std::span<const std::size_t> channels) {
    std::vector<std::string> out;
    for (const std::size_t ch : channels) {
        out.push_back(std::format("voltage{}", 2UZ * ch));
        out.push_back(std::format("voltage{}", 2UZ * ch + 1UZ));
    }
    return out;
}

/*| contract: the refusal for a channel list, or nothing where the device takes it: one or
        two distinct channels, each below deviceChannels, and the second port connected
        exactly when two are listed: a second channel needs a port, and a connected second port
        needs a channel. port names that port and verb the direction in the refusal.
    verified-by: ad9361.a-channel-list-is-checked
*/
inline std::optional<std::string> channelsRefusal(std::span<const std::size_t> channels, std::size_t deviceChannels, bool secondPortConnected, std::string_view port = "out2", std::string_view verb = "receives") {
    if (channels.empty() || channels.size() > 2UZ) {
        return std::format("channels lists {} device channels; a block streams one or two", channels.size());
    }
    if (channels.size() == 2UZ && channels[0] == channels[1]) {
        return std::format("channels lists device channel {} twice", channels[0]);
    }
    for (const std::size_t ch : channels) {
        if (ch >= deviceChannels) {
            return std::format("channels lists device channel {}, and the radio {} on {}", ch, verb, deviceChannels);
        }
    }
    if (secondPortConnected && channels.size() < 2UZ) {
        return std::format("{} is connected, and channels lists one device channel", port);
    }
    if (!secondPortConnected && channels.size() == 2UZ && port != "out2") {
        return std::format("channels lists two device channels, and {} is not connected", port);
    }
    return std::nullopt;
}

// libiio's text for a negative return code.
inline std::string iioError(long code) {
    char buf[256] = {};
    iio_strerror(static_cast<int>(code < 0 ? -code : code), buf, sizeof buf);
    return buf;
}

struct ContextDeleter {
    void operator()(iio_context* ctx) const noexcept { iio_context_destroy(ctx); }
};
using ContextPtr = std::unique_ptr<iio_context, ContextDeleter>;

/*| contract: the context a URI names, or the default one for an empty URI. The error is
        libiio's text for the failure.
*/
inline ContextPtr openContext(const std::string& uri, std::string& error) {
    int          code = 0;
    iio_context* made = iio::createContext(uri, code);
    if (made == nullptr) {
        error = std::format("{}: {}", uri.empty() ? std::string("the default IIO context") : uri, iioError(code != 0 ? code : -EIO));
    }
    return ContextPtr(made);
}

/*| contract: the URI a radio is keyed by, from the spelling a caller gave. A usb: URI that
        names a radio in part, "usb:" or "usb:<bus>.<address>", becomes the one full URI the
        USB scan lists for it; it stays as given where none or several match. An ip: host name
        becomes the numeric address the resolver answers. Every other URI stays as given.
    frame: usbUris supplies the URIs libiio's USB scan lists, and resolveHost the system's name lookup;
        acquireContext passes the real ones, and a case passes its own.
    verified-by: ad9361.a-radio-has-one-key
*/
inline std::string canonicalUri(const std::string& uri, std::span<const std::string> usbUris, const std::function<std::optional<std::string>(const std::string&)>& resolveHost) {
    if (uri.starts_with("usb:")) {
        std::vector<std::string> matches;
        for (const auto& listed : usbUris) {
            if (uri == "usb:" || listed == uri || listed.starts_with(uri + ".")) {
                matches.push_back(listed);
            }
        }
        return matches.size() == 1UZ ? matches.front() : uri;
    }
    if (uri.starts_with("ip:") && uri.size() > 3UZ) {
        if (const auto address = resolveHost(uri.substr(3)); address.has_value()) {
            return "ip:" + *address;
        }
    }
    return uri;
}

// The full URIs libiio's USB scan lists.
inline std::vector<std::string> usbContextUris() { return iio::scanUris("usb"); }

// The first numeric address the system resolves a host name to, or nothing.
inline std::optional<std::string> numericHost(const std::string& host) {
    addrinfo  hints{};
    addrinfo* found = nullptr;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(host.c_str(), nullptr, &hints, &found) != 0 || found == nullptr) {
        return std::nullopt;
    }
    char text[INET6_ADDRSTRLEN] = {};
    const bool named = getnameinfo(found->ai_addr, found->ai_addrlen, text, sizeof text, nullptr, 0, NI_NUMERICHOST) == 0;
    freeaddrinfo(found);
    return named ? std::optional<std::string>(text) : std::nullopt;
}

/*| contract: the identity of the radio a context reaches: its hw_serial where the context
        states one, and the URI it was opened by otherwise.
*/
inline std::string radioIdentity(iio_context* ctx, const std::string& uri) {
    const char* serial = iio::contextAttr(ctx, "hw_serial");
    return serial != nullptr && *serial != '\0' ? std::string("serial:") + serial : "uri:" + uri;
}

/*| role: the IIO context every block of this family in the process holds for one radio.
    contract: acquireContext answers the context a block of this process holds for the radio
        the URI reaches, or opens it with the timeout given where none does. The URI is taken
        to its canonical form first, so "usb:" and the full USB URI of the one radio attached
        share a context, as do a host name and its address. A new context whose radio states
        the hw_serial of a context already held, reached by another URI or another link, is
        dropped and the held one answered. The context is destroyed when the last block lets
        go of its handle. A failed open answers no handle and libiio's text.
    why: libiio's USB backend claims the radio's interface for the first context, and refuses
        a second in the same process, so a source and a sink on one radio over USB need one
        context between them, whichever spelling each was given. iiod serves several
        contexts, and one is still cheaper.
    invariant: each block makes buffer calls on its own data device only. libiio serializes
        the commands it sends for one context, so two blocks' attribute calls may overlap.
    trap: the registry is a static local of an inline function, as radioRatesFor's is. GCC
        emits each as a unique symbol, which the loader shares among the libraries of one
        process. A build that emits them otherwise, such as one with hidden visibility, gives
        each library its own registries and its own context.
    verified-by: ad9361.one-context-per-uri
    verified-by: ad9361.one-context-per-radio
*/
using SharedContext = std::shared_ptr<iio_context>;

inline SharedContext acquireContext(const std::string& uri, unsigned timeoutMs, std::string& error) {
    struct Held {
        std::weak_ptr<iio_context> ctx;
        std::string                identity;
    };
    static std::mutex                  guard;
    static std::map<std::string, Held> held; // by canonical URI
    std::lock_guard                    lock(guard);
    std::erase_if(held, [](const auto& entry) { return entry.second.ctx.expired(); });

    const bool        partialUsb = uri.starts_with("usb:") && std::ranges::count(uri, '.') < 2;
    const auto        usbUris    = partialUsb ? usbContextUris() : std::vector<std::string>{};
    const std::string key        = canonicalUri(uri, usbUris, numericHost);
    if (auto it = held.find(key); it != held.end()) {
        if (auto ctx = it->second.ctx.lock()) {
            return ctx;
        }
    }
    ContextPtr made = openContext(key, error);
    if (!made) {
        return {};
    }
    const std::string identity = radioIdentity(made.get(), key);
    for (const auto& [otherKey, other] : held) {
        if (other.identity == identity) {
            if (auto ctx = other.ctx.lock()) {
                held[key] = {.ctx = ctx, .identity = identity};
                return ctx; // the context just opened is dropped
            }
        }
    }
    std::ignore = iio_context_set_timeout(made.get(), timeoutMs);
    SharedContext ctx(made.release(), ContextDeleter{});
    held[key] = {.ctx = ctx, .identity = identity};
    return ctx;
}

using iio::readAttr;
using iio::writeAttr;

/*| role: the sample rate each block of this process runs one radio at.
    contract: radioRatesFor answers the record for a context, one per context while any block
        holds it. A block enters its rate through applyRadioRate and leaves through
        releaseRadioRate when it closes the radio.
    invariant: a block holds the record for exactly as long as it holds the context.
    trap: the record is keyed by the context's address. Each block releases its rate before it
        lets go of the context, so a context opened later at the same address starts with an
        empty record.
*/
struct RadioRates {
    std::mutex                    guard;
    std::map<const void*, double> held; // by block
};

inline std::shared_ptr<RadioRates> radioRatesFor(iio_context* ctx) {
    static std::mutex                                        guard;
    static std::map<const iio_context*, std::weak_ptr<RadioRates>> all;
    std::lock_guard                                          lock(guard);
    std::erase_if(all, [](const auto& entry) { return entry.second.expired(); });
    if (auto it = all.find(ctx); it != all.end()) {
        if (auto rates = it->second.lock()) {
            return rates;
        }
    }
    auto rates = std::make_shared<RadioRates>();
    all[ctx]   = rates;
    return rates;
}

inline void releaseRadioRate(RadioRates& rates, const void* owner) {
    std::lock_guard lock(rates.guard);
    rates.held.erase(owner);
}

/*| contract: the rate a request is written as: whole hertz inside [kRateFloorHz, kMaxRateHz],
        a request outside the range brought to the nearer end, and one that is not a number to
        kRateFloorHz.
    verified-by: ad9361.rates-follow-the-build
*/
inline double rateToWrite(double requestHz) {
    if (std::isnan(requestHz)) {
        return kRateFloorHz;
    }
    return std::clamp(std::round(requestHz), kRateFloorHz, kMaxRateHz);
}

/*| contract: put the radio behind phy on a request of want S/s for the block owner, and answer
        the rate it runs at afterwards, or why it does not. The request is written as
        rateToWrite has it, and a request it moves by more than half a hertz is named on
        standard error under tag. A
        rate other than the one another block of the process holds the radio at is refused
        before anything is written. A rate the radio already runs is taken as it stands.
        output names the direction whose voltage0 carries the rate.
    why: the AD9361 runs one sample clock for both directions. A write for one block moves
        the other block's stream with no flush, gap marker or tag, and that block's setting
        keeps the old rate. A rate write also recalibrates the radio, which stalls the other
        direction's stream for most of a second.
    verified-by: ad9361.a-second-rate-on-one-radio-is-refused
*/
inline std::expected<double, std::string> applyRadioRate(RadioRates& rates, const void* owner, iio_device* phy, bool output, double request, const char* tag) {
    std::lock_guard lock(rates.guard);
    const double    want = rateToWrite(request);
    if (!(std::abs(want - request) <= 0.5)) {
        std::fprintf(stderr, "[%s] rate %.1f snapped to %.0f, inside the AD9361's %.0f to %.0f S/s\n", tag, request, want, kRateFloorHz, kMaxRateHz);
    }
    for (const auto& [other, hz] : rates.held) {
        if (other != owner && hz > 0.0 && !sameRate(hz, want)) {
            return std::unexpected(std::format("sample_rate asked {:.0f} S/s, and another block in this process streams this radio at {:.0f} S/s; the AD9361 runs one sample clock for both directions", request, hz));
        }
    }
    iio_channel* rate = iio_device_find_channel(phy, "voltage0", output);
    if (const auto text = readAttr(rate, "sampling_frequency"); text.has_value()) {
        if (const double current = std::strtod(text->c_str(), nullptr); sameRate(current, want)) {
            rates.held[owner] = current;
            return current;
        }
    }
    bool ok = false;
#if defined(CAPTURE_AD9361_LIBAD9361)
    ok = ad9361_set_bb_rate(phy, static_cast<unsigned long>(want)) == 0;
#else
    if (iio_channel* fir = iio_device_find_channel(phy, "out", false); fir != nullptr) {
        std::ignore = writeAttr(fir, "voltage_filter_fir_en", std::string("0"));
    }
    ok = writeAttr(rate, "sampling_frequency", std::format("{:.0f}", want));
#endif
    if (!ok) {
        return std::unexpected(std::format("the radio refused a rate of {:.0f} S/s", want));
    }
    const double actual = heldAfterWrite(readAttr(rate, "sampling_frequency"), want);
    rates.held[owner]   = actual;
    return actual;
}

/*| contract: (label, device string) pairs for every IIO context a scan finds that holds
        ad9361-phy and cf-ad9361-lpc. Each context is taken through acquireContext to read its
        devices, so a radio a block of this process streams from is listed through the
        context the block holds rather than refused a second one.
    trap: an RWT image and an ADI one offer the same devices, and no attribute read here tells
        them apart, so the device string names no image. The label carries the context's
        hw_model where it states one.
*/
inline std::vector<std::pair<std::string, std::string>> listRadios() {
    std::vector<std::pair<std::string, std::string>> out;
    for (const std::string& uri : iio::scanUris(nullptr)) {
        std::string error;
        const auto  ctx = acquireContext(uri, 2000U, error);
        if (!ctx || iio_context_find_device(ctx.get(), kPhyName) == nullptr || iio_context_find_device(ctx.get(), kRxName) == nullptr) {
            continue;
        }
        const char* model = iio::contextAttr(ctx.get(), "hw_model");
        out.emplace_back(std::format("AD9361 {} at {}", model != nullptr ? model : "radio", uri), std::format("driver=ad9361,uri={}", uri));
    }
    return out;
}

} // namespace capture::ad9361
