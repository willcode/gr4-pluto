/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

/*| role: what a device block says its controls are, and the two readings a caller needs to
        drive them: the value a key carries in a device string, and whether a frequency is
        inside the coverage the device stated.
    frame: a header of plain values with no dependency of its own, for a caller that draws a
        control surface. capture/common/Device.hpp includes it.
*/
namespace capture {

/*| contract: the value a key carries in a "key=value,key=value" device string, and an empty
        string where the string names no such key. Space around a key or a value is not part of
        it, a comma inside single quotes separates nothing, and a value wrapped in single quotes
        comes back without them.
    frame: the key=value spelling of a device string, and the spelling a person types:
        "index=0, serial=00000001" names a serial, and "label='unit 1,serial=x',serial=A1" names
        the serial A1.
    trap: every family reads its device selector through this and every one of them treats an
        empty answer as "the first device found", so a key that fails to match opens the wrong
        radio and says nothing. A comma and a space produce a key with a space in front of it,
        and that key matched nothing at all; a quoted value carrying a comma split into
        two pairs, so the text after it read as a key of its own.
    verified-by: controls.device-string-key-lookup
*/
inline std::string kwargsValue(const std::string& kwargs, const std::string& key) {
    const auto trim = [](std::string_view s) {
        while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
            s.remove_prefix(1);
        }
        while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
            s.remove_suffix(1);
        }
        return s;
    };

    const std::string_view text(kwargs);
    std::size_t            pos = 0;
    while (pos <= text.size()) {
        // The end of this pair: the next comma that is not inside single quotes.
        std::size_t end    = pos;
        bool        quoted = false;
        while (end < text.size() && (quoted || text[end] != ',')) {
            if (text[end] == '\'') {
                quoted = !quoted;
            }
            ++end;
        }
        const std::string_view pair = text.substr(pos, end - pos);
        if (const auto eq = pair.find('='); eq != std::string_view::npos && trim(pair.substr(0, eq)) == key) {
            std::string_view value = trim(pair.substr(eq + 1));
            if (value.size() >= 2UZ && value.front() == '\'' && value.back() == '\'') {
                value = value.substr(1, value.size() - 2UZ);
            }
            return std::string(value);
        }
        if (end >= text.size()) {
            break;
        }
        pos = end + 1;
    }
    return {};
}

/*| contract: the request, when the device states a range that holds it, and nothing when no
        range does.
    why: refused rather than clamped. A receiver quietly moved to the nearest edge hears one
        frequency while the tag emitted for the same settings change carries another, and every
        frequency axis downstream is then out by the difference with nothing to say so — a
        request for 2.5 GHz on a 2 GHz tuner is 500 MHz of silent error. A refusal leaves the
        device where it is, which the caller can be told about and a clamp cannot.
    invariant: it is also what keeps the cast that follows defined: every family narrows the
        request into a fixed-width integer for its device call, and a negative or over-large
        double cast into one is undefined behavior rather than a wrong frequency.
    verified-by: controls.frequency-out-of-range-refused
*/
inline std::optional<double> frequencyInRange(double requestHz, std::span<const std::pair<double, double>> ranges) {
    for (const auto& [lo, hi] : ranges) {
        if (requestHz >= lo && requestHz <= hi) {
            return requestHz;
        }
    }
    return std::nullopt;
}

/*| role: when a write of one control takes effect.
    contract: Runtime, the write reaches the running stream. Start, the device reads the
        setting only at the next start of its stream. setControl refuses a write to a stopped
        block. A family either keeps a Start write made while the device is up for the next
        start, or refuses every value but the one in force; its README says which. A caller
        that builds a new block for the next start passes the value as the open-time setting
        the README names.
    why: a caller draws the controls that wait for a restart apart from the live ones.
*/
enum class AppliesAt : std::uint8_t { Runtime, Start };

/*| role: one control a device block declares, as its device really works.
    frame: kind codes are shared with a consumer's own facade: 0 continuous range, 1 discrete
        steps, 2 toggle, 3 enum with the value as the option index, 4 list — a non-uniform value
        ladder such as a tuner's gain table, whose value is the entry itself and which a caller
        draws as a selection over the entries.
    frame: anyInRange marks a list whose device also takes every value between min and max.
        min and max then state that range, and a caller lets a person enter a value inside it.
        A list without it takes its entries alone.
    why: a caller lays the controls out from the descriptors alone, with the steps, toggles,
        options and ladders the device really has.
    frame: isGain marks a gain, which the hardware's automatic gain mode overrides.
        inoperativeAboveHz marks a control the hardware ignores above a tuner frequency.
*/
struct ControlSeed {
    const char*        id;      // stable id; for gains this is the legacy config key
    const char*        label;
    const char*        unit;
    int                kind;    // 0 range, 1 steps, 2 toggle, 3 enum, 4 list
    double             min;
    double             max;
    double             step;
    double             defValue;
    const char* const* options;    // enum kind: nOptions strings, value is the index
    int                nOptions;
    const double*      listValues; // list kind: nListValues ascending values, value is the value itself
    int                nListValues;
    bool               isGain;
    double             inoperativeAboveHz; // 0 = always operative
    /*| frame: sessionOnly comes up at defValue and is never stored, for a control such as a
            bias tee that acts on whatever is connected now.
    */
    bool               sessionOnly = false;
    AppliesAt          appliesAt   = AppliesAt::Runtime;
    bool               anyInRange  = false;
};

/*| role: one live reading from a device's own sensors or from the stream a block runs.
    contract: a reading is something the device or the stream produces. A value a control or a
        setting sets is never a reading: the control surface states it. The analog bandwidth is
        the one exception, the filter in force, which a fixed filter states and no control
        reaches. brief marks the readings an operator watches while receiving: the stream
        state, an overload, dropped samples, a lock, a measured error. A transport diagnostic
        carries brief false: a drill-down reads it and a glance row leaves it out.
    frame: label is a short noun phrase for a person. value is display-ready, unit included
        where the label does not name what is counted.
    invariant: a device's readings exist only while the source is streaming, read from the
        open device. The stream state is the one reading of a block whose device is down.
    trap: good carries the meaning of a two-state sensor: a lock that is not locked reads
        false. A reading with no pass or fail sense reads true. The field good is therefore no
        validity flag.
        brief is false for a multi-field record, an NMEA sentence or a GPSDO servo status line.
*/
struct SensorReading {
    std::string id;    // the device's own sensor name, e.g. "ref_locked"
    std::string label; // the reading's display name
    std::string value; // formatted value, unit included
    bool        good  = true;
    bool        brief = true;
};

/*| role: the caller state a control surface may depend on.
    contract: describeControls() computes per-band ladders and rate-derived defaults from it.
    frame: gainMode is the automatic gain mode the caller holds, which becomes the GAIN_MODE
        descriptor's default. wireFormat is the link packing the caller holds, where the family
        has one; empty stands for the family's default packing.
*/
struct TruthContext {
    std::string deviceParams; // "key=value,..." device selector
    double      sampleRate = 0.0;
    double      centerFreq = 0.0;
    bool        gainMode   = false;
    std::string wireFormat = {};
};

/*| role: an owning control descriptor, the type a source and a sink both answer a caller
        with.
    frame: built by each family's describeControls(const TruthContext&) from its constexpr
        ControlSeed table, or from a device probe. The field meanings match ControlSeed;
        defValue is the value shown when a caller holds no stored override.
    frame: rateDerived marks a control whose default re-derives from the sample rate. A caller
        drops a stored value of one at every rate change.
    frame: ranges holds the spans a range control covers, each as [min, max] and in ascending
        order, where it covers more than one. min and max are then the outer ends, and a value
        between two spans is outside the control. Empty where [min, max] is the one span.
*/
struct ControlDesc {
    std::string              id;
    std::string              label;
    std::string              unit;
    int                      kind = 0; // 0 range, 1 steps, 2 toggle, 3 enum, 4 list
    double                   min = 0.0, max = 0.0, step = 0.0;
    double                   defValue = 0.0;
    std::vector<std::string> options;
    std::vector<double>      listValues;
    bool                     isGain             = true;
    double                   inoperativeAboveHz = 0.0;
    bool                     rateDerived        = false;
    bool                     sessionOnly        = false; // never persisted (see ControlSeed)
    AppliesAt                appliesAt          = AppliesAt::Runtime;
    bool                     anyInRange         = false; // a list that takes [min, max] too (see ControlSeed)
    std::vector<std::pair<double, double>> ranges; // the spans of a range control, where there are several

    friend bool operator==(const ControlDesc&, const ControlDesc&) = default;
};

inline ControlDesc controlFromSeed(const ControlSeed& d) {
    ControlDesc c;
    c.id       = d.id;
    c.label    = d.label;
    c.unit     = d.unit;
    c.kind     = d.kind;
    c.min      = d.min;
    c.max      = d.max;
    c.step     = d.step;
    c.defValue = d.defValue;
    if (d.options != nullptr && d.nOptions > 0) {
        c.options.assign(d.options, d.options + d.nOptions);
    }
    if (d.listValues != nullptr && d.nListValues > 0) {
        c.listValues.assign(d.listValues, d.listValues + d.nListValues);
    }
    c.isGain             = d.isGain;
    c.inoperativeAboveHz = d.inoperativeAboveHz;
    c.sessionOnly        = d.sessionOnly;
    c.appliesAt          = d.appliesAt;
    c.anyInRange         = d.anyInRange;
    return c;
}

/*| contract: bound one value to what a descriptor states this control can take. The answer is
        inside the descriptor's own domain: [min, max] for a range, a step or a toggle, the
        option indices for an enum, and the first-to-last span of the ladder for a list, whose
        entries are the value itself, widened to [min, max] where the list is anyInRange. A value that is not a number comes back as the
        descriptor's default. A descriptor stating no usable domain, max below min, leaves the
        value alone rather than collapsing it.
    trap: std::clamp answers NaN for NaN, neither comparison holding, so a stored value read
        back as one reaches a family's setControl and its device call unbounded. The default is
        the one value in the domain that every descriptor carries.
    trap: an enum's domain is its option count, and not min and max. A descriptor built field by
        field that names its options and leaves the range alone has hi < lo false at 0 and 0, so
        every selection would clamp to the first option. Every enum descriptor in this library
        sets the range from the option count, so no instance shows it.
    trap: a steps control's listValues are a display ladder over the step index rather than the
        value domain, so only kind 4 reads them as values.
    why: a device parameter whose legal domain moves with the sample rate states the domain in
        force, so a value stored under one rate cannot reach the device under another that
        cannot carry it. This is the arithmetic; each family's describeControls() states the
        domain and each family's setControl() applies the same bound at the device. A stored
        value outlives the state it was stored under, so the bound belongs wherever such a value
        is read as well as where it is written.
    verified-by: controls.bandwidth-ceiling-per-rate
*/
inline double clampToControl(const ControlDesc& d, double value) {
    if (value != value) {
        return d.defValue;
    }
    double lo = d.min;
    double hi = d.max;
    if (d.kind == 3 && !d.options.empty()) {
        lo = 0.0;
        hi = static_cast<double>(d.options.size() - 1UZ);
    }
    if (d.kind == 4 && !d.listValues.empty()) {
        lo = d.listValues.front();
        hi = d.listValues.back();
        if (d.anyInRange) {
            lo = std::min(lo, d.min);
            hi = std::max(hi, d.max);
        }
    }
    return hi < lo ? value : std::clamp(value, lo, hi);
}

inline std::vector<ControlDesc> controlsFromSeeds(std::span<const ControlSeed> seeds) {
    std::vector<ControlDesc> out;
    out.reserve(seeds.size());
    for (const auto& d : seeds) {
        out.push_back(controlFromSeed(d));
    }
    return out;
}

/*| role: the ids of the two descriptors every device block states beside its device's own
        controls: the sample rate and the center frequency, both in hertz.
    contract: SAMPLE_RATE is a list whose entries are the family's rate ladder, anyInRange
        where the device takes every rate between min and max. FREQUENCY is a range whose min
        and max are the outer ends of the coverage, with ranges carrying the spans where there
        are several. Each default is the family's default setting. A write of either through
        the control property stages the block's sample_rate or frequency setting, and the block
        applies it as it applies any setting write.
    why: a program that holds no block type reads the rate ladder and the coverage from the
        surface, as it reads every other control, rather than from the family's static calls.
*/
inline constexpr const char* kSampleRateControl = "SAMPLE_RATE";
inline constexpr const char* kFrequencyControl  = "FREQUENCY";

/*| role: the id of the descriptor a source states where its unit has an automatic gain mode.
    contract: a toggle whose default is the gain_mode the block holds. A family whose unit has
        no automatic mode states no such descriptor, so its presence in a surface says whether
        the mode exists. A write through the control property stages the block's gain_mode
        setting.
*/
inline constexpr const char* kGainModeControl = "GAIN_MODE";

// The GAIN_MODE descriptor, its default the mode the caller holds.
inline ControlDesc gainModeControl(bool on) {
    ControlDesc c;
    c.id       = kGainModeControl;
    c.label    = "Automatic gain";
    c.kind     = 2;
    c.min      = 0.0;
    c.max      = 1.0;
    c.step     = 1.0;
    c.defValue = on ? 1.0 : 0.0;
    c.isGain   = false;
    return c;
}

/*| contract: the SAMPLE_RATE descriptor for a ladder, ascending, and the default rate. A
        span marks a device that takes every rate inside it, and min and max are then that
        span; without one they are the ends of the ladder.
*/
inline ControlDesc sampleRateControl(std::vector<double> ladderHz, double defaultHz, std::optional<std::pair<double, double>> anyWithinHz = std::nullopt) {
    ControlDesc c;
    c.id         = kSampleRateControl;
    c.label      = "Sample rate";
    c.unit       = "Hz";
    c.kind       = 4;
    c.anyInRange = anyWithinHz.has_value();
    c.min        = anyWithinHz.has_value() ? anyWithinHz->first : (ladderHz.empty() ? 0.0 : ladderHz.front());
    c.max        = anyWithinHz.has_value() ? anyWithinHz->second : (ladderHz.empty() ? 0.0 : ladderHz.back());
    c.listValues = std::move(ladderHz);
    c.defValue   = defaultHz;
    c.isGain     = false;
    return c;
}

/*| contract: the FREQUENCY descriptor for the coverage a family states, as spans in hertz,
        and the default center. min and max are the lowest and the highest end; ranges holds the
        spans, sorted, where there are several.
*/
inline ControlDesc frequencyControl(std::span<const std::pair<double, double>> coverageHz, double defaultHz) {
    ControlDesc c;
    c.id       = kFrequencyControl;
    c.label    = "Frequency";
    c.unit     = "Hz";
    c.kind     = 0;
    c.defValue = defaultHz;
    c.isGain   = false;
    if (coverageHz.empty()) {
        return c;
    }
    c.min = coverageHz.front().first;
    c.max = coverageHz.front().second;
    for (const auto& [lo, hi] : coverageHz) {
        c.min = std::min(c.min, lo);
        c.max = std::max(c.max, hi);
    }
    if (coverageHz.size() > 1UZ) {
        c.ranges.assign(coverageHz.begin(), coverageHz.end());
        std::ranges::sort(c.ranges);
    }
    return c;
}

/*| contract: append SAMPLE_RATE and FREQUENCY to a surface, each where the family states
        something for it: a ladder with at least one rate, a coverage with at least one span.
*/
inline void appendRateAndFrequency(std::vector<ControlDesc>& surface, std::vector<double> ladderHz, double defaultRateHz, std::optional<std::pair<double, double>> anyWithinHz, std::span<const std::pair<double, double>> coverageHz,
                                   double defaultFrequencyHz) {
    if (!ladderHz.empty()) {
        surface.push_back(sampleRateControl(std::move(ladderHz), defaultRateHz, anyWithinHz));
    }
    if (!coverageHz.empty()) {
        surface.push_back(frequencyControl(coverageHz, defaultFrequencyHz));
    }
}

/*| role: the ids of the readings a block states from its own state and the filter it
        holds, beside the readings of its device.
    contract: the stream state is one of the words below. good is true for streaming alone,
        so a caller reads whether the device is up from that flag. A source states it under
        rx_stream_state and a sink under tx_stream_state. The analog bandwidth is the width
        of the analog filter in force, in hertz, stated by a source whose family states one
        and absent from every other.
    contract: every source states rx_dropped_samples, the samples this host lost since the
        stream started, on a glance row, and then rx_dropped_markers, the tags its output
        port had no room for, on a detail row. Both are counts and read good.
*/
inline constexpr const char* kStreamStateReading     = "rx_stream_state";
inline constexpr const char* kTxStreamStateReading   = "tx_stream_state";
inline constexpr const char* kAnalogBandwidthReading = "rx_analog_bandwidth";
inline constexpr const char* kDroppedSamplesReading  = "rx_dropped_samples";
inline constexpr const char* kDroppedMarkersReading  = "rx_dropped_markers";

/*| contract: the id of the reading a block that answers the sensors property from a cache adds
        to a whole answer made before a whole sweep has landed. Such an answer carries the brief
        readings, and a later whole request receives the whole set.
*/
inline constexpr const char* kSensorsWholePendingReading = "sensors_whole_pending";

/*| contract: the words of the stream state. stopped, the block holds no stream: it has not
        started, or it has stopped. starting, the block is running and its start has not yet
        put the stream in place. streaming, the device is up and delivering. paused, a sink
        whose stream is paused. failed, a start that threw or a stream that gave up on the
        device, until the next start.
*/
namespace stream_state {
inline constexpr const char* kStopped   = "stopped";
inline constexpr const char* kStarting  = "starting";
inline constexpr const char* kStreaming = "streaming";
inline constexpr const char* kPaused    = "paused";
inline constexpr const char* kFailed    = "failed";
} // namespace stream_state

// The analog bandwidth reading, the width in whole hertz followed by the unit.
inline SensorReading analogBandwidthReading(double hz) { return {.id = kAnalogBandwidthReading, .label = "Analog bandwidth", .value = std::format("{:.0f} Hz", hz), .good = true, .brief = true}; }

// The dropped samples reading, a glance row.
inline SensorReading droppedSamplesReading(std::uint64_t samples) { return {.id = kDroppedSamplesReading, .label = "Dropped samples", .value = std::format("{}", samples), .good = true, .brief = true}; }

// The lost markers reading, a detail row.
inline SensorReading droppedMarkersReading(std::size_t markers) { return {.id = kDroppedMarkersReading, .label = "Lost markers", .value = std::format("{}", markers), .good = true, .brief = false}; }

/*| contract: the analog bandwidth a reading set states, in hertz, and nothing where the set
        carries no such reading or its value does not start with a number.
*/
inline std::optional<double> analogBandwidthIn(std::span<const SensorReading> readings) {
    const auto it = std::ranges::find_if(readings, [](const SensorReading& r) { return r.id == kAnalogBandwidthReading; });
    if (it == readings.end()) {
        return std::nullopt;
    }
    double     hz  = 0.0;
    const auto end = it->value.data() + it->value.size();
    if (const auto parsed = std::from_chars(it->value.data(), end, hz); parsed.ec != std::errc{}) {
        return std::nullopt;
    }
    return hz;
}

} // namespace capture

