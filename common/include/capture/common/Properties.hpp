/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Message.hpp>

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Discovery.hpp>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <expected>
#include <format>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

/*| role: the four message properties every device block answers, and the property maps
        a control descriptor, a sensor reading and a device listing travel in.
    frame: the keys of every map here are stated once, in knowledge/common.md, and
        the conversions below are their one spelling.
    contract: controls answers Get with the surface the block states for the device it holds,
        and Subscribe with the same surface; a subscriber then receives it again, as a
        notification, each time it differs from the one last sent. control answers Set of one
        id and one value with the value in force after the write, or an error naming the id
        and the cause. sensors answers Get with one reading set, the block's stream state
        first; a subscriber receives every set a Get produces for another caller. devices
        answers Get with the family's listing.
    contract: a block registers the four when it is built, so a block that has not started
        answers them too. controls then answers the surface the discovery probe states for the
        selector, the rate, the frequency, the gain mode and the wire format the block holds
        staged or set, control stages a SAMPLE_RATE, FREQUENCY or GAIN_MODE write and refuses
        every other id, and sensors answers the stream state alone.
    contract: each block states controlSurface(), the surface of the device it holds at the
        rate and the frequency in force, computed from a Snapshot without the block's control
        mutex. Its controlProperty() forwards to answerControlProperty.
    trap: every answer runs on the scheduler worker that runs the block, and holds the
        block's stream while it is made. A full sensor sweep can take most of a second. A
        sensors Get therefore takes the brief sweep unless it asks for the full one.
*/
namespace capture {

namespace property {
inline constexpr const char* kControls = "controls";
inline constexpr const char* kControl  = "control";
inline constexpr const char* kSensors  = "sensors";
inline constexpr const char* kDevices  = "devices";
} // namespace property

namespace detail {

// A number a caller put in a map, whatever arithmetic type it arrived as.
inline std::optional<double> numberIn(const gr::pmt::Value& v) {
    if (const auto* d = v.get_if<double>()) {
        return *d;
    }
    if (const auto* f = v.get_if<float>()) {
        return static_cast<double>(*f);
    }
    if (const auto* b = v.get_if<bool>()) {
        return *b ? 1.0 : 0.0;
    }
    if (const auto* i = v.get_if<std::int64_t>()) {
        return static_cast<double>(*i);
    }
    if (const auto* i = v.get_if<std::int32_t>()) {
        return static_cast<double>(*i);
    }
    if (const auto* i = v.get_if<std::uint64_t>()) {
        return static_cast<double>(*i);
    }
    if (const auto* i = v.get_if<std::uint32_t>()) {
        return static_cast<double>(*i);
    }
    return std::nullopt;
}

inline const gr::pmt::Value* field(const gr::property_map& map, std::string_view key) {
    const auto it = map.find(key);
    return it == map.end() ? nullptr : &it->second;
}

inline std::string textField(const gr::property_map& map, std::string_view key) {
    const auto* v = field(map, key);
    return v == nullptr ? std::string{} : v->value_or(std::string{});
}

inline double numberField(const gr::property_map& map, std::string_view key, double fallback = 0.0) {
    const auto* v = field(map, key);
    return v == nullptr ? fallback : numberIn(*v).value_or(fallback);
}

inline bool flagField(const gr::property_map& map, std::string_view key, bool fallback) {
    const auto* v = field(map, key);
    return v == nullptr ? fallback : numberIn(*v).value_or(fallback ? 1.0 : 0.0) != 0.0;
}

// The maps a list value holds, in order; empty where the key holds no list.
inline std::vector<gr::property_map> mapsIn(const gr::property_map& map, std::string_view key) {
    std::vector<gr::property_map> out;
    const auto*                   v = field(map, key);
    if (v == nullptr) {
        return out;
    }
    if (const auto* list = v->get_if<gr::Tensor<gr::pmt::Value>>()) {
        for (const auto& element : *list) {
            if (const auto* m = element.get_if<gr::property_map>()) {
                out.push_back(*m);
            }
        }
    }
    return out;
}

inline gr::pmt::Value listOf(std::vector<gr::property_map> maps) {
    gr::Tensor<gr::pmt::Value> list;
    list.reserve(maps.size());
    for (auto& m : maps) {
        list.emplace_back(gr::pmt::Value(std::move(m)));
    }
    return gr::pmt::Value(std::move(list));
}

inline gr::Message errorReply(gr::Message request, std::string reason) {
    request.data = std::unexpected(gr::Error(reason));
    return request;
}

} // namespace detail

/*| contract: one descriptor as a property map, every field under its fixed key: id, label and
        unit as strings; kind as an integer; min, max, step, default and inoperative_above_hz
        as numbers; options as a list of strings and list_values as a list of numbers, both
        present and empty where the descriptor has none; is_gain, rate_derived, session_only
        and any_in_range as booleans; applies_at as the string "runtime" or "start"; ranges as
        a list of [min, max] pairs of numbers, present and empty where the descriptor has one
        span.
*/
inline gr::property_map controlToMap(const ControlDesc& c) {
    gr::property_map m;
    m["id"]                   = c.id;
    m["label"]                = c.label;
    m["unit"]                 = c.unit;
    m["kind"]                 = static_cast<std::int32_t>(c.kind);
    m["min"]                  = c.min;
    m["max"]                  = c.max;
    m["step"]                 = c.step;
    m["default"]              = c.defValue;
    m["options"]              = c.options;
    m["list_values"]          = c.listValues;
    m["is_gain"]              = c.isGain;
    m["inoperative_above_hz"] = c.inoperativeAboveHz;
    m["rate_derived"]         = c.rateDerived;
    m["session_only"]         = c.sessionOnly;
    m["applies_at"]           = std::string(c.appliesAt == AppliesAt::Start ? "start" : "runtime");
    m["any_in_range"]         = c.anyInRange;
    gr::Tensor<gr::pmt::Value> spans;
    spans.reserve(c.ranges.size());
    for (const auto& [lo, hi] : c.ranges) {
        spans.emplace_back(gr::pmt::Value(std::vector<double>{lo, hi}));
    }
    m["ranges"] = gr::pmt::Value(std::move(spans));
    return m;
}

/*| contract: the descriptor a map from controlToMap carries. A key the map lacks leaves the
        field at the descriptor's own default, and a number may arrive as any arithmetic type.
        applies_at reads start for the word "start" alone, and runtime for any other word or
        for no key.
    why: runtime is every descriptor's default. A map without the key states that default, as
        it does for every other key.
*/
inline ControlDesc controlFromMap(const gr::property_map& m) {
    ControlDesc c;
    c.id                 = detail::textField(m, "id");
    c.label              = detail::textField(m, "label");
    c.unit               = detail::textField(m, "unit");
    c.kind               = static_cast<int>(detail::numberField(m, "kind"));
    c.min                = detail::numberField(m, "min");
    c.max                = detail::numberField(m, "max");
    c.step               = detail::numberField(m, "step");
    c.defValue           = detail::numberField(m, "default");
    c.inoperativeAboveHz = detail::numberField(m, "inoperative_above_hz");
    c.isGain             = detail::flagField(m, "is_gain", c.isGain);
    c.rateDerived        = detail::flagField(m, "rate_derived", c.rateDerived);
    c.sessionOnly        = detail::flagField(m, "session_only", c.sessionOnly);
    c.appliesAt          = detail::textField(m, "applies_at") == "start" ? AppliesAt::Start : AppliesAt::Runtime;
    c.anyInRange         = detail::flagField(m, "any_in_range", c.anyInRange);
    if (const auto* v = detail::field(m, "options")) {
        if (const auto* list = v->get_if<gr::Tensor<gr::pmt::Value>>()) {
            for (const auto& option : *list) {
                c.options.push_back(option.value_or(std::string{}));
            }
        }
    }
    if (const auto* v = detail::field(m, "list_values")) {
        if (const auto* list = v->get_if<gr::Tensor<double>>()) {
            c.listValues.assign(list->begin(), list->end());
        }
    }
    if (const auto* v = detail::field(m, "ranges")) {
        if (const auto* list = v->get_if<gr::Tensor<gr::pmt::Value>>()) {
            for (const auto& span : *list) {
                if (const auto* ends = span.get_if<gr::Tensor<double>>(); ends != nullptr && ends->size() == 2UZ) {
                    c.ranges.emplace_back((*ends)[0UZ], (*ends)[1UZ]);
                }
            }
        }
    }
    return c;
}

/*| contract: one reading as a property map: id, label and value as strings, good and brief
        as booleans.
*/
inline gr::property_map sensorToMap(const SensorReading& s) {
    gr::property_map m;
    m["id"]    = s.id;
    m["label"] = s.label;
    m["value"] = s.value;
    m["good"]  = s.good;
    m["brief"] = s.brief;
    return m;
}

inline SensorReading sensorFromMap(const gr::property_map& m) {
    return {.id = detail::textField(m, "id"), .label = detail::textField(m, "label"), .value = detail::textField(m, "value"), .good = detail::flagField(m, "good", true), .brief = detail::flagField(m, "brief", true)};
}

// The body of a controls reply: the surface as a list of descriptor maps under "controls".
inline gr::property_map controlsReply(std::span<const ControlDesc> surface) {
    std::vector<gr::property_map> maps;
    maps.reserve(surface.size());
    for (const auto& c : surface) {
        maps.push_back(controlToMap(c));
    }
    return {{"controls", detail::listOf(std::move(maps))}};
}

inline std::vector<ControlDesc> controlsFromReply(const gr::property_map& body) {
    std::vector<ControlDesc> out;
    for (const auto& m : detail::mapsIn(body, "controls")) {
        out.push_back(controlFromMap(m));
    }
    return out;
}

// The body of a sensors reply: the readings as a list of maps under "sensors".
inline gr::property_map sensorsReply(std::span<const SensorReading> readings) {
    std::vector<gr::property_map> maps;
    maps.reserve(readings.size());
    for (const auto& s : readings) {
        maps.push_back(sensorToMap(s));
    }
    return {{"sensors", detail::listOf(std::move(maps))}};
}

inline std::vector<SensorReading> sensorsFromReply(const gr::property_map& body) {
    std::vector<SensorReading> out;
    for (const auto& m : detail::mapsIn(body, "sensors")) {
        out.push_back(sensorFromMap(m));
    }
    return out;
}

/*| contract: the body of a devices reply: each unit of a listing as a map of label and
        selector, the selector being what device_parameter takes, in a list under "devices",
        and under "opens" whether the family's listing opens a unit to name it.
*/
inline gr::property_map devicesReply(std::span<const std::pair<std::string, std::string>> listing, bool opens) {
    std::vector<gr::property_map> maps;
    maps.reserve(listing.size());
    for (const auto& [label, selector] : listing) {
        maps.push_back({{"label", label}, {"selector", selector}});
    }
    return {{"devices", detail::listOf(std::move(maps))}, {"opens", opens}};
}

// The (label, selector) pairs of a devices reply, in the order the family listed them.
inline std::vector<std::pair<std::string, std::string>> devicesFromReply(const gr::property_map& body) {
    std::vector<std::pair<std::string, std::string>> out;
    for (const auto& m : detail::mapsIn(body, "devices")) {
        out.emplace_back(detail::textField(m, "label"), detail::textField(m, "selector"));
    }
    return out;
}

/*| role: what one block keeps for its properties: who subscribed to controls and sensors, and
        the surface last sent to the controls subscribers.
    invariant: a member of every device block, declared before the teardown guard and built
        from the block's own pointer, which registers the four properties on the block.
    why: its own mutex, because the settings drain sends a changed surface on the block's own
        thread while the scheduler's worker adds and removes subscribers.
*/
struct ControlProperties {
    ControlProperties() = default;

    template <typename TBlock>
    explicit ControlProperties(TBlock* blk) {
        using Callback = gr::BlockBase::PropertyCallback;
        for (const char* name : {property::kControls, property::kControl, property::kSensors, property::kDevices}) {
            blk->propertyCallbacks.try_emplace(name, static_cast<Callback>(&TBlock::controlProperty));
        }
    }

    std::mutex               mutex;
    std::set<std::string>    controlsClients;
    std::set<std::string>    sensorsClients;
    std::vector<ControlDesc> lastSurface;
};

/*| role: the values a block's control surface is computed from, as the thread that last changed
        them left them.
    contract: publish() replaces the values held, and read() answers a copy. A block publishes
        after every write of one of those values, under the lock that write holds, and its
        controlSurface() computes the surface from read().
    invariant: the mutex is held for the copy alone and never across a device call, so a read
        waits at most as long as a copy takes.
    why: a settings hook reads the surface on the thread that drains the device, and the
        block's control mutex is held across device calls as long as a tune.
*/
template <typename T>
class Snapshot {
public:
    Snapshot() = default;
    explicit Snapshot(T values) : _values(std::move(values)) {}

    void publish(T values) {
        std::lock_guard lock(_mutex);
        _values = std::move(values);
    }

    [[nodiscard]] T read() const {
        std::lock_guard lock(_mutex);
        return _values;
    }

private:
    mutable std::mutex _mutex;
    T                  _values{};
};

/*| contract: send the block's surface to every controls subscriber where it differs from the
        one last sent, and do nothing where nobody subscribed. Safe from any thread of the
        block that holds none of the block's own locks.
    invariant: the surface is read, compared with the one last sent, recorded and sent under
        the one subscriber mutex, in that order. Two threads sending at once therefore send in
        the order they read, and the last surface sent is the last one read.
    frame: each block calls this after its settings drain and at the end of its start, and the
        control property calls it after every write, which covers every path that moves a
        surface: the rate, the frequency, the device a start opened, and a control whose write
        moves another control's domain.
*/
template <typename TBlock>
void publishControlSurface(TBlock& blk) {
    auto&           state = blk._controlProperties;
    std::lock_guard lock(state.mutex);
    if (state.controlsClients.empty()) {
        return;
    }
    std::vector<ControlDesc> surface = blk.controlSurface();
    if (surface == state.lastSurface) {
        return;
    }
    state.lastSurface           = std::move(surface);
    const gr::property_map body = controlsReply(state.lastSurface);
    for (const auto& client : state.controlsClients) {
        blk.emitMessage(property::kControls, body, client);
    }
}

/*| contract: on leaving its scope, send the block's surface to the controls subscribers where
        it moved. A settings hook declares one as its first statement.
    why: a scope guard runs on every return from the hook, the early ones included, and it runs
        after the body has published every value the body moved.
*/
template <typename TBlock>
struct ControlSurfaceCheck {
    TBlock& blk;
    ~ControlSurfaceCheck() { publishControlSurface(blk); }
};

/*| contract: the stream state of a block, one of the words of stream_state, from its lifecycle
        state, whether its device is up, and whether its stream gave up on the device where
        the family keeps that flag. An error state or a stream that gave up reads failed. A
        running block reads streaming with its device up and starting before that. A paused
        block reads paused, and every other state reads stopped.
    why: the lifecycle state alone says running from the moment the framework calls start(),
        before the device is open, and goes on saying it after a stream gave up.
*/
template <typename TBlock>
std::string_view streamStateOf(const TBlock& blk) {
    using gr::lifecycle::State;
    const State state = blk.state();
    bool        dead  = false;
    if constexpr (requires { blk.streamDead(); }) {
        dead = blk.streamDead();
    }
    if (state == State::ERROR || dead) {
        return stream_state::kFailed;
    }
    if (state == State::RUNNING) {
        if constexpr (requires { blk.deviceUp(); }) {
            return blk.deviceUp() ? stream_state::kStreaming : stream_state::kStarting;
        } else {
            return stream_state::kStreaming;
        }
    }
    if (state == State::REQUESTED_PAUSE || state == State::PAUSED) {
        return stream_state::kPaused;
    }
    return stream_state::kStopped;
}

// The stream state as a reading: brief, and good while the stream runs.
template <typename TBlock>
SensorReading streamStateReading(const TBlock& blk) {
    const std::string_view word = streamStateOf(blk);
    return {.id = detail::TransmitBlock<TBlock> ? kTxStreamStateReading : kStreamStateReading, .label = "Stream", .value = std::string(word), .good = word == stream_state::kStreaming, .brief = true};
}

namespace detail {

/*| contract: a block that keeps its readings in a cache answers from cachedSensors, which reads
        no device on the thread that serves the block's messages. Any other block sweeps its
        device here.
*/
template <typename TBlock>
std::optional<std::vector<SensorReading>> sweepSensors(TBlock& blk, bool full) {
    std::vector<SensorReading> out;
    if constexpr (requires { blk.cachedSensors(out, TBlock::SensorSweep::Full); }) {
        if (!blk.cachedSensors(out, full ? TBlock::SensorSweep::Full : TBlock::SensorSweep::Brief)) {
            return std::nullopt;
        }
    } else if constexpr (requires { typename TBlock::SensorSweep; }) {
        if (!blk.readSensors(out, full ? TBlock::SensorSweep::Full : TBlock::SensorSweep::Brief)) {
            return std::nullopt;
        }
    } else if constexpr (requires { blk.readSensors(out); }) {
        if (!blk.readSensors(out)) {
            return std::nullopt;
        }
    }
    return out;
}

template <typename TBlock>
bool deviceIsUp(const TBlock& blk) {
    if constexpr (requires { blk.deviceUp(); }) {
        return blk.deviceUp();
    } else {
        return true;
    }
}

template <typename TBlock>
bool writeControl(TBlock& blk, const ControlDesc& desc, double value) {
    if constexpr (requires { blk.setElementGain(desc.id, value); }) {
        if (desc.isGain) {
            return blk.setElementGain(desc.id, value);
        }
    }
    return blk.setControl(desc.id, value);
}

// The value the block reads back for one control, or nothing where it keeps no record of it.
template <typename TBlock>
std::optional<double> readBack(TBlock& blk, const ControlDesc& desc) {
    if constexpr (requires { blk.elementGain(desc.id); }) {
        if (desc.isGain) {
            return blk.elementGain(desc.id);
        }
    }
    if constexpr (requires { blk.controlValue(desc.id); }) {
        return blk.controlValue(desc.id);
    }
    return std::nullopt;
}

// Whether the block has not started: built, and initialized where a graph holds it.
template <typename TBlock>
bool unstarted(const TBlock& blk) {
    const gr::lifecycle::State state = blk.state();
    return state == gr::lifecycle::State::IDLE || state == gr::lifecycle::State::INITIALISED;
}

/*| contract: the context the block's next start would open its unit with: the selector, the
        rate, the first frequency, the automatic gain mode and the wire format, each as the
        block holds it staged, or as it holds it set where nothing is staged. A block that
        declares no gain_mode or no wire_format leaves that field at its default.
*/
template <typename TBlock>
TruthContext stagedContext(TBlock& blk) {
    const gr::property_map staged = blk.settings().stagedParameters();
    const auto             held   = [&](const std::string& key) -> std::optional<gr::pmt::Value> {
        if (const auto* v = field(staged, key)) {
            return *v;
        }
        return blk.settings().get(key);
    };
    TruthContext ctx;
    if (const std::optional<gr::pmt::Value> v = held("device_parameter")) {
        ctx.deviceParams = v->value_or(std::string{});
    }
    if (const std::optional<gr::pmt::Value> v = held("sample_rate")) {
        ctx.sampleRate = numberIn(*v).value_or(0.0);
    }
    if (const std::optional<gr::pmt::Value> v = held("frequency")) {
        if (const auto* list = v->get_if<gr::Tensor<double>>(); list != nullptr && !list->empty()) {
            ctx.centerFreq = (*list)[0UZ];
        } else {
            ctx.centerFreq = numberIn(*v).value_or(0.0);
        }
    }
    if (const std::optional<gr::pmt::Value> v = held("gain_mode")) {
        ctx.gainMode = numberIn(*v).value_or(0.0) != 0.0;
    }
    if (const std::optional<gr::pmt::Value> v = held("wire_format")) {
        ctx.wireFormat = v->value_or(std::string{});
    }
    return ctx;
}

// The surface the block states: the probe's answer while it has not started, and its own
// otherwise.
template <typename TBlock>
std::expected<std::vector<ControlDesc>, std::string> surfaceOf(TBlock& blk) {
    if (unstarted(blk)) {
        return probeControls<TBlock>(TBlock::kDevice, TransmitBlock<TBlock>, stagedContext(blk));
    }
    return blk.controlSurface();
}

template <typename TBlock>
std::optional<gr::Message> answerControls(TBlock& blk, gr::Message message) {
    using enum gr::message::Command;
    if (message.cmd == Unsubscribe) {
        std::lock_guard lock(blk._controlProperties.mutex);
        blk._controlProperties.controlsClients.erase(message.clientRequestID);
        return std::nullopt;
    }
    if (message.cmd != Get && message.cmd != Subscribe) {
        return errorReply(std::move(message), "controls answers Get, Subscribe and Unsubscribe; a control is written through the control property");
    }
    /*| contract: a block that has not started holds no unit, so it answers what the discovery
            probe answers for the unit its settings name, and an error where the probe could
            not open that unit.
        trap: a family whose surface exists only on an open device opens the unit for the
            length of the probe, on the thread that serves the block's messages.
    */
    auto stated = surfaceOf(blk);
    if (!stated.has_value()) {
        return errorReply(std::move(message), std::format("controls: {}", stated.error()));
    }
    std::vector<ControlDesc> surface = std::move(*stated);
    if (message.cmd == Subscribe) {
        std::lock_guard lock(blk._controlProperties.mutex);
        blk._controlProperties.controlsClients.insert(message.clientRequestID);
        blk._controlProperties.lastSurface = surface;
    }
    message.data = controlsReply(surface);
    return message;
}

template <typename TBlock>
gr::Message answerControl(TBlock& blk, gr::Message message) {
    if (message.cmd != gr::message::Command::Set) {
        return errorReply(std::move(message), "control answers Set with an id and a value");
    }
    if (!message.data.has_value()) {
        return errorReply(std::move(message), "control: the request carries no id and value");
    }
    const std::string id = textField(*message.data, "id");
    const auto*       v  = field(*message.data, "value");
    const auto        requested = v == nullptr ? std::nullopt : numberIn(*v);
    if (!requested.has_value()) {
        return errorReply(std::move(message), std::format("control {}: the request carries no numeric value", id));
    }
    /*| contract: SAMPLE_RATE and FREQUENCY stage the setting of the same meaning, unbounded, and
            the block applies it as it applies any setting write: at its next settings drain,
            where it writes back the value the device took and refuses a frequency outside its
            coverage. The reply states the staged value and read_back false.
    */
    if (id == kSampleRateControl || id == kFrequencyControl) {
        const bool       rate = id == kSampleRateControl;
        gr::property_map setting;
        if (rate) {
            setting["sample_rate"] = *requested;
        } else {
            setting["frequency"] = std::vector<double>{*requested};
        }
        if (const gr::property_map refused = blk.settings().setStaged(setting); !refused.empty()) {
            return errorReply(std::move(message), std::format("control {}: the block refused the {} setting", id, rate ? "sample_rate" : "frequency"));
        }
        message.data = gr::property_map{{"id", id}, {"value", *requested}, {"requested", *requested}, {"read_back", false}};
        return message;
    }
    /*| contract: GAIN_MODE stages gain_mode, true for any value but zero, where the surface
            the block states carries the descriptor, and the block applies it at its next
            settings drain. The reply states the staged mode and read_back false. A block whose
            surface carries no GAIN_MODE refuses the id as it refuses any id it does not state.
    */
    if (id == kGainModeControl) {
        const auto stated = surfaceOf(blk);
        if (!stated.has_value()) {
            return errorReply(std::move(message), std::format("control {}: {}", id, stated.error()));
        }
        if (std::ranges::none_of(*stated, [](const ControlDesc& c) { return c.id == kGainModeControl; })) {
            return errorReply(std::move(message), std::format("control {}: the device states no control of that id", id));
        }
        const bool on = *requested != 0.0;
        if (const gr::property_map refused = blk.settings().setStaged({{"gain_mode", on}}); !refused.empty()) {
            return errorReply(std::move(message), std::format("control {}: the block refused the gain_mode setting", id));
        }
        message.data = gr::property_map{{"id", id}, {"value", on ? 1.0 : 0.0}, {"requested", *requested}, {"read_back", false}};
        return message;
    }
    const std::vector<ControlDesc> surface = blk.controlSurface();
    const auto                     desc    = std::ranges::find_if(surface, [&id](const ControlDesc& c) { return c.id == id; });
    if (desc == surface.end()) {
        return errorReply(std::move(message), std::format("control {}: the device states no control of that id", id));
    }
    if (!writeControl(blk, *desc, *requested)) {
        return errorReply(std::move(message), deviceIsUp(blk) ? std::format("control {}: the device refused {}", id, *requested) : std::format("control {}: the block holds no open device", id));
    }
    const std::optional<double> held = readBack(blk, *desc);
    message.data                     = gr::property_map{{"id", id}, {"value", held.value_or(clampToControl(*desc, *requested))}, {"requested", *requested}, {"read_back", held.has_value()}};
    publishControlSurface(blk);
    return message;
}

template <typename TBlock>
std::optional<gr::Message> answerSensors(TBlock& blk, gr::Message message) {
    using enum gr::message::Command;
    auto& state = blk._controlProperties;
    if (message.cmd == Subscribe || message.cmd == Unsubscribe) {
        std::lock_guard lock(state.mutex);
        if (message.cmd == Subscribe) {
            state.sensorsClients.insert(message.clientRequestID);
        } else {
            state.sensorsClients.erase(message.clientRequestID);
        }
        return std::nullopt;
    }
    if (message.cmd != Get) {
        return errorReply(std::move(message), "sensors answers Get, Subscribe and Unsubscribe");
    }
    /*| contract: the stream state first, then the device's readings while the device is up.
            A block whose device is down answers the stream state alone, and one whose device
            is up and did not answer the sweep answers an error.
    */
    const bool                 full = message.data.has_value() && textField(*message.data, "sweep") == "full";
    std::vector<SensorReading> readings{streamStateReading(blk)};
    if (deviceIsUp(blk)) {
        const auto swept = sweepSensors(blk, full);
        if (swept.has_value()) {
            readings.insert(readings.end(), swept->begin(), swept->end());
        } else if (deviceIsUp(blk)) {
            return errorReply(std::move(message), "sensors: the device did not answer");
        } else {
            readings = {streamStateReading(blk)};
        }
    }
    gr::property_map      body = sensorsReply(readings);
    std::set<std::string> clients;
    {
        std::lock_guard lock(state.mutex);
        clients = state.sensorsClients;
    }
    for (const auto& client : clients) {
        if (client != message.clientRequestID) {
            blk.emitMessage(property::kSensors, body, client);
        }
    }
    message.data = std::move(body);
    return message;
}

/*| contract: the family's listing, started block or not. A family whose listing opens a unit
        to name it states kListingOpens, and lists nothing unless the request carries
        {allow_open: true}; the reply says so under "opens" either way. A listing that throws
        answers an error naming the cause.
    trap: a listing enumerates a bus or a network on the thread that serves the block's
        messages, and a network listing can wait on every interface the vendor library reaches.
*/
template <typename TBlock>
gr::Message answerDevices(gr::Message message) {
    if (message.cmd != gr::message::Command::Get) {
        return errorReply(std::move(message), "devices answers Get");
    }
    const bool                                       allowOpen = message.data.has_value() && flagField(*message.data, "allow_open", false);
    std::vector<std::pair<std::string, std::string>> listing;
    if (!TBlock::kListingOpens || allowOpen) {
        try {
            listing = TBlock::enumerateDevices();
        } catch (const std::exception& e) {
            return errorReply(std::move(message), std::format("devices: the {} listing failed: {}", TBlock::kDevice, e.what()));
        }
    }
    message.data = devicesReply(listing, TBlock::kListingOpens);
    return message;
}

} // namespace detail

/*| contract: the answer to one message for any of the four properties, the reply the
        framework sends back to the caller, or nothing where the command takes no reply. A
        block's own property callback forwards here.
*/
template <typename TBlock>
std::optional<gr::Message> answerControlProperty(TBlock& blk, std::string_view name, gr::Message message) {
    if (name == property::kControls) {
        return detail::answerControls(blk, std::move(message));
    }
    if (name == property::kControl) {
        return detail::answerControl(blk, std::move(message));
    }
    if (name == property::kDevices) {
        return detail::answerDevices<TBlock>(std::move(message));
    }
    return detail::answerSensors(blk, std::move(message));
}

/*| contract: send the surface of the device the block has just opened to the controls
        subscribers it kept from an earlier run or from before its start. Each block calls this
        at the end of a start that opened its device. It also registers the four properties
        where the block has not, and a registration of a name already present changes nothing.
*/
template <typename TBlock>
void offerControlProperties(TBlock& blk) {
    using Callback = gr::BlockBase::PropertyCallback;
    for (const char* name : {property::kControls, property::kControl, property::kSensors, property::kDevices}) {
        blk.propertyCallbacks.try_emplace(name, static_cast<Callback>(&TBlock::controlProperty));
    }
    publishControlSurface(blk);
}

} // namespace capture
