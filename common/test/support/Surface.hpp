/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Properties.hpp>

#include <gnuradio-4.0/BlockModel.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Port.hpp>

#include <boost/ut.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace capture::test {

// The descriptor of one id in a surface, or nothing where the surface states none.
inline std::optional<ControlDesc> descriptorOf(const std::vector<ControlDesc>& surface, std::string_view id) {
    const auto it = std::ranges::find_if(surface, [id](const ControlDesc& c) { return c.id == id; });
    return it == surface.end() ? std::nullopt : std::optional<ControlDesc>(*it);
}

/*| contract: a surface states SAMPLE_RATE with the ladder given, anyInRange over the span given
        and bounded by the ladder's ends where no span is given, and FREQUENCY over the coverage
        given: the outer ends as min and max, and the spans themselves where there are several.
        Both are in hertz and neither is a gain.
*/
inline void expectRateAndFrequency(const std::vector<ControlDesc>& surface, const std::vector<double>& ladder, std::optional<std::pair<double, double>> span, const std::vector<std::pair<double, double>>& coverage,
                                   std::string_view who) {
    using namespace boost::ut;
    const auto rate = descriptorOf(surface, kSampleRateControl);
    expect(fatal(rate.has_value())) << who << " states SAMPLE_RATE";
    expect(rate->kind == 4 && rate->unit == "Hz" && !rate->isGain) << who << ": a list of rates in hertz";
    const bool sameLadder = rate->listValues == ladder;
    expect(sameLadder) << who << ": the ladder is the family's own";
    expect(rate->anyInRange == span.has_value()) << who << ": anyInRange where the device takes every rate in a span";
    expect(rate->min == (span ? span->first : ladder.front()) && rate->max == (span ? span->second : ladder.back())) << who << ": bounded by the span, or by the ladder's ends";
    const auto band = descriptorOf(surface, kFrequencyControl);
    expect(fatal(band.has_value())) << who << " states FREQUENCY";
    expect(band->kind == 0 && band->unit == "Hz" && !band->isGain) << who << ": a range in hertz";
    auto sorted = coverage;
    std::ranges::sort(sorted);
    expect(band->min == sorted.front().first && band->max == std::ranges::max(sorted, {}, &std::pair<double, double>::second).second) << who << ": the outer ends of the coverage";
    const bool spans = band->ranges == (sorted.size() > 1UZ ? sorted : std::vector<std::pair<double, double>>{});
    expect(spans) << who << ": the spans where there are several, and none for one";
}

// The readings a block's sensors property answers with, asked on the case's own thread.
template <typename Block>
std::vector<SensorReading> sensorsAnswer(Block& blk) {
    gr::Message request;
    request.cmd      = gr::message::Command::Get;
    request.endpoint = property::kSensors;
    const auto reply = answerControlProperty(blk, property::kSensors, std::move(request));
    return reply.has_value() && reply->data.has_value() ? sensorsFromReply(*reply->data) : std::vector<SensorReading>{};
}

/*| contract: a block whose device is down answers its sensors with its stream state alone,
        under the id of its direction, in the word given, not good and on a glance row.
*/
template <typename Block>
void expectStreamStateAlone(Block& blk, std::string_view word, std::string_view who) {
    using namespace boost::ut;
    const auto readings = sensorsAnswer(blk);
    expect(fatal(readings.size() == 1UZ)) << who << ": the sensors answer the stream state alone";
    const SensorReading& state = readings.front();
    expect(state.id == (detail::TransmitBlock<Block> ? kTxStreamStateReading : kStreamStateReading)) << who << ": under the id of its direction";
    expect(state.value == word && !state.good && state.brief) << who << ": reads " << word << ", not good, on a glance row; it reads " << state.value;
}

/*| contract: the family states GAIN_MODE where present is true: a toggle that is not a gain,
        whose default is the mode the context holds. A control write of it on an unstarted
        block stages gain_mode, and the surface the block answers next carries the staged mode.
        A family with present false states it in no surface, refuses the write and stages
        nothing.
*/
template <typename Block>
void expectGainMode(bool present, std::string_view who) {
    using namespace boost::ut;
    for (const bool on : {false, true}) {
        TruthContext ctx;
        ctx.gainMode       = on;
        const auto surface = probeControls<Block>(Block::kDevice, detail::TransmitBlock<Block>, ctx);
        expect(fatal(surface.has_value())) << who << " answers the probe";
        const auto mode = descriptorOf(*surface, kGainModeControl);
        expect(fatal(mode.has_value() == present)) << who << (present ? " states GAIN_MODE" : " states no GAIN_MODE");
        if (mode.has_value()) {
            expect(mode->kind == 2 && !mode->isGain && mode->defValue == (on ? 1.0 : 0.0)) << who << ": a toggle and not a gain, at the mode the context holds";
        }
    }

    gr::BlockWrapper<Block> model;
    model.init(std::make_shared<gr::Sequence>());
    auto&       blk = model.blockRef();
    gr::Message write;
    write.cmd          = gr::message::Command::Set;
    write.endpoint     = property::kControl;
    write.data         = gr::property_map{{"id", std::string(kGainModeControl)}, {"value", 1.0}};
    const auto reply   = answerControlProperty(blk, property::kControl, std::move(write));
    const auto staged  = blk.settings().stagedParameters();
    const auto* held   = detail::field(staged, "gain_mode");
    const bool  stages = held != nullptr && detail::numberIn(*held).value_or(0.0) == 1.0;
    if (!present) {
        expect(reply.has_value() && !reply->data.has_value() && reply->data.error().message.find("no control") != std::string::npos) << who << ": the write is refused as an id the surface does not state";
        expect(held == nullptr) << who << ": and stages nothing";
        return;
    }
    expect(fatal(reply.has_value() && reply->data.has_value())) << who << ": the write is answered before any start";
    expect(detail::numberField(*reply->data, "value", -1.0) == 1.0 && !detail::flagField(*reply->data, "read_back", true)) << who << ": with the staged mode, not read back";
    expect(stages) << who << ": as the gain_mode setting, staged for the start";
    gr::Message ask;
    ask.cmd              = gr::message::Command::Get;
    ask.endpoint         = property::kControls;
    const auto answered  = answerControlProperty(blk, property::kControls, std::move(ask));
    const auto surface   = answered.has_value() && answered->data.has_value() ? controlsFromReply(*answered->data) : std::vector<ControlDesc>{};
    const auto mode      = descriptorOf(surface, kGainModeControl);
    expect(mode.has_value() && mode->defValue == 1.0) << who << ": and the next surface states the staged mode";
}

/*| role: a block asked through its model's two message ports on the case's own thread, with
        no scheduler, as a program that holds no block type asks a block it has not started.
    contract: ask() writes one request to the block's message input, has the block process its
        messages, and answers the reply written for that request, or nothing where none was.
*/
class DirectAsker {
public:
    explicit DirectAsker(gr::BlockModel& block) : _block(block) {
        _connected = _block.msgOut->connect(_replies).has_value() && _requests.connect(*_block.msgIn).has_value();
    }

    [[nodiscard]] bool connected() const { return _connected; }

    std::optional<gr::Message> ask(gr::message::Command cmd, std::string_view endpoint, gr::property_map data, std::string client = "case") {
        gr::Message request;
        request.cmd             = cmd;
        request.endpoint        = std::string(endpoint);
        request.clientRequestID = client;
        request.data            = std::move(data);
        {
            auto span = _requests.streamWriter().template reserve<gr::SpanReleasePolicy::ProcessAll>(1UZ);
            span[0]   = std::move(request);
        }
        _block.processScheduledMessages();
        auto                       replies = _replies.streamReader().get();
        std::optional<gr::Message> found;
        for (const gr::Message& m : replies) {
            if (m.endpoint == endpoint && m.clientRequestID == client) {
                found = m;
            }
        }
        std::ignore = replies.consume(replies.size());
        return found;
    }

private:
    gr::BlockModel& _block;
    gr::MsgPortOut  _requests;
    gr::MsgPortIn   _replies;
    bool            _connected = false;
};

} // namespace capture::test
