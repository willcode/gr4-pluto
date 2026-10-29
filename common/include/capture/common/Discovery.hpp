/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/ControlDesc.hpp>

#include <algorithm>
#include <concepts>
#include <exception>
#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

/*| role: the devices attached to this host for every family a program compiles in, and the
        control surface of one of them, without a block and without a graph.
    frame: the families are the block types a program passes in, sources and sinks alike, so a
        program answers for exactly the vendor libraries it links. A family is named by its
        blocks' kDevice, the value their device setting carries. Each answer comes from the
        family's own static calls: enumerateDevices() for the listing, describeControls() for
        the surface, and probeTruth() first where the family states its surface from an open
        device.
    contract: the surface a probe answers is the one a running block of that family answers
        on its controls property for the same device at the same rate, before any control
        has moved.
    trap: a probe of a family with probeTruth opens the device for the length of the probe. A
        device a block is streaming from refuses that open or is disturbed by it, so a program
        holding a running block asks the block's controls property instead.
*/
namespace capture {

/*| role: one attached device as its family lists it.
    frame: device is the selector a block's device_parameter takes, and label the family's
        own name for the unit. receive and transmit say which blocks of the family the call was
        given, since every family lists the same units for both directions.
*/
struct DeviceListing {
    std::string family;
    std::string label;
    std::string device;
    bool        receive  = false;
    bool        transmit = false;

    bool operator==(const DeviceListing&) const = default;
};

/*| role: one family a program compiled in, and the directions it holds blocks for.
*/
struct CompiledFamily {
    std::string family;
    bool        receive  = false;
    bool        transmit = false;

    bool operator==(const CompiledFamily&) const = default;
};

/*| role: the blocks of one family as one type. A device directory's Discovery.hpp states its
        family's blocks in this form, and the discovery library's source joins those lists.
    frame: the order is the order the family gives, a source ahead of a sink.
*/
template <typename... Blocks>
struct BlockList {};

namespace detail {

template <typename TBlock>
concept TransmitBlock = requires(TBlock& b) { b.in; };

template <typename TBlock>
concept ProbedFamily = requires(const std::string& device, double rateHz, typename TBlock::Truth& truth) {
    { TBlock::probeTruth(device, rateHz, truth) } -> std::convertible_to<bool>;
};

} // namespace detail

/*| contract: every family the given blocks belong to, once each, in the order the blocks were
        given, with the directions of the blocks given for it. No device is asked.
*/
template <typename... Blocks>
std::vector<CompiledFamily> listFamilies() {
    std::vector<CompiledFamily> out;
    [[maybe_unused]] const auto one = [&]<typename TBlock>() {
        const std::string family = TBlock::kDevice;
        auto              it     = std::ranges::find(out, family, &CompiledFamily::family);
        if (it == out.end()) {
            out.push_back({.family = family});
            it = out.end() - 1;
        }
        (detail::TransmitBlock<TBlock> ? it->transmit : it->receive) = true;
    };
    (one.template operator()<Blocks>(), ...);
    return out;
}

/*| contract: every device the given families list, one entry per unit and family, in the
        order the families were given and each family's own order within it. A family's
        listing is taken once however many of its blocks were given. A family whose listing
        throws lists nothing, and the others still answer.
*/
template <typename... Blocks>
std::vector<DeviceListing> listDevices() {
    std::vector<DeviceListing> out;
    std::vector<std::string>   listed;
    [[maybe_unused]] const auto one = [&]<typename TBlock>() {
        const std::string family = TBlock::kDevice;
        if (std::ranges::find(listed, family) == listed.end()) {
            listed.push_back(family);
            try {
                for (auto& [label, device] : TBlock::enumerateDevices()) {
                    out.push_back({.family = family, .label = std::move(label), .device = std::move(device)});
                }
            } catch (const std::exception&) { // the family's library failed; the others still list
            }
        }
        for (auto& entry : out) {
            if (entry.family == family) {
                (detail::TransmitBlock<TBlock> ? entry.transmit : entry.receive) = true;
            }
        }
    };
    (one.template operator()<Blocks>(), ...);
    return out;
}

/*| contract: the control surface one device of one family states for a direction, at the rate
        and the center frequency the context names, with ctx.deviceParams naming the device as
        a listing does. The error names a family or a direction the given blocks do not carry,
        or a device the family's probe could not open.
    frame: a family with probeTruth opens the device through it, at the context's rate, and
        states its surface from what the device answered, since a unit's gain elements, filter
        range and reference choices can exist only there. A family without probeTruth states
        its surface from the context and what its own listing recorded about the unit.
*/
template <typename... Blocks>
std::expected<std::vector<ControlDesc>, std::string> probeControls(std::string_view family, bool transmit, const TruthContext& ctx) {
    std::optional<std::expected<std::vector<ControlDesc>, std::string>> answer;
    [[maybe_unused]] const auto one = [&]<typename TBlock>() {
        if (answer.has_value() || family != std::string_view(TBlock::kDevice) || detail::TransmitBlock<TBlock> != transmit) {
            return;
        }
        if constexpr (detail::ProbedFamily<TBlock>) {
            typename TBlock::Truth truth;
            if (TBlock::probeTruth(ctx.deviceParams, ctx.sampleRate, truth)) {
                answer = TBlock::describeControls(truth, ctx);
            } else {
                answer = std::unexpected(std::format("{}: the probe could not open the device '{}'", family, ctx.deviceParams));
            }
        } else {
            answer = TBlock::describeControls(ctx);
        }
    };
    (one.template operator()<Blocks>(), ...);
    if (!answer.has_value()) {
        return std::unexpected(std::format("{}: no {} of that family is compiled in", family, transmit ? "sink" : "source"));
    }
    return *answer;
}

} // namespace capture
