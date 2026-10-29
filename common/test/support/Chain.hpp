/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/BlockModel.hpp>
#include <gnuradio-4.0/Runtime.hpp>

#include <expected>
#include <memory>
#include <string>
#include <utility>

namespace capture::test {

/*| role: two blocks of a case in a graph built by name, the first one's out port wired to the
        second one's in port, with a typed pointer to each.
    contract: chain() builds each block from its settings, adds it to a gr::RuntimeGraph and
        connects the two. The error is the graph's reason for the step that refused. A case
        hands graph to gr::Runtime::create(). The pointers stay valid while the Runtime holds
        the graph.
    why: a case reads its blocks by type while the graph is built by name, and so builds
        against every core whose Runtime takes a gr::RuntimeGraph.
*/
template <typename Up, typename Down>
struct Chain {
    gr::RuntimeGraph graph;
    Up*              up   = nullptr;
    Down*            down = nullptr;
};

template <typename Up, typename Down>
[[nodiscard]] std::expected<Chain<Up, Down>, gr::RuntimeError> chain(gr::property_map upSettings, gr::property_map downSettings) {
    Chain<Up, Down> out;
    auto            up   = std::make_shared<gr::BlockWrapper<Up>>(std::move(upSettings));
    auto            down = std::make_shared<gr::BlockWrapper<Down>>(std::move(downSettings));
    out.up               = std::addressof(up->blockRef());
    out.down             = std::addressof(down->blockRef());
    auto upHandle        = out.graph.add(std::move(up));
    if (!upHandle) {
        return std::unexpected(upHandle.error());
    }
    auto downHandle = out.graph.add(std::move(down));
    if (!downHandle) {
        return std::unexpected(downHandle.error());
    }
    if (auto wired = out.graph.connect(*upHandle, "out", *downHandle, "in"); !wired) {
        return std::unexpected(wired.error());
    }
    return out;
}

/*| contract: startRun() starts a runtime and answers whether the start was accepted. reason
        holds the core's refusal and is empty for an accepted start. It compiles against a
        start() that returns std::expected<void, gr::RuntimeError> and against one that returns
        std::optional<gr::RuntimeError>.
*/
struct Started {
    bool        accepted = false;
    std::string reason;
};

template <typename Run>
[[nodiscard]] Started startRun(Run& runtime) {
    auto answer = runtime.start();
    if constexpr (requires { answer.error(); }) {
        if (answer.has_value()) {
            return {.accepted = true, .reason = {}};
        }
        return {.accepted = false, .reason = answer.error().message};
    } else {
        if (!answer.has_value()) {
            return {.accepted = true, .reason = {}};
        }
        return {.accepted = false, .reason = answer->message};
    }
}

} // namespace capture::test
