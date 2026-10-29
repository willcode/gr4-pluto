/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the plain functions of capture/common/DiscoveryLibrary.hpp, over the families this
        build lists.
    frame: DiscoveryFragments.hpp is written into the build tree by capture_discovery_sources()
        of common/cmake/capture.cmake. It includes the Discovery.hpp of each family the build
        lists and names their block lists, in that order, as capture::discovery::Fragments.
        This source joins those lists into one. A build that lists no family answers empty
        lists and refuses every probe.
*/
#include <capture/common/DiscoveryLibrary.hpp>

#include <DiscoveryFragments.hpp>

namespace capture::discovery {
namespace {

// Joins two block lists, so each family's list adds its blocks to one list.
template <typename... A, typename... B>
BlockList<A..., B...> operator+(BlockList<A...>, BlockList<B...>) {
    return {};
}

template <typename... Lists>
auto joined(BlockList<Lists...>) -> decltype((BlockList<>{} + ... + Lists{}));

using Selected = decltype(joined(Fragments{}));

template <typename... Blocks>
std::vector<CompiledFamily> familiesOver(BlockList<Blocks...>) {
    return capture::listFamilies<Blocks...>();
}

template <typename... Blocks>
std::vector<DeviceListing> devicesOver(BlockList<Blocks...>) {
    return capture::listDevices<Blocks...>();
}

template <typename... Blocks>
std::expected<std::vector<ControlDesc>, std::string> probeOver(BlockList<Blocks...>, std::string_view family, bool transmit, const TruthContext& ctx) {
    return capture::probeControls<Blocks...>(family, transmit, ctx);
}

} // namespace

std::vector<CompiledFamily> families() { return familiesOver(Selected{}); }

std::vector<DeviceListing> listDevices() { return devicesOver(Selected{}); }

std::expected<std::vector<ControlDesc>, std::string> probeControls(std::string_view family, bool transmit, const TruthContext& ctx) { return probeOver(Selected{}, family, transmit, ctx); }

} // namespace capture::discovery
