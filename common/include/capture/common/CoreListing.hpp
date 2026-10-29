/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/Discovery.hpp>

/*| role: a family's device listing in the form GNU Radio 4's device listing registry takes,
        and its registration there. A block library registers its family's listing when it
        loads, so a program that loads the library lists the family's units through
        gr::PluginLoader::listDevices() without holding a block type.
    frame: the declarations stand under CAPTURE_DEVICE_LISTINGS, which the configure defines
        where a probe against the core it found compiles and links a registration. Against a
        core without <gnuradio-4.0/DeviceListing.hpp> this header declares nothing, and a
        block library registers its blocks alone.
*/
#ifdef CAPTURE_DEVICE_LISTINGS

#include <gnuradio-4.0/DeviceListing.hpp>

#include <string_view>
#include <utility>
#include <vector>

namespace capture {

namespace detail {

/*| contract: the units the family of TBlock lists, as its enumerateDevices() gives them, each
        label and selector unchanged. An exception from the listing reaches the caller.
*/
template <typename TBlock>
std::vector<gr::DeviceUnit> coreUnits() {
    std::vector<gr::DeviceUnit> units;
    for (auto& [label, selector] : TBlock::enumerateDevices()) {
        units.push_back({.label = std::move(label), .selector = std::move(selector)});
    }
    return units;
}

} // namespace detail

/*| contract: the one listing of the family whose blocks a device directory's Discovery.hpp
        states: the blocks' kDevice as the family word, their kListingOpens as opens, receive
        where a source is given and transmit where a sink is, and the first block's
        enumerateDevices() as the function that lists the units.
    invariant: the blocks of one family share kDevice and kListingOpens, since both blocks
        list the same units through the same call.
    verified-by: listings.each-block-library-registers-its-family
*/
template <typename TFirst, typename... TRest>
gr::DeviceListing coreListing(BlockList<TFirst, TRest...> /*blocks*/) {
    static_assert(((std::string_view(TRest::kDevice) == std::string_view(TFirst::kDevice)) && ...), "the blocks of one family share its word");
    static_assert(((TRest::kListingOpens == TFirst::kListingOpens) && ...), "the blocks of one family list alike");
    return {.family    = TFirst::kDevice,
            .opens     = TFirst::kListingOpens,
            .receive   = !detail::TransmitBlock<TFirst> || (!detail::TransmitBlock<TRest> || ...),
            .transmit  = detail::TransmitBlock<TFirst> || (detail::TransmitBlock<TRest> || ...),
            .listUnits = &detail::coreUnits<TFirst>};
}

/*| contract: add the listing of the family TBlocks states to the process's device listing
        registry, and answer whether the registry took it. A registration of the same family
        word that came first holds the word, and the registry then refuses this one.
*/
template <typename TBlocks>
bool registerCoreListing() {
    return gr::globalDeviceListingRegistry().add(coreListing(TBlocks{}));
}

} // namespace capture

#endif // CAPTURE_DEVICE_LISTINGS
