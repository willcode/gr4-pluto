/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: registers the AD9361 source and sink under their type names, and the family's
        device listing with the core's registry, for the block library the build generates
        from this header. The listing is the one the family's Discovery.hpp states, and the
        library registers it when it loads, where the configure defined
        CAPTURE_DEVICE_LISTINGS.
*/
#pragma once

#include <gnuradio-4.0/BlockRegistry.hpp>

#include <capture/common/CoreListing.hpp>
#include <capture/ad9361/Discovery.hpp>
#include <capture/ad9361/Sink.hpp>
#include <capture/ad9361/Source.hpp>

GR_REGISTER_BLOCK(capture::ad9361::Source)
GR_REGISTER_BLOCK(capture::ad9361::Sink)

#ifdef CAPTURE_DEVICE_LISTINGS
namespace capture::ad9361 {
inline const bool coreListingRegistered = capture::registerCoreListing<DiscoveryBlocks>();
} // namespace capture::ad9361
#endif
