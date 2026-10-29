/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/Discovery.hpp>
#include <capture/ad9361/Sink.hpp>
#include <capture/ad9361/Source.hpp>

namespace capture::ad9361 {

/*| role: the blocks the discovery library lists and probes for the AD9361 family: its source
        and its sink.
*/
using DiscoveryBlocks = capture::BlockList<Source, Sink>;

} // namespace capture::ad9361
