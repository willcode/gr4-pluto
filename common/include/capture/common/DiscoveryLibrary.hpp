/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/Discovery.hpp>

#include <expected>
#include <string>
#include <string_view>
#include <vector>

/*| role: the family listing, the device listing and the control probe of Discovery.hpp as
        plain functions, for a program that holds no block type. A program that loads the
        blocks as plugins calls these, and so does a language binding.
    frame: common/src/common/DiscoveryLibrary.cpp defines them over the families its build
        lists. Each family states its blocks in the Discovery.hpp of its own directory, and
        the build gives the source the list of those headers. A library built from it links
        the vendor libraries of those families and no other. Each answer is the answer of the
        template of the same name over every block of the listed families, in the order of the
        list.
    trap: a process holds one library built from that source. Two of them define the same
        functions, and the dynamic linker binds each name to one of the two.
*/
namespace capture::discovery {

/*| contract: the families the library was built with, each with the directions it carries
        blocks for. No device is asked.
*/
std::vector<CompiledFamily> families();

/*| contract: every device the library's families list, as listDevices gives it over their
        blocks.
    trap: a family's listing enumerates its bus or its network, and a network listing can
        broadcast on every interface the vendor library reaches.
*/
std::vector<DeviceListing> listDevices();

/*| contract: the control surface one device states for a direction, as probeControls gives it
        over the library's blocks. The error names a family the library was not built with,
        and a direction the family carries no block for, as the template does.
*/
std::expected<std::vector<ControlDesc>, std::string> probeControls(std::string_view family, bool transmit, const TruthContext& ctx);

} // namespace capture::discovery
