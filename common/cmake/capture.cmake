# Copyright 2026 Jeff Long
# SPDX-License-Identifier: MIT

# The build logic of a top of this tree: the options, the interface targets, the
# vendor-neutral lookups, capture_device(), capture_test(), capture_discovery_sources() and
# capture_finish(). A top includes this file after project() and after it sets
# CAPTURE_DEVICES, the device directories it carries. It then adds each device directory,
# whose CMakeLists.txt calls capture_device() and capture_test(), and calls capture_finish()
# once after the last one. The top keeps its own libraries, its install rules and its package.
#
# The variables a top may set ahead of the include:
# - CAPTURE_PACKAGE, the name of the package the top installs, with CAPTURE_PACKAGE_BLOCKS,
#   the name gr_add_block_library() gives the device's registration. A device's block
#   library is then <package>-blocks, exported as blocks, and CAPTURE_INSTALL_INCLUDEDIR is
#   include/<package>. Unset, a device's block library is capture-<device>-blocks, exported
#   as <device>-blocks, and CAPTURE_INSTALL_INCLUDEDIR is include.
# - CAPTURE_TEST_LABEL, capture by default, the label of every case capture_test() adds.
# - CAPTURE_UT_VARIABLE, CAPTURE_UT_INCLUDE_DIR by default, the setting the configure asks
#   a user for where it finds no boost/ut.hpp.

include(GNUInstallDirs)

# The root of the tree: the directory holding common/.
get_filename_component(CAPTURE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

if(CAPTURE_PACKAGE)
  set(CAPTURE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/${CAPTURE_PACKAGE})
else()
  set(CAPTURE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
endif()
if(NOT CAPTURE_TEST_LABEL)
  set(CAPTURE_TEST_LABEL capture)
endif()
if(NOT CAPTURE_UT_VARIABLE)
  set(CAPTURE_UT_VARIABLE CAPTURE_UT_INCLUDE_DIR)
endif()

if(PROJECT_IS_TOP_LEVEL)
  set(CAPTURE_TESTING_DEFAULT ON)
else()
  set(CAPTURE_TESTING_DEFAULT OFF)
endif()
option(CAPTURE_ENABLE_TESTING "Build the capture test suite" ${CAPTURE_TESTING_DEFAULT})

find_package(Threads REQUIRED)

# What this project compiles with when it compiles its own code. PRIVATE, so no
# consumer inherits them. They serve the header parse below: an unused variable
# or a wrong constant in a driver is a diagnostic only where something asks for
# diagnostics.
set(CAPTURE_WARNINGS -Wall -Wextra)

# The include directories and the language level of both tiers. In the build tree the
# include directories are common/include/ and every device's include/. Installed, the
# include directory is CAPTURE_INSTALL_INCLUDEDIR under the prefix, and the headers sit
# under its capture/.
add_library(capture-interface INTERFACE)
add_library(capture::headers ALIAS capture-interface)
set_target_properties(capture-interface PROPERTIES EXPORT_NAME headers)
target_include_directories(capture-interface INTERFACE $<BUILD_INTERFACE:${CAPTURE_ROOT}/common/include>)
foreach(device IN LISTS CAPTURE_DEVICES)
  target_include_directories(capture-interface INTERFACE $<BUILD_INTERFACE:${CAPTURE_ROOT}/${device}/include>)
endforeach()
target_include_directories(capture-interface INTERFACE $<INSTALL_INTERFACE:${CAPTURE_INSTALL_INCLUDEDIR}>)
# On the TARGET and not as a directory variable: the source headers are consumed
# without linking this library at all, and a requirement stated only in
# CMAKE_CXX_STANDARD reaches nobody outside this directory.
target_compile_features(capture-interface INTERFACE cxx_std_23)
target_link_libraries(capture-interface INTERFACE Threads::Threads)

# ---- the device headers, parsed ------------------------------------------------------
#
# <device>/include/capture/<device>/Source.hpp and Sink.hpp are thousands of lines
# of driver that nothing in this repository otherwise compiles. Without this target, a
# syntax error, an unused variable or a wrong constant in any of them reaches a consumer
# that happens to have GNU Radio 4 and the right vendor headers, and a ten-bit scale four
# times too small cost exactly that.
#
# capture-headers is one translation unit per device header, each behind the same
# detection a consumer does: a header whose vendor library is absent is not parsed, and
# the configure output says which. It links nothing and produces no artifact anybody
# uses; the objects exist so that the compiler has to have an opinion.
# These are GNU Radio 4 blocks: gr::Block's settings machinery, the lifecycle states
# and the tag vocabulary are all 4.x. The version is named here for the same reason a
# device's vendor minimum is, an older package being a compile failure in whichever
# consumer reached it first.
find_package(gnuradio4 4.0 CONFIG QUIET)
set(CAPTURE_GR4 "")
if(TARGET gnuradio4::gnuradio-core)
  list(APPEND CAPTURE_GR4 gnuradio4::gnuradio-core)
endif()
if(TARGET gnuradio4::gnuradio-blocklib-core)
  list(APPEND CAPTURE_GR4 gnuradio4::gnuradio-blocklib-core)
endif()

# ---- the block labels ----------------------------------------------------------------
#
# A device block describes itself to the core with labels: its family, its role, what it
# holds and the media it takes in or puts out, declared through gr::block::describe() of
# <gnuradio-4.0/BlockAttributes.hpp>. A core may carry an earlier form of the header under
# the same name and include guard, and __has_include cannot tell that form from the label
# form. The configure compiles a probe against the core it found, anew at each run, and
# defines CAPTURE_BLOCK_LABELS where the probe compiles. A device header includes the header
# and declares its labels under that macro alone, so its blocks build without labels against
# any other core. The definition is part of capture-interface in the build tree and in the
# installed package alike, so a program built on the installed package compiles each block
# type with the labels its installed block library carries.
set(CAPTURE_BLOCK_LABELS FALSE)
if(CAPTURE_GR4)
  include(CheckCXXSourceCompiles)
  include(CMakePushCheckState)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_LIBRARIES ${CAPTURE_GR4})
  set(CMAKE_REQUIRED_QUIET ON)
  set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
  unset(CAPTURE_CORE_HAS_BLOCK_LABELS CACHE)
  check_cxx_source_compiles(
    [[
#include <gnuradio-4.0/BlockAttributes.hpp>
struct Probe {
    static constexpr auto attributes = gr::block::describe(1U, gr::block::labels::holds::device);
};
static_assert(gr::block::attributesOf<Probe>().labels.size() == 1U);
int main() { return 0; }
]]
    CAPTURE_CORE_HAS_BLOCK_LABELS)
  cmake_pop_check_state()
  set(CAPTURE_BLOCK_LABELS ${CAPTURE_CORE_HAS_BLOCK_LABELS})
endif()
if(CAPTURE_BLOCK_LABELS)
  target_compile_definitions(capture-interface INTERFACE CAPTURE_BLOCK_LABELS)
  message(STATUS "capture: block labels DECLARED (the core has gr::block::describe)")
else()
  message(STATUS "capture: block labels NOT declared (the core has no gr::block::describe)")
endif()

# ---- the device listings -------------------------------------------------------------
#
# A block library registers its family's device listing with the core's device listing
# registry when it loads, so a program that loads the library lists the family's units through
# gr::PluginLoader::listDevices(). The core offers the registry in
# <gnuradio-4.0/DeviceListing.hpp>. The configure compiles and links a probe of it against the
# core it found, anew at each run, and defines CAPTURE_DEVICE_LISTINGS where the probe builds.
# common/include/capture/common/CoreListing.hpp and each registration header declare the
# registration under that macro alone, so against any other core a block library registers
# its blocks alone. The installed interface target carries the definition as the labels'.
set(CAPTURE_DEVICE_LISTINGS FALSE)
if(CAPTURE_GR4)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_LIBRARIES ${CAPTURE_GR4})
  set(CMAKE_REQUIRED_QUIET ON)
  unset(CAPTURE_CORE_HAS_DEVICE_LISTINGS CACHE)
  check_cxx_source_compiles(
    [[
#include <gnuradio-4.0/DeviceListing.hpp>
#include <vector>
std::vector<gr::DeviceUnit> probeUnits() { return {}; }
int main() {
    return gr::globalDeviceListingRegistry().add({.family = "probe", .opens = false, .receive = true, .transmit = false, .listUnits = probeUnits}) ? 0 : 1;
}
]]
    CAPTURE_CORE_HAS_DEVICE_LISTINGS)
  cmake_pop_check_state()
  set(CAPTURE_DEVICE_LISTINGS ${CAPTURE_CORE_HAS_DEVICE_LISTINGS})
endif()
if(CAPTURE_DEVICE_LISTINGS)
  target_compile_definitions(capture-interface INTERFACE CAPTURE_DEVICE_LISTINGS)
  message(STATUS "capture: device listings REGISTERED (the core has gr::globalDeviceListingRegistry)")
else()
  message(STATUS "capture: device listings NOT registered (the core has no gr::globalDeviceListingRegistry)")
endif()

if(CAPTURE_GR4)
  set(CAPTURE_HEADER_PARSE_DEFAULT ON)
else()
  set(CAPTURE_HEADER_PARSE_DEFAULT OFF)
endif()
if(NOT PROJECT_IS_TOP_LEVEL AND NOT CAPTURE_ENABLE_TESTING)
  set(CAPTURE_HEADER_PARSE_DEFAULT OFF)
endif()
option(CAPTURE_ENABLE_HEADER_PARSE "Compile one translation unit per device header"
       ${CAPTURE_HEADER_PARSE_DEFAULT})

# A device header can include its vendor header by the plain name the vendor's own
# examples use while the vendor installs it under a subdirectory of the include prefix.
# This target carries every directory the library was configured against, so a consumer
# of capture/<device>/*.hpp links it.
# It carries every device, sources and sinks alike, and the name says so. Each device
# directory adds its own vendor directory through capture_device() below.
add_library(capture-device-headers INTERFACE)
add_library(capture::device-headers ALIAS capture-device-headers)
target_link_libraries(capture-device-headers INTERFACE capture-interface)

# The discovery library's option, stated ahead of the device directories because
# capture_device() reads it. capture_finish() builds the library.
option(CAPTURE_ENABLE_DISCOVERY_LIBRARY "Build the device discovery library" ${CAPTURE_HEADER_PARSE_DEFAULT})

# ---- the block libraries -------------------------------------------------------------
#
# A device's include/gnuradio-4.0/capture/<device>/<Device>Blocks.hpp registers each block
# of the device under its type name. gr_add_block_library() generates the registration
# units from that header into an object library, and capture_device() builds a shared
# library, named as the top of this file states, from those objects into
# lib/gnuradio-4/plugins of the build tree. A program that loads plugins from that directory
# creates the blocks by name. The libraries are on by default where GNU Radio 4's
# block-library macros are found and this is the top-level project or a top that sets
# CAPTURE_PACKAGE. gr_add_block_library() also installs a generated registration header
# under any condition, so the option is refused under any other top: the header would
# install into the including project's prefix, and the library would not. A top that sets
# CAPTURE_PACKAGE installs its block library itself.
set(CAPTURE_BLOCK_LIBRARIES_MISSING "")
if(NOT CAPTURE_GR4)
  set(CAPTURE_BLOCK_LIBRARIES_MISSING "no gnuradio4")
else()
  find_package(GnuRadioBlockLib CONFIG QUIET)
  if(NOT GnuRadioBlockLib_FOUND)
    set(CAPTURE_BLOCK_LIBRARIES_MISSING "no GnuRadioBlockLib")
  endif()
endif()
set(CAPTURE_BLOCK_LIBRARIES_DEFAULT OFF)
if((PROJECT_IS_TOP_LEVEL OR CAPTURE_PACKAGE) AND NOT CAPTURE_BLOCK_LIBRARIES_MISSING)
  set(CAPTURE_BLOCK_LIBRARIES_DEFAULT ON)
endif()
option(CAPTURE_ENABLE_BLOCK_LIBRARIES "Build a GNU Radio 4 block library per device found"
       ${CAPTURE_BLOCK_LIBRARIES_DEFAULT})
if(CAPTURE_ENABLE_BLOCK_LIBRARIES AND NOT PROJECT_IS_TOP_LEVEL AND NOT CAPTURE_PACKAGE)
  message(
    FATAL_ERROR
      "capture: CAPTURE_ENABLE_BLOCK_LIBRARIES needs capture as the top-level project, because gr_add_block_library() installs each include/gnuradio-4.0/Capture<Device>Blocks.hpp into the including project's prefix while the block libraries install nowhere."
  )
endif()
set(CAPTURE_BLOCK_LIBRARIES_REASON "")
if(NOT CAPTURE_ENABLE_BLOCK_LIBRARIES)
  set(CAPTURE_BLOCK_LIBRARIES_REASON "CAPTURE_ENABLE_BLOCK_LIBRARIES is off")
  if(CAPTURE_BLOCK_LIBRARIES_MISSING)
    string(APPEND CAPTURE_BLOCK_LIBRARIES_REASON "; ${CAPTURE_BLOCK_LIBRARIES_MISSING}")
  endif()
elseif(CAPTURE_BLOCK_LIBRARIES_MISSING)
  set(CAPTURE_BLOCK_LIBRARIES_REASON "${CAPTURE_BLOCK_LIBRARIES_MISSING}")
endif()

# capture_device(<device> INCLUDE <directories> LIBRARY <libraries> [VERDICT <bool>])
#
# Each device directory calls this once its vendor lookup is done. INCLUDE is the vendor
# include directory the device's headers need and LIBRARY the vendor library that a
# block's start, stop and destructor call; an empty or NOTFOUND value means that part is
# absent. VERDICT, where given, is the device's own judgment of what it found, a version
# minimum for one; a false verdict makes the include directory absent as well. The call
# hands the include directory to capture::device-headers and writes a parse unit for the
# device's Source.hpp and Sink.hpp where that directory is present. It enters the device
# into the discovery library, and builds its block library, where the include directory
# and the library are present and the verdict holds. The units come from the tree: a
# Source.hpp or a Sink.hpp under a device's directory gets a translation unit written into
# the build tree, so a device added is parsed without a list to keep. A device enters the
# discovery library through include/capture/<device>/Discovery.hpp, its fragment, and a
# device without one stays out. The global property CAPTURE_BLOCKS_BUILT names each block
# library built, for the top's install rules.
function(capture_device device)
  cmake_parse_arguments(D "" "VERDICT" "INCLUDE;LIBRARY" ${ARGN})
  set(include "")
  foreach(dir IN LISTS D_INCLUDE)
    if(dir)
      list(APPEND include ${dir})
    endif()
  endforeach()
  set(absent "")
  if(NOT include)
    set(absent "vendor header missing")
  elseif(NOT D_LIBRARY)
    set(absent "vendor library missing")
  elseif(DEFINED D_VERDICT AND NOT D_VERDICT)
    set(absent "vendor check failed")
  endif()
  if(DEFINED D_VERDICT AND NOT D_VERDICT)
    set(include "")
  endif()
  foreach(dir IN LISTS include)
    target_include_directories(capture-device-headers SYSTEM INTERFACE ${dir})
  endforeach()

  if(CAPTURE_ENABLE_HEADER_PARSE)
    if(NOT include)
      set_property(GLOBAL APPEND PROPERTY CAPTURE_HEADER_SKIPPED "${device} (${absent})")
    else()
      foreach(role Source Sink)
        set(header ${CMAKE_CURRENT_SOURCE_DIR}/include/capture/${device}/${role}.hpp)
        if(NOT EXISTS ${header})
          continue()
        endif()
        set(unit ${PROJECT_BINARY_DIR}/headers/parse_${device}_${role}.cpp)
        set(body
            "// Written by the build from ${device}/include/capture/${device}/${role}.hpp.\n#include <capture/${device}/${role}.hpp>\n"
        )
        set(held "")
        if(EXISTS ${unit})
          file(READ ${unit} held)
        endif()
        # Written only where it differs, so a configure does not rebuild what it did not change.
        if(NOT held STREQUAL body)
          file(WRITE ${unit} "${body}")
        endif()
        set_property(GLOBAL APPEND PROPERTY CAPTURE_HEADER_UNITS ${unit})
      endforeach()
    endif()
  endif()

  if(absent)
    set_property(GLOBAL APPEND PROPERTY CAPTURE_BLOCKS_SKIPPED "${device} (${absent})")
    return()
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/capture/${device}/Discovery.hpp)
    set_property(GLOBAL APPEND PROPERTY CAPTURE_DISCOVERY_FAMILIES ${device})
    set_property(GLOBAL APPEND PROPERTY CAPTURE_DISCOVERY_LIBS ${D_LIBRARY})
  endif()

  if(CAPTURE_BLOCK_LIBRARIES_REASON)
    return()
  endif()
  string(SUBSTRING ${device} 0 1 first)
  string(SUBSTRING ${device} 1 -1 rest)
  string(TOUPPER ${first} first)
  if(CAPTURE_PACKAGE)
    set(stem ${CAPTURE_PACKAGE_BLOCKS})
    set(library ${CAPTURE_PACKAGE}-blocks)
    set(exported blocks)
    set(alias ${CAPTURE_PACKAGE}::blocks)
  else()
    set(stem Capture${first}${rest}Blocks)
    set(library capture-${device}-blocks)
    set(exported ${device}-blocks)
    set(alias capture::${device}-blocks)
  endif()
  gr_add_block_library(${stem} HEADERS include/gnuradio-4.0/capture/${device}/${first}${rest}Blocks.hpp
                       LINK_LIBRARIES capture-interface)
  if(NOT TARGET ${stem}Object)
    set_property(GLOBAL APPEND PROPERTY CAPTURE_BLOCKS_SKIPPED "${device} (the registration was not generated)")
    return()
  endif()
  target_include_directories(${stem}Object SYSTEM PRIVATE ${include})
  target_compile_options(${stem}Object PRIVATE ${CAPTURE_WARNINGS})

  add_library(${library} SHARED $<TARGET_OBJECTS:${stem}Object>)
  add_library(${alias} ALIAS ${library})
  set_target_properties(${library} PROPERTIES EXPORT_NAME ${exported} LIBRARY_OUTPUT_DIRECTORY
                                                                      ${PROJECT_BINARY_DIR}/lib/gnuradio-4/plugins)
  target_link_libraries(${library} PUBLIC gnuradio4::gnuradio-core gnuradio4::gnuradio-blocklib-core capture-interface
                        PRIVATE ${D_LIBRARY})
  set_property(GLOBAL APPEND PROPERTY CAPTURE_BLOCKS_BUILT ${library})
endfunction()

# ---- the suite -----------------------------------------------------------------------
#
# Hardware-free end to end: no test here opens a device, and the ones whose subject is a
# device driver reach it through the pure functions and the plain decisions the drivers
# were factored into for exactly that reason. Each device directory adds its own cases;
# the top adds the rest.
#
# Boost.UT is a single header and is not vendored here. Point CAPTURE_UT_INCLUDE_DIR at
# the directory holding boost/ut.hpp, or install a package that provides it; without one
# the suite is skipped and the library still builds.
set(CAPTURE_TESTS FALSE)
if(CAPTURE_ENABLE_TESTING)
  set(CAPTURE_UT_INCLUDE_DIR "" CACHE PATH "Directory holding boost/ut.hpp")
  if(NOT CAPTURE_UT_INCLUDE_DIR)
    find_path(CAPTURE_UT_FOUND_DIR boost/ut.hpp)
    if(CAPTURE_UT_FOUND_DIR)
      set(CAPTURE_UT_INCLUDE_DIR ${CAPTURE_UT_FOUND_DIR})
    endif()
  endif()
  if(CAPTURE_UT_INCLUDE_DIR)
    set(CAPTURE_TESTS TRUE)
    enable_testing()
  else()
    message(STATUS "capture: test suite SKIPPED (set ${CAPTURE_UT_VARIABLE} to boost/ut.hpp's directory)")
  endif()
endif()

if(CAPTURE_TESTS)
  add_library(capture-ut INTERFACE)
  target_include_directories(capture-ut SYSTEM INTERFACE ${CAPTURE_UT_INCLUDE_DIR})
  target_include_directories(capture-ut INTERFACE ${CAPTURE_ROOT}/common/test)
  target_compile_features(capture-ut INTERFACE cxx_std_23)

  # capture_test(<binary> [DIR <subdirectory>] CASES <ctest name>... [SOURCES ...] [LIBS ...]
  #              [INCLUDES ...] [DEFINITIONS ...])
  #
  # One binary per subject and one ctest entry per case, the case being named on the
  # command line, so a failure says which fact stopped holding without a reader having to
  # open the binary, and the compile cost of a heavy header is paid once per subject rather
  # than once per assertion. A name that matches no case in its binary is a failure and not
  # a quiet pass; see common/test/support/Cases.hpp.
  #
  # DIR names the directory the case file sits in, relative to the calling directory: test
  # for a device's own cases. The binary keeps its name whichever directory its source came
  # from, because the bench scripts and the verified-by lines name it. SOURCES names further
  # files of this tree compiled into the binary.
  function(capture_test name)
    cmake_parse_arguments(T "" "DIR" "CASES;SOURCES;LIBS;INCLUDES;DEFINITIONS" ${ARGN})
    set(source ${name}.cpp)
    if(T_DIR)
      set(source ${T_DIR}/${name}.cpp)
    endif()
    add_executable(${name} ${source} ${T_SOURCES})
    target_link_libraries(${name} PRIVATE capture-interface capture-ut ${T_LIBS})
    target_compile_options(${name} PRIVATE ${CAPTURE_WARNINGS})
    target_compile_definitions(${name} PRIVATE ${T_DEFINITIONS})
    foreach(dir ${T_INCLUDES})
      target_include_directories(${name} SYSTEM PRIVATE ${dir})
    endforeach()
    foreach(case ${T_CASES})
      add_test(NAME ${case} COMMAND ${name} ${case})
      # Every case here is arithmetic or a few thousand samples. The ceiling is a stuck
      # test, not a slow one.
      set_tests_properties(${case} PROPERTIES TIMEOUT 15 LABELS ${CAPTURE_TEST_LABEL})
      # A case that builds a graph, as a gr::Graph or a gr::RuntimeGraph, loads every
      # block library in the directories GNURADIO4_PLUGIN_DIRECTORIES names, or in the
      # core's installed plugin directory where the variable is unset. An installed
      # library can link another build of a vendor library than this tree's. Two builds
      # of one vendor library in one process can abort it at exit. Each case loads this
      # tree's own block libraries.
      set_property(TEST ${case} PROPERTY ENVIRONMENT_MODIFICATION
                                         "GNURADIO4_PLUGIN_DIRECTORIES=set:${PROJECT_BINARY_DIR}/lib/gnuradio-4/plugins")
    endforeach()
  endfunction()
endif()

# capture_discovery_sources(<target> <device>...)
#
# Compiles common/src/common/DiscoveryLibrary.cpp into <target> over the given devices, in
# the order given. The call writes DiscoveryFragments.hpp into a directory of the build tree
# that only <target> includes: one include of each device's
# include/capture/<device>/Discovery.hpp, and capture::discovery::Fragments, the list of
# their block lists. A test builds the source over its own family this way, and
# capture_finish() builds the discovery library over every device found. The target links
# the vendor libraries itself.
function(capture_discovery_sources target)
  set(dir ${CMAKE_CURRENT_BINARY_DIR}/${target}-discovery)
  set(body "// Written by the build: the discovery fragments of the devices ${target} lists.\n")
  string(APPEND body "#pragma once\n\n#include <capture/common/Discovery.hpp>\n")
  set(lists "")
  foreach(device IN LISTS ARGN)
    string(APPEND body "#include <capture/${device}/Discovery.hpp>\n")
    list(APPEND lists "capture::${device}::DiscoveryBlocks")
  endforeach()
  list(JOIN lists ", " joined)
  string(APPEND body "\nnamespace capture::discovery {\nusing Fragments = capture::BlockList<${joined}>;\n}\n")
  set(held "")
  if(EXISTS ${dir}/DiscoveryFragments.hpp)
    file(READ ${dir}/DiscoveryFragments.hpp held)
  endif()
  if(NOT held STREQUAL body)
    file(WRITE ${dir}/DiscoveryFragments.hpp "${body}")
  endif()
  target_sources(${target} PRIVATE ${CAPTURE_ROOT}/common/src/common/DiscoveryLibrary.cpp)
  target_include_directories(${target} PRIVATE ${dir})
endfunction()

# capture_finish(<discovery target> <discovery alias>)
#
# Called once after the last device directory. It prints one line naming the block
# libraries built and the devices skipped, builds capture-headers from the parse units the
# device directories wrote, and builds the discovery library as <discovery target>.
function(capture_finish discovery alias)
  get_property(built GLOBAL PROPERTY CAPTURE_BLOCKS_BUILT)
  get_property(skipped GLOBAL PROPERTY CAPTURE_BLOCKS_SKIPPED)
  string(REPLACE ";" " " built_line "${built}")
  string(REPLACE ";" ", " skipped_line "${skipped}")
  if(NOT built_line)
    set(built_line "none")
  endif()
  if(NOT skipped_line)
    set(skipped_line "none")
  endif()
  if(CAPTURE_BLOCK_LIBRARIES_REASON)
    message(STATUS "capture: block libraries NOT built (${CAPTURE_BLOCK_LIBRARIES_REASON})")
  else()
    message(STATUS "capture: block libraries BUILT ${built_line}; SKIPPED ${skipped_line}")
  endif()

  if(CAPTURE_ENABLE_HEADER_PARSE)
    get_property(units GLOBAL PROPERTY CAPTURE_HEADER_UNITS)
    get_property(unparsed GLOBAL PROPERTY CAPTURE_HEADER_SKIPPED)
    if(units)
      add_library(capture-headers OBJECT ${units})
      target_link_libraries(capture-headers PRIVATE capture-device-headers ${CAPTURE_GR4})
      target_compile_options(capture-headers PRIVATE ${CAPTURE_WARNINGS})
      list(LENGTH units count)
      message(STATUS "capture: ${count} device headers PARSED")
    endif()
    if(unparsed)
      string(REPLACE ";" ", " unparsed_line "${unparsed}")
      message(STATUS "capture: device headers SKIPPED ${unparsed_line}")
    endif()
  elseif(NOT CAPTURE_GR4)
    message(STATUS "capture: device headers NOT parsed (no gnuradio4)")
  endif()

  # ---- the discovery library ---------------------------------------------------------
  #
  # common/include/capture/common/DiscoveryLibrary.hpp states the family listing, the
  # device listing and the control probe as plain functions, and
  # common/src/common/DiscoveryLibrary.cpp defines them over the fragments
  # capture_discovery_sources() lists. The discovery library is that source over every
  # device whose vendor header, vendor library and fragment this configure found, as a
  # shared library linking those vendor libraries. A device whose detection states a
  # verdict of its own is held to it, so a vendor library below its minimum is left out. The
  # library is on by default where the header parse is.
  if(CAPTURE_ENABLE_DISCOVERY_LIBRARY AND CAPTURE_GR4)
    get_property(families GLOBAL PROPERTY CAPTURE_DISCOVERY_FAMILIES)
    get_property(libs GLOBAL PROPERTY CAPTURE_DISCOVERY_LIBS)
    list(REMOVE_DUPLICATES libs)

    add_library(${discovery} SHARED)
    capture_discovery_sources(${discovery} ${families})
    add_library(${alias} ALIAS ${discovery})
    set_target_properties(${discovery} PROPERTIES EXPORT_NAME discovery)
    target_link_libraries(${discovery} PUBLIC capture-interface)
    target_compile_options(${discovery} PRIVATE ${CAPTURE_WARNINGS})
    target_link_libraries(${discovery} PRIVATE capture-device-headers ${CAPTURE_GR4} ${libs})
    message(STATUS "capture: the discovery library carries ${families}")
  elseif(CAPTURE_ENABLE_DISCOVERY_LIBRARY)
    message(STATUS "capture: the discovery library is NOT built (no gnuradio4)")
  endif()
endfunction()
