# Copyright 2026 Jeff Long
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.28)

project(
  gr4-pluto
  VERSION 0.1.0
  DESCRIPTION "AD9361 source and sink blocks for GNU Radio 4 over libiio, loaded as a block library"
  LANGUAGES CXX)

# A single-config generator given no build type compiles without optimization and without
# NDEBUG. The package builds Release there unless the caller names a type.
get_property(GR4PLUTO_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT CMAKE_BUILD_TYPE AND NOT GR4PLUTO_MULTI_CONFIG)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "gr4-pluto: no CMAKE_BUILD_TYPE given; building Release")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# GNU Radio 4 core and its block-library macros. ad9361/CMakeLists.txt finds libiio, which the
# blocks need, and libad9361, which they use where it is found.
find_package(Threads REQUIRED)
find_package(gnuradio4 4.0 CONFIG REQUIRED)
find_package(GnuRadioBlockLib CONFIG REQUIRED)

# The cases: one binary per subject under ad9361/test/ and one ctest entry per case, each named
# on the binary's command line. The offline cases carry the label offline and ask no device.
# The hardware cases open a radio and carry the label hardware; CAPTURE_HW_AD9361_ARGS in the
# environment selects it. Boost.UT is one header: GR4PLUTO_UT_INCLUDE_DIR names the directory
# holding boost/ut.hpp.
option(GR4PLUTO_ENABLE_TESTING "Build the cases" ${PROJECT_IS_TOP_LEVEL})
option(GR4PLUTO_ENABLE_HARDWARE_TESTS "Build the cases that open a radio (label hardware)" OFF)
set(GR4PLUTO_UT_INCLUDE_DIR "" CACHE PATH "Directory holding boost/ut.hpp")

# The settings common/cmake/capture.cmake and ad9361/CMakeLists.txt read: the AD9361
# directory, the package's names, the label of the offline cases, and the options above. The
# block library compiles both device headers, so the separate header parse is off. The package
# installs its discovery library under any top.
set(CAPTURE_DEVICES ad9361)
set(CAPTURE_PACKAGE gr4-pluto)
set(CAPTURE_PACKAGE_BLOCKS GrPlutoBlocks)
set(CAPTURE_TEST_LABEL offline)
set(CAPTURE_UT_VARIABLE GR4PLUTO_UT_INCLUDE_DIR)
set(CAPTURE_ENABLE_TESTING ${GR4PLUTO_ENABLE_TESTING})
set(CAPTURE_ENABLE_HEADER_PARSE OFF)
set(CAPTURE_ENABLE_DISCOVERY_LIBRARY ON)
if(GR4PLUTO_UT_INCLUDE_DIR)
  set(CAPTURE_UT_INCLUDE_DIR ${GR4PLUTO_UT_INCLUDE_DIR})
endif()
if(GR4PLUTO_ENABLE_HARDWARE_TESTS)
  set(CAPTURE_HW_AD9361 ON)
endif()
include(common/cmake/capture.cmake)
add_library(gr4-pluto::headers ALIAS capture-interface)

add_subdirectory(ad9361)

# The plain discovery functions of capture/common/DiscoveryLibrary.hpp over the AD9361 family.
# capture_finish() also prints the capture: lines naming the block library built or the reason
# it was not.
capture_finish(gr4-pluto-discovery gr4-pluto::discovery)
if(NOT TARGET gr4-pluto-blocks)
  get_property(GR4PLUTO_ABSENT GLOBAL PROPERTY CAPTURE_AD9361_ABSENT)
  if(GR4PLUTO_ABSENT)
    message(FATAL_ERROR "gr4-pluto: ${GR4PLUTO_ABSENT}")
  endif()
  message(FATAL_ERROR "gr4-pluto: the block library was not built (see the capture: lines above)")
endif()
if(TARGET gr4-pluto-discovery)
  set_target_properties(gr4-pluto-discovery PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
  # The discovery library a program links, asked for the families it carries.
  if(CAPTURE_TESTS)
    capture_test(
      qa_discovery_library_ad9361
      DIR ad9361/test
      CASES discovery.ad9361-the-linked-library-carries-the-ad9361-family
      LIBS gr4-pluto-discovery)
  endif()
endif()

# ---- install ---------------------------------------------------------------------------
#
# The block library goes to the plugin directory of GNU Radio 4 under the same prefix, the
# discovery library to the library directory, and the headers under include/gr4-pluto/.
# find_package(gr4-pluto) gives gr4-pluto::blocks, gr4-pluto::discovery and gr4-pluto::headers.
install(
  TARGETS gr4-pluto-blocks
  EXPORT gr4-plutoTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gnuradio-4/plugins)
install(TARGETS capture-interface EXPORT gr4-plutoTargets)
if(TARGET gr4-pluto-discovery)
  install(
    TARGETS gr4-pluto-discovery
    EXPORT gr4-plutoTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(
  DIRECTORY common/include/capture ad9361/include/capture
  DESTINATION ${CAPTURE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.hpp")
install(
  EXPORT gr4-plutoTargets
  NAMESPACE gr4-pluto::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gr4-pluto)
file(
  CONFIGURE
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/gr4-plutoConfig.cmake
  CONTENT
    [[
include(CMakeFindDependencyMacro)
find_dependency(Threads)
find_dependency(gnuradio4 CONFIG)
include("${CMAKE_CURRENT_LIST_DIR}/gr4-plutoTargets.cmake")
]]
  @ONLY)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/gr4-plutoConfigVersion.cmake
                                 COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gr4-plutoConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/gr4-plutoConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gr4-pluto)
