# Copyright 2026 Jeff Long
# SPDX-License-Identifier: MIT

# AD9361 radios under the Linux ad9361 driver: capture::ad9361::Source and
# capture::ad9361::Sink over libiio, with libad9361 where it is found.

# libiio 0.x installs iio.h; libiio 1.0, whose API differs, installs iio/iio.h.
# capture/ad9361/Iio.hpp takes either, and the definition CAPTURE_AD9361_LIBIIO1 on
# capture-interface selects 1.0, so a program compiling the installed headers follows the
# build. 0.x is used where both are found, unless CAPTURE_AD9361_PREFER_LIBIIO1 is ON. Each
# library is looked for beside its header first. libad9361 is used only where its library links
# the libiio chosen. The reason the family is absent is the global property
# CAPTURE_AD9361_ABSENT, which the gr4-pluto package states when it stops.
option(CAPTURE_AD9361_PREFER_LIBIIO1 "Build the AD9361 family against libiio 1.0 where 0.x is found too" OFF)
find_path(CAPTURE_IIO_INCLUDE iio.h)
find_path(CAPTURE_IIO1_INCLUDE iio/iio.h)
if(CAPTURE_IIO1_INCLUDE AND (CAPTURE_AD9361_PREFER_LIBIIO1 OR NOT CAPTURE_IIO_INCLUDE))
  set(CAPTURE_AD9361_IIO1 ON)
  set(CAPTURE_AD9361_IIO_NAME "libiio 1.0")
  set(CAPTURE_AD9361_IIO_INCLUDE ${CAPTURE_IIO1_INCLUDE})
  find_library(CAPTURE_IIO1_LIB iio HINTS ${CAPTURE_IIO1_INCLUDE}/../lib64 ${CAPTURE_IIO1_INCLUDE}/../lib NO_DEFAULT_PATH)
  find_library(CAPTURE_IIO1_LIB iio)
  set(CAPTURE_AD9361_IIO_LIB ${CAPTURE_IIO1_LIB})
else()
  set(CAPTURE_AD9361_IIO1 OFF)
  set(CAPTURE_AD9361_IIO_NAME "libiio 0.x")
  set(CAPTURE_AD9361_IIO_INCLUDE ${CAPTURE_IIO_INCLUDE})
  find_library(CAPTURE_IIO_LIB iio HINTS ${CAPTURE_IIO_INCLUDE}/../lib64 ${CAPTURE_IIO_INCLUDE}/../lib NO_DEFAULT_PATH)
  find_library(CAPTURE_IIO_LIB iio)
  set(CAPTURE_AD9361_IIO_LIB ${CAPTURE_IIO_LIB})
endif()

set(CAPTURE_AD9361_ABSENT "")
if(NOT CAPTURE_AD9361_IIO_INCLUDE)
  set(CAPTURE_AD9361_ABSENT "libiio not found: the AD9361 blocks need libiio 0.x (iio.h) or 1.0 (iio/iio.h) with its library")
elseif(NOT CAPTURE_AD9361_IIO_LIB)
  set(CAPTURE_AD9361_ABSENT "${CAPTURE_AD9361_IIO_NAME}'s header is at ${CAPTURE_AD9361_IIO_INCLUDE} and its library is not found")
else()
  message(STATUS "capture: AD9361 over ${CAPTURE_AD9361_IIO_NAME} (${CAPTURE_AD9361_IIO_LIB})")
  if(CAPTURE_AD9361_IIO1)
    target_compile_definitions(capture-interface INTERFACE CAPTURE_AD9361_LIBIIO1)
  endif()
endif()
if(CAPTURE_AD9361_ABSENT)
  message(STATUS "capture: ${CAPTURE_AD9361_ABSENT}")
endif()
set_property(GLOBAL PROPERTY CAPTURE_AD9361_ABSENT "${CAPTURE_AD9361_ABSENT}")

# Beside the libiio chosen first: CMAKE_PREFIX_PATH is searched before HINTS, and a prefix that
# holds the other libiio's libad9361 would otherwise win.
if(CAPTURE_AD9361_IIO_INCLUDE)
  find_path(CAPTURE_AD9361_INCLUDE ad9361.h HINTS ${CAPTURE_AD9361_IIO_INCLUDE} NO_DEFAULT_PATH)
  find_library(CAPTURE_AD9361_LIB ad9361 HINTS ${CAPTURE_AD9361_IIO_INCLUDE}/../lib64 ${CAPTURE_AD9361_IIO_INCLUDE}/../lib NO_DEFAULT_PATH)
endif()
find_path(CAPTURE_AD9361_INCLUDE ad9361.h)
find_library(CAPTURE_AD9361_LIB ad9361)
# The two libad9361 headers are the same text; the library names the libiio it was built for
# among its dependencies, libiio.so.0 or libiio.so.1.
set(CAPTURE_AD9361_LIB_USED ${CAPTURE_AD9361_LIB})
if(CAPTURE_AD9361_LIB)
  file(STRINGS ${CAPTURE_AD9361_LIB} CAPTURE_AD9361_FOR_IIO1 REGEX "^libiio\\.so\\.1$")
  if(CAPTURE_AD9361_FOR_IIO1 AND NOT CAPTURE_AD9361_IIO1)
    message(STATUS "capture: libad9361 at ${CAPTURE_AD9361_LIB} is built for libiio 1.0 and is not used")
    set(CAPTURE_AD9361_LIB_USED "")
  elseif(NOT CAPTURE_AD9361_FOR_IIO1 AND CAPTURE_AD9361_IIO1)
    message(STATUS "capture: libad9361 at ${CAPTURE_AD9361_LIB} is built for libiio 0.x and is not used")
    set(CAPTURE_AD9361_LIB_USED "")
  endif()
endif()

# The headers compile libad9361 in where CAPTURE_AD9361_LIBAD9361 is defined, which happens
# only where its header and library are found and built for the libiio chosen. A header found
# on the include path is not enough: a libad9361 for the other libiio sits in /usr/include
# beside one in a prefix.
set(CAPTURE_AD9361_INCLUDES ${CAPTURE_AD9361_IIO_INCLUDE})
set(CAPTURE_AD9361_LIBS ${CAPTURE_AD9361_IIO_LIB})
if(CAPTURE_AD9361_INCLUDE AND CAPTURE_AD9361_LIB_USED)
  list(APPEND CAPTURE_AD9361_INCLUDES ${CAPTURE_AD9361_INCLUDE})
  list(APPEND CAPTURE_AD9361_LIBS ${CAPTURE_AD9361_LIB})
  if(NOT CAPTURE_AD9361_ABSENT)
    target_compile_definitions(capture-interface INTERFACE CAPTURE_AD9361_LIBAD9361)
  endif()
  message(STATUS "capture: libad9361 FOUND (${CAPTURE_AD9361_LIB}); AD9361 rates reach 520834 S/s")
elseif(NOT CAPTURE_AD9361_ABSENT)
  message(STATUS "capture: no libad9361 for ${CAPTURE_AD9361_IIO_NAME}; AD9361 rates start at 2083334 S/s")
endif()

if(NOT CAPTURE_AD9361_ABSENT)
  capture_device(ad9361 INCLUDE ${CAPTURE_AD9361_INCLUDES} LIBRARY ${CAPTURE_AD9361_LIBS})
else()
  capture_device(ad9361 INCLUDE "" LIBRARY "")
endif()

if(NOT CAPTURE_TESTS OR NOT CAPTURE_GR4)
  return()
endif()

# The sample decoder, the ranges, the selector and the surface. None of them opens a radio.
if(NOT CAPTURE_AD9361_ABSENT)
  capture_test(
    qa_ad9361
    DIR test
    CASES ad9361.adi-words-follow-the-channel-order
          ad9361.rwt-one-channel-words-put-the-earlier-sample-high
          ad9361.rwt-two-channel-words-put-the-first-channel-low
          ad9361.a-tag-split-across-buffers-decodes-once
          ad9361.an-escaped-escape-is-a-sample
          ad9361.available-ranges-parse
          ad9361.the-gain-range-follows-the-band
          ad9361.the-bandwidth-follows-the-rate
          ad9361.a-transfer-holds-whole-words
          ad9361.the-selector-names-a-context
          ad9361.a-channel-list-is-checked
          ad9361.rates-follow-the-build
          ad9361.an-unconnected-output-ends-the-block
          ad9361.a-context-that-cannot-open-fails-the-start
          ad9361.a-context-without-the-radio-fails-the-start
          ad9361.a-pulse-names-the-nearest-second
          ad9361.a-radio-has-one-key
          ad9361.one-context-per-radio
          ad9361.one-context-per-uri
          ad9361.a-second-rate-on-one-radio-is-refused
          ad9361.declares-its-labels
          ad9361.transmit-words-mirror-the-receive-words
          ad9361.sink-names-the-tags-it-ignores
          ad9361.sink-states-its-transmit-range
          ad9361.sink-refuses-a-start-it-cannot-make
          ad9361.states-its-rate-and-frequency
          ad9361.states-its-gain-mode-and-its-stream-state
          ad9361.a-refused-buffer-is-counted-in-its-marker
          ad9361.the-filter-stated-is-the-one-the-radio-holds
          ad9361.sink-keeps-the-filter-the-radio-holds
          ad9361.a-rate-change-carries-the-loss-the-ring-refused
          ad9361.an-overflow-after-a-rate-change-joins-its-marker
          ad9361.the-sensors-state-what-the-stream-lost
    LIBS ${CAPTURE_GR4} ${CAPTURE_AD9361_LIBS}
    INCLUDES ${CAPTURE_AD9361_INCLUDES})
else()
  message(STATUS "capture: the AD9361 cases are SKIPPED (no libiio)")
endif()

# The cases that need a radio. They open one, so they are off by default, carry the label
# hardware rather than capture, and are never part of the hardware-free suite. One
# invocation runs one case; CAPTURE_HW_AD9361_ARGS in the environment names the radio and its
# image, uri=usb: by default.
option(CAPTURE_HW_AD9361 "Build the AD9361 cases that open a radio" OFF)
if(CAPTURE_HW_AD9361 AND NOT CAPTURE_AD9361_ABSENT)
  set(CAPTURE_HW_AD9361_CASES
      ad9361.open-stream-close ad9361.rate-ladder ad9361.rate-change-in-stream ad9361.tuning-and-range
      ad9361.gain-and-agc ad9361.teardown-and-restart ad9361.every-spelling-shares-one-context
      ad9361.a-sink-start-keeps-the-source-whole ad9361.a-refused-source-start-is-refused-again
      ad9361.tx-tone-reaches-the-receiver ad9361.tx-eob-ends-the-burst)
  capture_test(
    hw_ad9361
    DIR test
    CASES ${CAPTURE_HW_AD9361_CASES}
    LIBS ${CAPTURE_GR4} ${CAPTURE_AD9361_LIBS}
    INCLUDES ${CAPTURE_AD9361_INCLUDES})
  # The rate ladder streams every rate the family offers for a second each.
  set_tests_properties(${CAPTURE_HW_AD9361_CASES} PROPERTIES TIMEOUT 120 LABELS hardware)
  message(STATUS "capture: the AD9361 hardware cases are ENABLED (label hardware)")
endif()
