<!-- Copyright 2026 Jeff Long; SPDX-License-Identifier: MIT -->
# gr4-pluto

AD9361 blocks for GNU Radio 4 over libiio, for the ADALM-Pluto and other
radios whose AD9361, AD9363 or AD9364 runs under the Linux ad9361 driver,
the Red Wire Technologies radios among them. The package builds a block
library that GNU Radio 4 loads as a plugin, with two blocks:

- `capture::ad9361::Source` receives one or two channels of complex samples.
- `capture::ad9361::Sink` transmits one or two channels of complex samples.

It also builds a small discovery library that lists the attached radios and
states the control surface of the family, for a program that holds no block
type.

A copy script produces this tree from libcapture. `ad9361/` holds the blocks,
their registration header, their cases and their `CMakeLists.txt`. `common/`
holds the headers the blocks build on, the discovery library's source, the
case support and `cmake/capture.cmake`, the build logic
`ad9361/CMakeLists.txt` calls. `extract/ad9361.imported` lists every file the
copy placed and names the libcapture change the copy was taken at; a change to
one of them is made in libcapture.

## Building

The build needs a C++23 compiler, CMake 3.28 or later, libiio 0.x or 1.0
with its headers, and a GNU Radio 4 installation with its block-library
CMake package. The build takes 0.x where both libiio versions are
installed; `-DCAPTURE_AD9361_PREFER_LIBIIO1=ON` takes 1.0. libad9361 is optional:
with it the blocks reach 520834 S/s, without it 2083334 S/s. It is used only
where it was built for the libiio the build takes: a 0.x release or its
`libad9361-iio-v0` branch for libiio 0.x, its main branch for 1.0. The
configuration stops and says why when it finds no libiio it can use.
Point `CMAKE_PREFIX_PATH` at the GNU Radio 4 prefix and install
into the same prefix:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=/opt/gnuradio4 \
      -DCMAKE_INSTALL_PREFIX=/opt/gnuradio4
cmake --build build
cmake --install build
```

The block library must be built against the GNU Radio 4 it is loaded into.

The cases need Boost.UT, a single header. `GR4PLUTO_UT_INCLUDE_DIR` names the
directory that holds `boost/ut.hpp`. The offline cases ask no device:

```sh
ctest --test-dir build -LE hardware
```

ctest sets `GNURADIO4_PLUGIN_DIRECTORIES` to `build/lib/gnuradio-4/plugins` for
every case. A case that builds a graph loads block libraries from there and
none from a prefix.

`-DGR4PLUTO_ENABLE_HARDWARE_TESTS=ON` adds the cases that open a radio,
labeled `hardware`. `CAPTURE_HW_AD9361_ARGS` in the environment selects it,
`uri=usb:` by default. Two of them transmit a tone at the attenuation
`CAPTURE_HW_AD9361_TX_GAIN` names, -30 dB by default.

## Loading the blocks

`cmake --install` puts `libgr4-pluto-blocks.so` in the prefix's
`lib64/gnuradio-4/plugins` (the prefix's library directory), where the GNU
Radio 4 tools of that prefix look. A build tree is loaded by naming its plugin
directory:

```sh
grinfo blocks --plugin-dir build/lib/gnuradio-4/plugins
grinfo block capture::ad9361::Source --plugin-dir build/lib/gnuradio-4/plugins
```

`GNURADIO4_PLUGIN_DIRECTORIES` names the same directory for every tool. The
tools of a prefix outside the system library path also need that prefix's
library directory on `LD_LIBRARY_PATH`.

Where the GNU Radio 4 core offers its device listing registry, the block
library also registers the AD9361 family's listing when it loads, and `grinfo`
lists the attached radios by label and selector:

```sh
grinfo devices --allow-open --plugin-dir build/lib/gnuradio-4/plugins
```

The listing scans libiio's backends and opens each context it finds to read
its devices, so it is asked only where opening is allowed.

A receive graph for `rungraph`:

```yaml
blocks:
  - id: capture::ad9361::Source
    parameters:
      name: src
      device_parameter: uri=usb:
      sample_rate: 2500000.0
      frequency: !!float64 [915000000.0]
  - id: gr::blocks::testing::NullSink<complex<float32>>
    parameters:
      name: sink
connections:
  - [src, out, sink, in]
```

## The device selector

Each block's `device_parameter` setting names the radio's IIO context and its
FPGA image:

- `uri=<libiio URI>`, such as `uri=usb:1.78.5` or `uri=ip:pluto.local`, or
  `ip=<address>` for `uri=ip:<address>`. An empty URI opens libiio's default
  context: the one `IIO_CONTEXT_URI` names, or the radio's own devices on the
  radio itself.
- `image=adi`, the default, for the ADI reference designs, the Pluto's among
  them, or `image=rwt` for the Red Wire Technologies images. The images pack
  their samples differently, and no attribute tells them apart: a wrong
  `image` scrambles the samples without an error.

The blocks of one process share one IIO context per radio, so a source and a
sink reach one radio together over USB as well as over the network, however
each names it. `usb:` and the radio's full USB URI are one radio, a host name
and its address are one, and two links to a radio stating one `hw_serial`
are one.

## Rates, gains and channels

Both directions run on one sample clock, from 520834 S/s with libad9361 or
2083334 S/s without it, to 61.44 MS/s, and a rate applies in the running
stream. A request is rounded to whole hertz, one outside the range is
brought to the nearer end, and one that is not a number is written at the
floor; a request moved by more than half a hertz is named on standard error.
A block that finds the radio at its rate writes
nothing, so it does not recalibrate the radio under the other direction's
stream. While one block holds a radio, another block of the process asking for
a different rate is refused, at its start or at its rate write, with both rates
in the message. `channels` lists the device channels, `{0}`, `{1}` or `{0, 1}`:
the source puts them on `out` and `out2`, and the sink takes them on `in` and
`in2`, connected exactly when two are listed. `rx_gains` is the receive gain
over the range the radio states for the LO's band, and `gain_mode` hands it to
the AD9361's AGC. `tx_gains` is the transmit attenuation stated as a gain,
-89.75 to 0 dB. `bandwidth` sets the analog filter, and 0 follows the rate.

## Tags

The source puts `gr:sample_rate` and `gr:frequency` on the first block of a
stream and after a rate change, a wall-clock timing tag named by
`trigger_name` every `tag_interval`, and `gr:rx_overflow` with
`rx_overflow_cause` at a gap: `radio FIFO`, `host ring full` or `rate
change`. At `host ring full` the tag also carries `gr:n_dropped_samples`, the
frames the ring refused, per channel. At `rate change` it carries the frames
the change discarded and the frames the ring refused that no earlier marker
counted, per channel. An overflow or a lost buffer that falls among the frames
a rate change discards, or where they end, joins the rate change's tag and
adds no tag of its own. At `radio FIFO` the tag carries no count: the radio
reports the overflow and not its length. Each count holds the frames lost
since the last marker. The counts sum to `rx_dropped_samples`.
Under `image=rwt` it marks the radio's pulse-per-second on the first
sample after it: `gr:trigger_name` `PPS_RADIO`, `gr:trigger_time` the UTC
second nearest the host's time there, `gr:trigger_offset` 0, and the radio's
own sample count in `gr:trigger_meta_info` as `radio_sample_count`. The radio
inserts these tags only after a program on the radio enables them; a host
cannot.

The radio has no timed transmit, so the sink gates bursts itself. A sample
tagged `gr:tx_eob = true`, on either input with two channels, ends the burst:
the radio plays it out, frees its buffer and powers the transmit LO down, and
the next samples start the next burst. `gr:tx_time` is ignored and reported
once on the message port. A stop leaves the radio at its most attenuation
with the LO powered down.

## The controls, control and sensors properties

Each block answers three properties by message while it runs. A program
writes a message to the scheduler's message input with the block's unique
name as the service and the property as the endpoint. The reply arrives on
the scheduler's message output under the same endpoint and client request id.

- `controls` answers Get with `{controls: [descriptor, ...]}`: the gain
  (`RX` on the source, `TX` on the sink), the source's `AGC_MODE` and its
  three tracking loops, `GAIN_MODE`, and the rate and frequency. Subscribe
  sends the same body again each time the surface changes.
- `control` answers Set of `{id, value}` with `{id, value, requested,
  read_back}`.
- `sensors` answers Get with `{sensors: [reading, ...]}`, whose first reading
  is the stream state. While the radio is open the source adds
  `rx_overflows`, the overflows the radio reported, and `rx_dropped_samples`,
  the frames lost per channel, both glance rows; `rx_dropped_markers`, the
  tags the output port had no room for, a detail row; and
  `rx_analog_bandwidth`, the analog filter the radio holds, as `rf_bandwidth`
  reads back after the write.

`capture/common/Properties.hpp` holds the functions that read and write these
maps.

## The discovery library

`capture/common/DiscoveryLibrary.hpp` declares three functions in
`capture::discovery`, and `libgr4-pluto-discovery.so` defines them over the
AD9361 family: `families()`, `listDevices()` and `probeControls(family,
transmit, ctx)`. The listing opens each context a scan finds.

```cmake
find_package(gr4-pluto CONFIG REQUIRED)
target_link_libraries(myprogram PRIVATE gr4-pluto::discovery)
```

`find_package(gr4-pluto)` gives `gr4-pluto::headers`, `gr4-pluto::blocks` and
`gr4-pluto::discovery`. `gr4-pluto::headers` carries the installed include
directory, C++23, Threads, and the definitions `CAPTURE_AD9361_LIBIIO1` and
`CAPTURE_AD9361_LIBAD9361` where the build chose them. A program that compiles
the block headers finds and links GNU Radio 4, the libiio the package was built
with, and libad9361 where the package used it.

## License

gr4-pluto is licensed under the MIT license. Each file carries its own
copyright line and SPDX-License-Identifier header, and `LICENSE` carries the
license text.

gr4-pluto links libiio and, where found, libad9361. Their own licenses,
LGPL-2.1-or-later, govern the distribution of a binary that includes them.
The source files here stay MIT.
