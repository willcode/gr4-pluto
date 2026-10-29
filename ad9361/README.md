<!-- Copyright 2026 Jeff Long; SPDX-License-Identifier: MIT -->
# ad9361: the AD9361 source and sink

`capture::ad9361::Source` receives from an AD9361 radio whose transceiver
runs under the Linux ad9361 driver, through libiio: the ADALM-Pluto, the
Red Wire Technologies (RWT) radios such as the Oxygen, and other Zynq
AD9361 designs. `capture::ad9361::Sink` transmits to one. Both are GNU
Radio 4 blocks of complex float samples. The source puts one or two
receive channels on `out` and `out2`; the sink takes one or two transmit
channels on `in` and `in2`. The family word is
`ad9361`. A radio that reaches its AD9361 through a support package of its
own, with its own FPGA and driver, belongs to that package's family.

## Selecting a unit

`device_parameter` names the radio's IIO context and its FPGA image:

```
uri=<libiio URI>
ip=<address>
image=adi
image=rwt
```

`ip=<address>` stands for `uri=ip:<address>`, the iiod the radio runs: a
Pluto at `ip:192.168.2.1` or `usb:<bus>.<address>.<interface>`, an Oxygen
over USB gadget Ethernet. An empty URI opens libiio's default context: the
one `IIO_CONTEXT_URI` names, or the radio's own devices when the block runs
on the radio. `image` names how the FPGA packs the samples: `adi` for the
ADI reference designs, the Pluto's among them, and `rwt` for the RWT
images. `adi` is the default. A wrong `image` scrambles the samples
without an error.

## Its own settings, and the sink's

| Setting       | Type           | Meaning                                   |
|---------------|----------------|-------------------------------------------|
| `channels`    | list of Size_t | `{0}`, `{1}` or `{0, 1}`, taken at start  |
| `bandwidth`   | float64, Hz    | analog filter; 0 follows the rate         |
| `buffer_size` | Size_t         | samples per transfer; 0 follows the rate  |

The sink takes the same three. Its `channels` lists the transmit
channels, and `in2` is connected exactly when it lists two. Its
`tx_gains` is the attenuation stated as a gain, -89.75 to 0 dB, and
`tx_antennae` names the output, `A` or `B`. `rx_antennae` names the
AD9361 input, such as `A_BALANCED`. The controls
`AGC_MODE`, `QUADRATURE_TRACKING`, `RF_DC_TRACKING` and `BB_DC_TRACKING`
reach the AD9361 directly.

## Bursts

The radio has no timed transmit and no burst marker. The sink gates
bursts itself. The first samples it takes create the radio's buffer and
power the transmit LO up. A sample tagged `gr:tx_eob = true` is the last of
its burst: the sink pads the buffer with zeros, waits for the radio to
play what it holds, frees the buffer and powers the LO down. The next
samples start the next burst. With two channels a burst end on either
input ends the burst on both. `gr:tx_time` is ignored and reported once.
A stop plays out what is queued and leaves the radio at its most
attenuation with the LO powered down.

Under `image=rwt` the source marks the radio's pulse-per-second on the
first sample after it, as a timing source marks a second:
`gr:trigger_name` is `PPS_RADIO`, `gr:trigger_time` the UTC second
nearest the host's time for that sample, and `gr:trigger_offset` 0.
`gr:trigger_meta_info` holds the radio's own 56-bit sample count as
`radio_sample_count` and the host's time as `gr:local_time`. The host's
clock names the second, so it has to be right within half a second. A
pulse's tag replaces the wall-clock timing tag of its block. The radio
inserts these tags only after a program on the radio enables them; a host
cannot.

The two directions share one sample clock. A source and a sink on one
radio ask for the same rate; a block that finds the radio at its rate
writes nothing. While one block of a process holds a radio, another block
of that process asking for a different rate is refused, at its start or at
its rate write, with both rates in the message. The blocks of one process share one IIO context per radio,
so a source and a sink reach one radio together over `usb:` as well as
over the network, whichever spelling of the radio each was given.

## What it needs

- libiio 0.x or 1.0, with its headers. `capture/ad9361/Iio.hpp` holds the
  calls whose form differs. The build takes 0.x where both are installed;
  `CAPTURE_AD9361_PREFER_LIBIIO1=ON` takes 1.0, and the definition
  `CAPTURE_AD9361_LIBIIO1` on the build's targets carries the choice.
- libad9361 where rates under 2083334 S/s are wanted, built for the libiio
  the build takes: a 0.x release or its `libad9361-iio-v0` branch for 0.x,
  its main branch for 1.0. The definition `CAPTURE_AD9361_LIBAD9361` marks a
  build that uses it. Without it the family starts at that rate.
- GNU Radio 4 core; the block library also needs its block-library macros,
  `GnuRadioBlockLib`.
- On the radio, an FPGA image that creates `ad9361-phy` and
  `cf-ad9361-lpc`.

With libiio the build makes the family's block library and adds the cases
in `test/`. The cases open no radio. `CAPTURE_HW_AD9361=ON` adds the cases
that open one, labeled `hardware`; `CAPTURE_HW_AD9361_ARGS` in the
environment selects it.

Removing this directory needs no edit outside it.
