<!-- Copyright 2026 Jeff Long; SPDX-License-Identifier: MIT -->
# capture/common — the two device block families

The files in `common/include/capture/common/` are device-neutral and their names stay in the namespace `capture`.
`Env.hpp` and `Threading.hpp` are the two utilities the device tiers lean on.
`Ring.hpp` holds the drop-newest ring both directions stage samples in, every helper that
reads or replaces one, the allocator its storage takes and the doorbell a consumer waits on;
it depends on nothing. `Device.hpp` holds the parts a block needs
whichever direction it runs in: the streaming-thread guard, the teardown guard, the control
guard and the duties a source owes its output port, with `Ring.hpp` carried through it.
`Source.hpp` holds the publish side — the hold-and-retry publish, the rate and frequency tags
on a stream's first block, the wall clock its periodic timing tag is read from, and the gap marker beside
it — and every source carries `Device.hpp` through it while every sink includes `Device.hpp`
directly. `Sink.hpp`
holds the transmit vocabulary: the clip, the queue sizing and its bound, and the counters. `ControlDesc.hpp`
states the control descriptor both families answer with, the SAMPLE_RATE, FREQUENCY and
GAIN_MODE descriptors among them, the ids and words of the stream-state and analog-bandwidth
readings, the device-string reader and the coverage test, and it depends
on nothing, so a consumer drawing a control surface takes that header alone. `Properties.hpp`
holds the four message properties every block answers, controls, control, sensors and devices,
and the property maps a descriptor, a reading and a listing travel in;
`Device.hpp` carries it, so every block has it. `Discovery.hpp` lists the attached devices of
every family a program passes in and probes one of them for its surface, from each family's
own static calls; it depends on `ControlDesc.hpp` alone, and `Properties.hpp` takes it for the
probe an unstarted block answers with. `DiscoveryLibrary.hpp` states the
same answers as plain functions, which `common/src/common/DiscoveryLibrary.cpp` defines over the
families its build lists. `CoreListing.hpp` states a family's listing in the form GNU Radio 4's
device listing registry takes and registers it there; it depends on `Discovery.hpp` and the
core's `DeviceListing.hpp`.

Each device directory stands at the top of the tree beside `common/` and holds one radio's
files, its own knowledge file among them. A driver's unit listing and its board record live
in the `Device.hpp` of that directory's headers, so the sink of a radio offers the same
device list and narrows to the same board its source does, without including the source.
A device directory's `include/gnuradio-4.0/capture/<device>/` holds one registration
header, `GR_REGISTER_BLOCK` once per block, from which the build generates the family's
block library; a program loading that library creates each block by its type name. The
header also registers the family's device listing with the core's registry when the library
loads, through `registerCoreListing` over the `BlockList` of the family's `Discovery.hpp`: the
blocks' `kDevice` as the family word, their `kListingOpens` as `opens`, receive where a source
is given and transmit where a sink is, and the first block's `enumerateDevices()` as the
function that lists the units, each label and selector unchanged. An inline variable of the
family's namespace holds the registration, so the library registers once however many of its
units include the header. The registration stands under `CAPTURE_DEVICE_LISTINGS`, which the
configure defines where a probe of the core's registry compiles and links, and the installed
interface target carries the definition as well. Against a core without the registry a block
library registers its blocks alone. `gr::PluginLoader::listDevices()` then lists the units of
every family a program loaded, and leaves out, and names with the reason, a family whose
listing opens a unit unless the call allows it. Each
source and sink declares its labels through `gr::block::describe()`: its `kDevice` word as
its family with one sentence naming the library, `role/source` or `role/sink`, one `holds`
word, and the medium it ingests or emits. A block that opens a hardware unit holds `device`,
and a block that opens a sound server's stream holds `ipc`. A receiver ingests `rf`, a
transmitter emits `rf`, a sound source ingests `sound`, a block whose settings take a reference
and a PPS ingests `time`, and a board that takes an outside clock ingests `time`. A block whose
surface offers a bias tee emits `electrical`, for the DC power it puts on the antenna
connector, and a block that drives a clock output emits `time`. The type's label says the
block may put energy into the world, and the `BIAS_TEE` control's descriptor says whether it
does. The declaration and its include stand under `CAPTURE_BLOCK_LABELS`, which the configure
defines where a probe of the core's `describe()` compiles, so a block builds without labels
against a core without that header or with its earlier form under the same name. The
installed interface target carries the definition as well.
`support/Labels.hpp` pins each block's labels in its family's cases.

## The source family

**purpose**
One GNU Radio 4 source block per radio, speaking the vendor library directly and describing
the control surface the device really has.

**algorithm**
A device block is one shape with a device-specific middle. A device thread — the
vendor library's own callback, or one this block owns — puts samples into a bounded
drop-newest ring; a consumer thread drains staged settings, converts, and hands whole
blocks to `holdAndPublish`, which holds and retries until the output edge takes
them. A consumer thread that pulls from a library with a blocking receive can instead
receive into a reservation on the output edge and publish it, placing the same tags on its
first sample through `publishBlockHeadTags`. A source that finds no room waits on the graph's
progress counter, which every block that consumes samples moves. On a core whose counter offers
a timed wait, the wait ends when the reader takes samples. On a core without one, the source
reads the counter sixteen times across the bound, and the wait ends at most one step after the
reader takes samples. The wait is bounded by the time the stream takes to fill half the edge
at its rate, and by 0.1 s, which is how late a stop or a staged setting reaches a source whose
edge stays full. A source moves the progress counter after the span holding its samples is
released, because the edge takes the samples at the release; a reader woken earlier would
find nothing. The appliers write the device under one control mutex.

**io**
Settings in and CF32 out. The first block of every stream carries the rate and the frequency
in force as `gr:sample_rate` and `gr:frequency` tags. A periodic timing tag goes out, and a
`gr:rx_overflow` marker on every discontinuity.
`describeControls()` states the control surface; `setControl` and `setElementGain` apply
it. The message properties of the shared files answer that surface, those writes and those
readings to a caller outside the graph. `readSensors()` publishes what the device and the
stream produce: the device's own sensors and, beside them, what the stream has lost and what
the unit is, neither of which a device sensor reports. A value a control or a setting sets
is never a reading; the control surface states it. The analog filter in force is the one
exception, since a fixed filter has a width and no control. A device with no
sensor chain of its own still publishes what its firmware states about its sample loop —
the mode it is in, the shortfalls its own buffer has taken, the reason it last stopped —
because those are the readings that say whether a gap came from the radio or from the host.
A device whose firmware states nothing at all publishes which tuner the unit carries.
A reading an operator watches while receiving is brief: the stream state, an overload,
dropped samples, a lock, a measured error. A transport diagnostic and the unit's name are
not, and a one-line glance leaves them out.
A sweep asks for the whole set or for the readings one poll can afford.
A value a device library reads only when it opens the device — a transport size, a
converter clock, a wire format — is a setting rather than a control, because a write of it
while the device is open reaches nothing.

**guarantees**
- A sample the hardware delivered is never dropped on the way to the graph, and what the
  device shed before delivering it is counted in samples. A ring refuses a chunk and counts
  it whole, and what a ring is handed is whole samples, so a refusal loses a whole number of
  them and nothing after it lands on the wrong half of one: a transport that ends a transfer
  half way through a sample has its odd byte held back for the next transfer, at the place
  where the transfer is taken and before the ring can refuse anything. A device that
  timestamps its stream is counted in whole samples by the gap between where a block was
  expected and where it arrived, measured only against an expectation formed at the same
  rate and on the same device clock: a rate change or a clock set moves the stamps without
  losing a sample; and a source that discards a backlog itself, as a rate change does, counts
  what it discards where it discards it. A ring that counts the bytes a transfer carried
  converts to samples in one place, because every caller of the count reads samples. The
  count a block publishes is in the samples it publishes, whatever the device's own stream
  carries, because a consumer binds every family's count to one figure. A gap a full buffer
  caused is counted apart from one a lost packet caused, where the device tells the two
  apart, and the marker tag says which it was: a host that stopped draining causes the first,
  and the link between the radio and the host causes the second. A ring a block drops whole
  carries its count forward rather than restarting it, so the difference between two reads is
  the loss between them. One marker goes out on the first block published after a loss,
  however the loss arose, and losses that fall before that block share it, naming the latest
  cause where the family names one. It carries gr:n_dropped_samples, the samples lost since
  the last marker. The counts of a stream's markers sum to the loss they cover. A marker
  whose gap has no measured size carries no count: a report that states no length, a counter
  step of unknown size, a block with no stamp to measure against, a restart a vendor library
  reports without a length. A marker no publish has placed yet is owed to the first sample a
  later publish places, and a later loss joins it as one marker there, because a reader that
  merges the tags of one sample keeps the later count alone. A marker the output port
  had no room for is counted, and every source reports the count, because a stream missing
  its marker reads as a continuous one.
- A discontinuity marker sits at the head of the block the gap falls in front of. A ring
  that refuses the newest chunk refuses only while it is full, so the samples it sheds come
  after every sample already in it: the loss is noticed on one block and belongs to the
  next. A gap the device's own counter reports lies inside the block that carried it and is
  marked there.
- A control surface stated for a probed device is that device's. Which gain elements exist
  and what each reaches, whether the frontend carries an automatic gain control or a
  digital correction, whether the analog filter has room in it, and which converter clocks
  and reference sources are reachable, are all read from the device and none of them is
  assumed from a model name. A facility the device does not carry is not offered, because a
  vendor library takes the write for one that is absent, warns and changes nothing — whatever
  its own documentation says about refusing such a write, which on the version measured here
  it does not do. A vendor library that opens several boards reads the board at enumeration
  and again from the opened unit's own firmware, and the second answer wins. A call the
  firmware's stated interface version does not carry is a facility the unit does not have:
  the control for it is left out where that version has been read, and the write is refused
  and named where it has not.
- A rate change drops what the device already handed over and marks the discontinuity —
  the ring where the block owns one, and the vendor's own transport where the stream can be
  stopped and emptied around the write. Those samples were taken at the old rate, and the
  block forwards the new rate as a tag at the current write position, so publishing them
  behind it would label them with a rate they were not taken at and time them by it as well.
  The block already drained out of a ring is dropped too: the publish holds and retries and
  applies staged settings inside its own loop, so it carries the stream's own generation and
  abandons a block whose generation has passed. The publish answers how much of the block it
  handed over, and the part it abandoned is counted as lost. A marker names which of the three made the
  hole: a buffer that ran over, a packet the link lost, or the block's own drain across the
  change. A write the device refuses leaves the setting on the rate the device still holds,
  read back from it, because a stream counted at a rate the radio never took reads the
  difference as shed samples on every block. Anything the device dropped at its return to
  idle stays dropped. A control that acts on the world rather than on reception — antenna
  power on a connector — is written by the caller that asked for it and by nobody else, and
  the block says out loud that the change took it away.
- One device serves one block at a time, and a block that holds one lends it rather than
  making a second caller open it again. A half-duplex radio refuses a second open while it
  streams, so a transmit sink in the same process is handed the receiving block's own
  handle: the receive stream stops for the burst, the gap is marked, and every setting the
  burst changed is written again before the stream comes back. Exactly one of the two closes
  the handle and the other makes no device call on it afterwards: a teardown that overtakes a
  burst waits a bounded time for the handle to come home and closes it, and where the burst
  outlasts that wait, ownership passes to the borrower and the close happens when the handle
  comes home. A publisher that gave up ownership stops there rather than putting the
  transceiver back to idle, which would end the borrower's transmission.
- A timing tag names when the first sample of its block was taken. Where the device
  timestamps its stream, that timestamp is the tag whatever a caller asked for, carried into
  the host's epoch by one reading of the two clocks beside each other unless the caller asked
  for the device's own epoch: the device stamps the sample rather than the delivery, so
  neither the transport's backlog nor the host's scheduling is in the reading. Where the
  device timestamps nothing, the tag is a clock read at the drain with the backlog
  subtracted, and everything else that sits between the sample and that clock is subtracted
  with it — a capture backend's own buffering, read back from the opened stream rather than
  assumed, and the group delay of a conversion the source performs, the first published
  sample being centered that far ahead of the block's first input sample. A device clock set
  while a pulse-per-second source is in force takes a whole second on the next edge, and the
  second it takes is a disciplined oscillator's own where one reports a fix, so the
  timestamps are then that oscillator's time rather than the host's. Until that edge arrives
  the device clock is neither clock — it is the time since the device came up — so the tags
  stay in the host's epoch until a block stamped at or past the second asked for has
  arrived, and a set that never reaches the stream leaves them there and says so.
- A frequency outside the device's stated coverage is refused rather than clamped. The
  frequency the block then states is the one the tuner holds, read back from the device where
  the library offers such a read and otherwise the value the block commanded, and carried
  back out of any frequency correction the block applied as a scaled tune, so a caller can
  write the stated value back without the correction being applied twice. Before the first
  tune of a session there is no such value: the tuner sits where an earlier program left it,
  so a refusal at that point forwards nothing rather than a number the device never took.
  After a control write or a failed re-tune that leaves the block no tune it can state, as a
  change of sampling path away from the held frequency does, the block forwards nothing
  until a tune lands.
  A device whose coverage moves with the sample rate states it again at every rate change,
  because a request legal at one rate falls outside the radio at another. Where a local
  oscillator is deliberately offset from the center, the coverage has to hold the frequency
  the offset puts the frontend at rather than the center alone: a vendor library coerces a
  frontend past the end of that coverage without refusing it, which is the same silent clamp.
  The offset itself is bounded at the device by what the control states, half the rate in
  force, because a caller holding the block directly applies no bound of its own, and a
  frequency correction scales the center and not the offset: the digital converter that
  removes the offset runs at a tick rate the same reference sets, so both stages move by the
  same fraction and the net center lands on the request whatever the offset is.
- A correction stated in parts per million is pre-compensated where the device has no
  register for one, as the request divided by one plus the fraction, which is exact for the
  center; the first-order product differs from it by the request times the square of the
  fraction, which at the top of a wide tuner's range is larger than the tuner's own
  resolution. A correction fixes the center and nothing else. The same reference sets the
  converter's clock, so a stream stated at a nominal rate is sampled at that rate times one
  plus the fraction, and a caller reading its frequency axis from the nominal rate places a
  component of true offset b at b divided by one plus the fraction, short by b times the
  fraction: 100 Hz at an offset of 1 MHz and 100 ppm. A transmit block carries the same error
  the other way, a component asked for at offset b leaving at b times one plus the fraction.
  No block scales the rate it states, a consumer's frequency axis being its own. The rest
  hand the figure to the device and let it apply its own.
- A filter wider than the rate in force never reaches a device whose filter is the last one
  ahead of the rate. A device whose own digital converter filters to the rate behind the
  analog filter states its exception in its own knowledge file. The width the block states
  is the width the device settled on, read back after the write. A vendor library
  coerces a filter to the nearest value it can reach rather than to the value asked for, and
  the nearest value to a request can be wider than the request, which passes the signal the
  request was made to exclude: a block that chooses for itself takes the widest value at or
  below the request instead. Where a library publishes no read-back and resets the filter
  behind the block on every rate write, the block writes the filter itself after the rate and
  states what it commanded. A filter whose stated range has equal ends is fixed in the
  hardware: the block holds the width the device came up with, writes nothing to it, and
  offers no control for it.
- A rate ladder offered for a probed device comes from what the device said, except where
  the stated set is the ladder of one clock setting on a radio whose clock is programmable.
  There the stated set moves with the rate and a ladder taken from it would cap the radio at
  whichever setting the probe caught. A fixed clock, one the radio states as exactly one
  rate, states its own ladder instead, and the
  rates the radio reaches are that clock's even decimations, cut at the rate the vendor
  library states for the link, in the wire packing in force: an odd decimation runs the cascaded-integrator-comb
  decimator alone, the halfband filter out of the chain, and droops across its own passband,
  and a rate past the link delivers nothing at all. A converter clock a caller pinned is a
  fixed clock until the device is opened again, whatever range the device states, because a
  vendor library's own choice does not come back while the device stays open. What a caller
  is offered to choose from is shorter than what the radio reaches, since a fixed clock
  reaches hundreds of rates and the ones below a megasample sit less than a percent apart; a
  request between two entries is snapped to the nearest rate the radio reaches, so the rate
  the block states is one it chose rather than one a vendor library coerced. The link's rate is
  the figure the vendor library states for the link in use, compared as that library compares
  it, with nothing taken off; the selector and the transport it names state nothing. A figure
  the library writes for every unit of a kind, whatever its link, and never compares states no
  link. A link nothing states cuts nothing: the ladder then reaches the radio's own fastest
  rate, the line naming the unit says so, and what the host could not carry is said once, from
  the count delivered against the rate in force.
  A device whose clock is programmable, across a continuous range or among several stated
  rates, offers its ladder as a set
  of picks instead, with the fastest clock itself where no pick reaches it: a request
  between two entries reaches the device as it stands and the block states it, because
  snapping would refuse a caller a rate the radio can run. A link rate the vendor library
  states caps both on every clock kind: the offer stops at that ceiling in the wire packing in
  force and ends with the ceiling itself, and a request past it is written at the ceiling. A request
  outside the range, or one that is not a number, is brought to the nearer end before any
  narrowing cast sees it.
- A gain element set while the device's automatic gain control holds the gain path is held
  rather than written, and reaches the device when the AGC lets go. An overall gain set
  under the same condition is held the same way, down to the element values it implies, so
  leaving the AGC lands on the last request a caller made rather than on an older one. Sent
  under it, such a write is lost, fights the AGC, or switches it off behind a setting that
  still reads on. A held element reaches the device only where the device open now names
  that element: the names belong to the model, and a block restarted against another one
  holds names that model does not have.
- A device string names one unit and keeps naming it. A selector is built from something
  the device publishes and a second enumeration repeats — a serial, or the bus and address a
  unit sits at — and never from a position in a list whose order nothing states. A serial
  carrying a character the selector's own syntax uses is not offered as one: the unit is
  named another way, and the listing says why.
- A block that has a device open names the unit from that handle rather than from a fresh
  enumeration, and files what it learns under that name. The enumeration stays for resolving
  a selector before anything is open and for the case where the device refuses the read: it
  opens every unit attached, including one another process holds, where the read is one
  control transfer on a handle this process already has.
- Leaving an AGC writes the mode and then the gain the block holds: the overall gain
  first, then every element a caller moved while the loop had it, so the device ends on
  the more specific value rather than on whatever the overall setting last carried.
- Every applier is idempotent and makes no device call for a value already in force,
  because the staged settings map is replayed after streaming starts and a redundant rate or
  tuning write disturbs a running radio for nothing. Each applier remembers its last
  written value per open device, since a freshly opened device holds none of the previous
  one's state.
- A source whose output port reaches nothing ends its own block. The publish holds and
  retries while the device goes on delivering into a ring that sheds, so nothing inside the
  block would end it, and the framework's own test for an unconnected downstream sits in a
  work path a device source overrides. The work preamble makes that test and asks for the
  stop. It names a fault only for a port that had no reader when the source started: a
  consumer that finishes releases its readers, so an output that lost its reader is a graph
  ending from its consumer side, and that stop goes without a message. A source whose
  `disconnect_on_done` setting is false stays, as the framework's own test leaves it.
- A joinable device thread never outlives its block.

**invariants**
- A device handle is dereferenced only under the control mutex, by a holder that re-tested
  liveness under it, and it is written under that mutex as well, together with whatever a
  reader of it needs to know — the model, the serial, the claim flag. The start hook is the
  one exception and owns the handle alone: the framework holds one mutex across a block's
  whole start sweep and the same one across its stop sweep, so the two hooks of one block
  never run together, and the liveness flag stays false until start raises it, which keeps
  every other holder out. Nothing that runs after that flag is raised may read the handle
  without the mutex.
- A control entry point of a source reads no settings member. The settings drain rewrites
  those members on the streaming thread under the settings machinery's own mutex, which a
  block never takes, so a control path called from another thread has no lock in common with
  the writer — and for a vector setting the read can follow the reassignment that freed the
  buffer. Each applier therefore keeps what the device was given in a member guarded by the
  control mutex, and the control paths read that.
- `TeardownGuard` is the last data member of every source, so it runs first. No buffer
  a device callback writes into may be declared after it: members are destroyed in reverse
  declaration order, so such a buffer is freed while the callback is still running.
- Teardown clears liveness, joins the thread, and only then closes the device.
- `hardwareTeardown()` is idempotent and returns true to exactly one caller, so a stream
  that started ends with exactly one end-of-stream tag. A start that fails publishes none,
  because there is no stream to end.

**domain**
A device's control surface is not a set of continuous sliders. A descriptor states the real
thing and a caller draws what it is given.
One driver covers models that differ in everything a control surface is made of. Every such
difference is read from the device, and a case written for one model measures that model.
The cost of reading a sensor is a property of the unit, not of the model: an oscillator
disciplined by a satellite fix answers its epoch-time sensor from a 1 Hz serial link, where
a read waits for the next sentence, while the same model without one answers in
microseconds. A driver therefore times one read of each sensor and keeps the cheap ones,
rather than carrying a list of names.
A gain, a rate or a filter written to a device that reports the request straight back is
indistinguishable from one that landed, which is why the decisions are pure functions and
the tests assert which device calls would be made.

**rejected**
A device-agnostic layer under every family: it flattens the control surfaces into ranges
that do not exist, and a generic idea of a gain is not the device's.
A starvation detector in `work()`. A host that has stopped draining is an ordinary state
that recovers by itself; only a receiver that is gone ends a stream.
Inferring a wire packing from how far a block counter advanced. A missed block is
indistinguishable from a denser packing, and the unpacking then invents samples.

**fails**
A device removed or failing surfaces through the family's own liveness path and stops the
block. A vendor library that refuses a control transfer is logged and the stream
continues, because a message emitted from a settings handler becomes a scheduler
exception, and the log names the register or the control that was refused and the code it
was refused with. An exception leaving a device thread stops that block: the thread body
catches, names the exception and sets the block's own dead flag, because an exception
leaving a `std::thread` ends the whole program. A vendor callback whose thread may not
block or allocate is given no work of that kind: a buffer it writes is sized when the
stream is opened and an error it meets is stored for a thread that may speak.

## The sink family

**purpose**
One GNU Radio 4 sink block per radio, speaking the vendor library directly and taking the
setting names its source takes. A device's sink is the peer of its source.

**algorithm**
A sink is the same shape with a device-specific middle. `processBulk` clips what it is
given, stages it in a bounded ring and consumes exactly what fitted; a drain hands that ring
to the device. The appliers write the device under one control mutex, and the lifecycle
hooks — `start`, `stop`, `pause`, `resume` — open the device, release it, take the carrier
down and bring it back.

**io**
CF32 in, full scale 1.0 in each part. A sink reads the burst tags `gr:tx_sob`, `gr:tx_eob` and
`gr:tx_time` where its family's knowledge says so, and ignores every other tag. Settings in — the
source's names, with the `tx_` prefix for what a transmitter has and none for what it has
not, so one caller drives both directions identically. `enumerateDevices()` offers the pairs
the source offers, from one body both families call. `describeControls()` states the transmit
control surface. `setControl()` writes every id that surface states other than SAMPLE_RATE and
FREQUENCY, bounded by the domain the descriptor carries. A sink answers the four message
properties exactly as a source does. `readSensors()` publishes its own readings under `tx_` ids, the unit's
name as `device_model`, and whatever the device reports. The status readers take the control
mutex with a two-millisecond deadline, in every sink, and answer the device-is-down value where
they cannot get it inside it. That answer
means the reading did not arrive and is asked for again; it does not say the device has
gone. `deviceUp()` says that, and it takes no lock. It answers whether the block holds a
device with its sending path in place.

**guarantees**
- A part outside plus or minus 1 is clipped and never wrapped, in every sink. The block
  applies the bound, rather than whatever converts the samples afterwards, and a part that
  is not a number becomes zero in the same place.
- A work call never waits on the device. A ring with no room consumes nothing and answers
  `INSUFFICIENT_OUTPUT_ITEMS`, so the scheduler offers the same samples again and nothing the
  caller handed over goes missing. The ring's own drop counter never moves.
- The queue holds the duration the sink's bound states at the rate in force, a quarter of a
  second by default, rounded up to the power of two the ring indexes by, so the time held runs
  from a quarter of a second to just under half of one: long enough that a scheduling delay on the thread filling it does not starve the
  device, short enough that the tail a stop plays out is about a quarter of a second. That
  range holds at every rate above four times the family's own floor in samples. At that
  crossing and below it the floor binds instead, and the time held is the floor rounded up to
  a power of two divided by the rate: a quarter of a second at the crossing itself, and longer
  as the rate falls further. Each family states its floor and the rates its floor binds at. The
  queue is sized from the rate the device took at the start and resized on a rate change, in
  the bracket where the ring has neither a producer nor a consumer. A scheduler that parks its
  worker on the graph's progress counter has to wake inside that quarter second, and a drain
  here bumps no progress counter when it frees ring room: under
  `ExecutionPolicy::singleThreadedBlocking` that park is `timeout_ms`, 100 ms by default.
- The queue never takes more storage than the bound's byte ceiling, 4 GiB by default, which
  holds a quarter of a second of CF32 at 2 GS/s. A family whose sink states the bound as
  settings takes a host's own figures; one that states none takes the defaults. The floor wins
  over the ceiling.
- The queue costs memory as well as time, and the storage follows the rate in both directions.
  Each block allocates its declared ring at construction, 4 MiB, before any start. The ring's
  allocator writes no element, so a start or a resize pays for no pass over the storage, and
  the kernel zeroes each page as the producer first writes it: at 128 MiB that first pass
  takes about 55 ms of the producer's time against 11 ms over pages already written. A resize
  to the size the queue already holds keeps its storage. A resize that asks for less gives the
  difference back rather than keeping the peak the block ran at, and a caller holding a lock
  takes back the storage a resize replaced to free it outside the lock.
- A reading of what the queue holds never exceeds the queue's own capacity, so every deadline
  sized from it is bounded. Three threads take that reading while the two ends of the ring
  move: the load order keeps the subtraction from wrapping to about 1.8e19, and the clamp to
  the capacity bounds the answer. The capacity is published beside the ring
  rather than read from its storage, which a rate change replaces.
- A stop sends what the work calls already handed over before it releases the device, and says
  how much it could not send. Every wait is bounded by how long what is owed the air takes to
  play out at the rate in force, so a device that has stopped draining costs a bounded delay
  rather than a thread that never returns. The families owe different things, and each states
  its own.
- Every count of samples a sink states measures samples the block itself held. Where a device
  library holds samples it publishes no count of, what that library owed the air is stated in a
  reading of its own whose name says it is a bound, so a caller that sums the counts and moves
  a stream position back by the sum moves back over no air the device already sent. A family
  whose library owes nothing carries no such reading.
- A rate change sends what was staged at the old rate at that rate before the new one is
  written, waiting for it under the same bound, and sizes the queue for the new rate whether
  or not a burst is open. Those samples are the caller's, handed over and acknowledged, so
  they are not the receive side's drop-and-mark case, which is for data a device produced. The
  block throws away what the bound could not get out and counts it in a counter of its own:
  the resize hands both ends an empty queue, and the device that left those
  samples behind is one that stopped consuming, which is also one with no rate to send them
  at. A drain that did not stand off the ring inside its own bound keeps the queue instead:
  the rate is written alone and the block says so, a queue reassigned under a live consumer
  being worse than a queue holding the wrong length of time. The resize is owed from there
  rather than dropped. The consumer asks to stand off again while the queue holds more storage
  than the rate in force asks for, the producer takes the resize behind that answer, and what
  it throws away is counted under the same rate change.
- A rate change made while the carrier is held lands under the counter the family keeps for
  it, one sequence being counted once.
- A rate change takes no send mutex on the block's own worker thread. The burst end is handed
  to the drain through a flag instead, because every other block in the job waits behind that
  worker. It does wait on that thread, under the control mutex, and the wait has two bounds:
  the play-out of what is owed the air at the rate in force, which the family states for
  itself, and the drain's answer that it has stood off the ring, which against a device stalled
  inside a send is one send timeout.
- A burst that played with a hole in it can be told from one that did not, in the terms the
  device can honestly state. Both counts are per run and cleared by the start that begins the
  run. The transfer that ends a clean stop is silence the stop asked for and counts as neither.
- No sample a stop owes the air leaves the queue on the way to an answer the device library
  discards.
- A pause stops radiating without giving up the device. The silence a pause causes counts as
  no underrun: that silence was asked for. Each family states what a pause does with the queue
  it found and with the work calls made under it. A count a pause
  keeps is whole by the time the pause hook returns, so a caller reading it as soon as the
  hook returns reads the whole of it, and one that moves a stream position back by the count
  moves it back once.
- A resume sends again and the next samples open a fresh burst. A work call that could not
  take the control mutex for a write owed to the resume stages nothing and is offered the same
  samples again. A status reader holds that mutex for two milliseconds at most and an applier
  for its own play-out.
- Every applier is idempotent and makes no device call for a value already in force, because
  the staged settings map is replayed after the stream starts. A rate already in force is not
  written again; a new one ends the burst in flight first.
- The analog filter follows the rate: every rate change derives it again. Each family states
  how wide it sets the filter against the rate.
- A frequency outside the device's stated coverage is refused rather than clamped, and the
  refusal leaves the tuner where it is; a start whose frequency or rate the device refuses
  transmits nothing at all, rather than transmitting where the previous program left the
  radio. The coverage tested is the open board's, and one tuner serves both directions, so
  the sink and the source of a radio read one coverage.
- The frequency a block states is the one the caller asked for, beside the value the
  synthesizer was commanded. The correction stated in parts per million is applied as the
  request divided by one plus the fraction, in the sign convention the sources apply. That
  fixes the center and nothing else: the source family's guarantee above states how far a
  component at an offset from the center lands from where it was asked for, and a transmit
  block carries the same error, since neither direction scales the rate it states. A status
  line stating the commanded value would sit 100 kHz from the request at 100 ppm and 1 GHz,
  and a caller writing it back would have the correction applied twice.
- The rate a block states is the rate the device runs. A request the block bounded or the
  device coerced is written back into the setting, before a stream starts and at every change,
  so every reader, the rate tag on a stream's first block and every forwarded tag carry one
  number.
- A setting a block writes back reaches the block's own readers as soon as the applier returns.
  The framework's parameter map holds a copy of the members rather than a read of them, and a
  sweep takes that copy twice: once before it calls the block's settings hook and once after
  it. A write-back made by an applier the hook runs therefore reaches the map at the end of the
  same sweep, and one made outside a sweep waits for the next sweep. A caller taking the value
  from the block's own reader sees either at once.
- A gain reaches the device inside the range the device states. A vendor library coerces a
  request past that range without refusing it, so the bound is applied here and the move is
  said out loud.
- A device another block in this process already holds is borrowed rather than opened again.
- Exactly one block closes a lent handle. Where the publisher's teardown outlasts the loan,
  ownership passes to the borrower and the publisher makes no further device call on the
  handle, because a stop of its own stream would end the burst running on it.
- A transmitter that the graph feeding it has left behind stops itself, where the device offers
  a limit for that. The limit is a setting whose default is no limit at all, because a low one
  stops a healthy burst.
- A drain that has ended ends the block: the work call reports it and requests the stop.
- A stop asked for during a start's own control transfers arms no transfer. The lifecycle
  state is read again after the open and after the configuration.

**invariants**
- A control entry point of a sink reads no settings member. The settings machinery rewrites
  those members on the block's own worker thread under a mutex the block never takes, and for
  a vector setting a read from another thread can follow the reassignment that freed it, so
  each applier keeps what it was given in a member guarded by the control mutex and the
  control paths read that.
- No sink writes the antenna port's power. That control feeds direct current up the coax into
  whatever is connected, and it belongs to the block the caller asked it of.
- `TeardownGuard` is the last data member of every sink, so its idempotent teardown runs while
  the rings, the scratch buffers and the `gr::Block` base are all still alive. No buffer a
  drain indexes is declared after the thread that drains: members are destroyed in reverse
  declaration order, and a teardown that raised — which the guard swallows — would otherwise
  leave the drain reading storage that has gone. Nothing in either teardown body raises.

**domain**
A part past full scale cast into an integer format wraps to the opposite sign, which turns an
overdriven sample into a loud one of the wrong polarity: a different signal, spread across the
whole band, rather than a distorted version of the intended one. Whether a given converter
saturates depends on which path a build selected, so nothing here relies on it.

**rejected**
Repeating the last buffer when a transfer starves: that transmits a signal nothing produced.
Blocking inside a work call until the device takes the samples. The scheduler's thread
belongs to every block in the graph, and a ring answers a full edge in one comparison.

**fails**
A device that cannot be opened, a rate or a center the radio cannot reach, or a gain the device
refuses ends the start through the failed-start path of the shared files. A transient control
failure during a settings change is logged and the stream
continues, because a message emitted from a settings handler becomes a scheduler exception. A
send that cannot place its samples ends the drain, and the work call ends the block behind it.

## The shared files

**io**
Every block of either family answers four GNU Radio 4 message properties, started or not. Its
`ControlProperties` member, built from the block's own pointer, registers them when the block
is built, and `answerControlProperty` in `Properties.hpp` answers them. A caller outside the
graph writes a message to the scheduler's message input, with the block's unique name as the
service and the property as the endpoint. The reply arrives on the scheduler's message output
under the same endpoint and client request id, as a final message; a push to a subscriber
arrives as a notification under the subscriber's own id. A caller holding a block that no
scheduler runs connects its own ports to the block model's two message ports, writes the
request and calls `processScheduledMessages()` on the model from its own thread.
- `controls` answers Get and Subscribe with `{controls: [descriptor, ...]}`, the block's
  `controlSurface()`: `describeControls()` for the device the block holds, at the rate and
  the frequency in force. A block that has not started, idle or initialized, answers
  `probeControls` over its own type for the selector, the rate, the first frequency, the gain
  mode and the wire format it holds staged, or set where nothing is staged, and an error where
  that probe could not open the unit. `TruthContext` carries the gain mode and the wire format
  for the probe; an empty wire format is the family's default. A subscriber receives the same body again each time the surface differs from the one
  last sent. A block checks after each settings drain, at the end of each start and after each
  write through `control`. Unsubscribe takes no reply, and Set is refused.
- `control` answers Set of `{id, value}`. A descriptor that is a gain goes through
  `setElementGain` where the block has one, and every other id through `setControl`. The
  reply is `{id, value, requested, read_back}`. `value` is the value in force: the block's
  own read-back where `read_back` is true, from `elementGain` for a gain element or the
  family's `controlValue`, and the request bounded by the descriptor where it is false. An
  error reply names the id and says which refusal it met: the surface states no such id,
  the device refused the value, or the block holds no open device. `SAMPLE_RATE` and
  `FREQUENCY` go through neither: the write stages `sample_rate` or `frequency` unbounded, on a
  started block or not, and the reply carries the staged value with `read_back` false. The
  block applies it at its next settings drain as any setting write. `GAIN_MODE` stages
  `gain_mode` the same way, true for any value but zero, where the surface the block states
  carries the descriptor, and is refused as an unknown id where it does not.
- `sensors` answers Get with `{sensors: [reading, ...]}`: the stream state first, then
  `readSensors()`, or `cachedSensors()` where a block keeps a cache. A request carrying
  `{sweep: "full"}` asks for a family's full sweep, and for the brief sweep otherwise. A cached
  answer of either kind carries the newest reading of each id that any sweep has landed, and
  the sweep the request queues lands for a later request. A full request made before any full
  sweep has landed answers the brief readings and `sensors_whole_pending`. A block whose
  device is down answers the stream state alone, and a block whose device is up and did not
  answer the sweep answers an error. A subscriber receives every set a Get produces for
  another client. Subscribe and Unsubscribe take no reply.
- `devices` answers Get with `{devices: [{label, selector}, ...], opens}` from the family's
  `enumerateDevices()`. `kListingOpens` on every block says whether that listing opens a unit
  to name it; such a family lists nothing unless the request carries `{allow_open: true}`, and
  `opens` repeats the constant. A listing that throws answers an error naming the cause.
- Every surface ends with `SAMPLE_RATE` and `FREQUENCY`, which `appendRateAndFrequency` in
  `ControlDesc.hpp` builds. `SAMPLE_RATE` is a list in hertz over the family's ladder,
  `anyInRange` over the span a device takes whole, and bounded by the ladder's ends otherwise.
  `FREQUENCY` is a range in hertz whose min and max are the outer ends of the coverage, with
  `ranges` holding the spans where there are several. Each default is the family's default
  setting, so neither moves with a tune. A family that states no ladder or no coverage leaves
  the descriptor out.
- `GAIN_MODE` is a toggle that is not a gain, stated where the unit has an automatic gain
  mode, before `SAMPLE_RATE`. Its default is the `gain_mode` the block holds: `TruthContext`'s
  gain mode for a probe, and for a running block a mirror its start and its settings drain
  write, which `controlSurface()` reads without a lock. A family without the mode states none,
  so its presence in a surface says whether `gain_mode` does anything.
- The stream state is `streamStateOf()` in `Properties.hpp`, one of the words of
  `stream_state`: `failed` in the error state or where the family's `streamDead()` says the
  stream gave up, `streaming` or `starting` while running as the device is up or not, `paused`
  while paused, and `stopped` otherwise. `streamStateReading()` states it under
  `rx_stream_state` or `tx_stream_state`, brief, and good while streaming. The analog
  bandwidth, `rx_analog_bandwidth`, is a family's own reading, built by
  `analogBandwidthReading()` as whole hertz and read back by `analogBandwidthIn()`. Every
  source states the samples it lost, `rx_dropped_samples`, brief, and after it the tags its
  port refused, `rx_dropped_markers`, not brief, built by `droppedSamplesReading()` and
  `droppedMarkersReading()`. A count has no pass or fail sense and reads good.
- A descriptor map carries `id`, `label` and `unit` as strings, `kind` as a 32-bit integer,
  `min`, `max`, `step`, `default` and `inoperative_above_hz` as numbers, `options` as a list
  of strings, `list_values` as a list of numbers, `is_gain`, `rate_derived`, `session_only`
  and `any_in_range` as booleans, `applies_at` as the string `runtime` or `start`, and `ranges`
  as a list of `[min, max]` pairs, empty for one span and read empty where the key is missing.
  A reading map carries `id`, `label` and `value` as strings and `good` and `brief` as
  booleans. `controlToMap`, `controlFromMap`, `sensorToMap`, `sensorFromMap`, `devicesReply`,
  `devicesFromReply` and the reply helpers beside them are the one spelling of these keys.
- `anyInRange` on a list descriptor says its device also takes every value between `min`
  and `max`, beside the entries of the list, and `clampToControl` bounds such a list to the
  span of both. A list without it takes its entries alone. A caller draws a list as a
  selection over its entries, and lets a person enter a value in the range where the
  descriptor carries it. `false` is the default of a seed, a descriptor and a map without the
  key.
- `appliesAt` on a descriptor says when a write takes effect: `Runtime` in the running
  stream, `Start` at the next start of the stream. `setControl` refuses a write to a stopped
  block. A family either keeps a `Start` write made while the device is up for the next start,
  or takes only the value in force and refuses any other, as its README states. A caller
  that builds
  a new block for the next start passes the value as the open-time setting the family's
  README names for the control. `Runtime` is the default of a seed and of a descriptor, and
  `controlFromMap` reads it for a map without `applies_at` and for any word there other than
  `start`.
- `listDevices<Blocks...>()` answers one `DeviceListing` per unit and family: the family's
  `kDevice`, the listing's label and device string, and whether a source and a sink of the
  family were given. `probeControls<Blocks...>(family, transmit, ctx)` answers the list the
  `controls` property of a running block gives for the same device at the same rate. A
  family whose surface needs an open device names its record as `Truth` beside
  `probeTruth`, which the probe calls first, and the probe of such a family opens the device
  for its length. `listFamilies<Blocks...>()` answers one `CompiledFamily` per
  family the blocks belong to, with the directions given for it, and asks no device.
- `capture::discovery::families()`, `listDevices()` and `probeControls(family, transmit,
  ctx)` in `DiscoveryLibrary.hpp` answer as the three templates do over every block of the
  families a build lists. A device directory's `include/capture/<device>/Discovery.hpp` states
  its family's blocks as one `BlockList`, and the build writes a header of its own that
  includes the listed fragments and names their lists; the source joins them in that order
  and links each listed family's vendor library. Every library built from the source defines
  the same names, so a process holds one.
- A block that keeps a record of its non-gain controls reads them back through
  `controlValue`. A block that keeps none answers a non-gain write with the bounded request.

**guarantees**
- A block of either family whose start fails ends the run it is in. The block releases what
  its start opened, prints the reason on the console under its own tag and throws the reason
  from `start()`. The framework puts the block in ERROR, the scheduler winds the run down, and
  `runAndWait()` fails with the block's reason. A block whose start threw is never given
  `stop()`, so the release comes before the throw. A stop a block asks for inside its start
  would end that block alone: the scheduler counts the start as a success, and a block that
  never streamed publishes no end-of-stream, so its consumers would wait for samples that
  never come.

**invariants**
- Every block declares its `ControlProperties` member before its teardown guard, built from
  the block's own pointer. Its settings
  hook declares a `ControlSurfaceCheck` as its first statement, so the surface is sent on every
  return from the hook.
- No block's `controlSurface()` takes a lock that a device call is made under. A block whose
  surface follows the device it holds computes the surface from values its writers publish
  through a `Snapshot`, after each write and under the lock that write holds, so the thread
  that drains the device never waits on a device call to read the surface.
- No block of either family requests a stop while it holds its control mutex. The request runs
  the block's own stop on the thread that makes it, that stop tears the hardware down, and the
  teardown takes that same plain mutex, so the thread would wait on itself. A body that fails
  under the lock carries the failure out of the locked scope and makes the message and the
  request behind it. A start that fails under the lock carries the failure out the same way,
  because the teardown that releases its device takes that mutex too.

**fails**
A property answer runs on the scheduler worker that serves the block's messages, and it
costs that worker the block's own call. A full sensor sweep can wait on a slow link to the
unit for most of a second, which is why the brief sweep is the default.

**note**
Nothing in a device directory is compiled into the library. Those headers instantiate in a
consumer that has GNU Radio 4 and the vendor headers; the `capture-headers` target compiles
one translation unit per device header so that a mistake is a diagnostic here rather than in
whichever consumer got there first.
`Env.hpp` and `Threading.hpp` have twins in a consumer's own kit: the same code under
another namespace, in two repositories. This is the capture library's own copy, which its
device threads use, and no translation unit sees both — but one process can hold both, and
only one of them may adopt the fast cores. The base set a copy records is the affinity the
process holds when it runs, so a second copy reads a set the first has already narrowed,
finds every processor in it quick, and settles on "all of one kind": its `useAllCores()` is
then a no-op for the life of the process and a background thread demoted through it keeps
the quick cores. Nothing in this library calls `runInBackground`, `lowerOwnPriority` or
`useAllCores`: the device threads name themselves through `nameStreamingThread`, and a
thread whose caller asks for a placement takes it through `placeStreamingThread`, which pins
it to the one processor asked for and asks for the real-time round-robin class. The
constraint falls on whichever program links both copies.
`placeOwnThread` asks for nothing where the priority is 0 or below and the processor below
0, and the thread then keeps the class and the processors it started with. A refusal of
either names the reason and changes nothing about that part of the thread.
`ControlWorker::settleFor` bounds the wait `settle` makes by a duration and answers whether
the worker settled within it.
