/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <gnuradio-4.0/LifeCycle.hpp>
#include <gnuradio-4.0/Message.hpp>

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Properties.hpp>
#include <capture/common/Ring.hpp>
#include <capture/common/Threading.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <source_location>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>

/*| role: the parts every device block shares, whichever direction it runs in: the reference
        correction, the duties a source owes its output port, the start failure, the thread
        naming and placement, the streaming and control threads, and the teardown and control
        guards.
    contract: every applier a settings change reaches skips a write of the value it last
        wrote. Each applier sees each staged value at least twice, before the start and again
        once the stream runs.
    trap: a rate write that reaches a streaming device can stop its sample clock until the
        device is reset.
*/
namespace capture {

/*| contract: the scale a reference error of ppm parts per million puts on a tuner, one plus
        the fraction.
    invariant: a correction of minus a million parts per million or below answers one, which
        applies no correction.
*/
constexpr double referenceScale(double ppm) {
    const double factor = 1.0 + ppm / 1e6;
    return factor > 0.0 ? factor : 1.0;
}

/*| contract: the frequency to command a tuner for a request under a reference error of ppm
        parts per million.
    frame: a positive correction states that the reference runs fast. A fast reference makes
        the synthesizer land above where it was told to by the same fraction, so the command is
        the request divided by that factor and the radio ends on the frequency asked for.
    why: the exact form. The first-order form is off by the request times the square of the
        fraction, 60 Hz at 6 GHz and 100 ppm, against a tuning resolution of about 50 Hz.
    verified-by: tune.a-fast-reference-tunes-low
*/
constexpr double correctedTuneHz(double requestHz, double ppm) { return requestHz / referenceScale(ppm); }

/*| role: what a source keeps about its output port for the duties it owes that port.
    invariant: a member of every source, declared before the teardown guard.
*/
struct OutputState {
    std::atomic<bool>        connectedAtStart{false};
    std::atomic<std::size_t> droppedTags{0};   // the port's refused-tag count, as last published
    std::atomic<bool>        rateTagOwed{false}; // the stream's first block carries gr:sample_rate, and gr:frequency where the frequency setting holds one
    // A gap marker no publish has placed yet, its cause and its count. The publishing thread
    // writes them, and noteOutputAtStart() clears them before that thread starts.
    bool                         gapOwed  = false;
    const char*                  gapCause = nullptr;
    std::optional<std::uint64_t> gapCount;
};

/*| contract: record whether the block's output port had a reader when the block started, and
        owe the stream that starts the rate and frequency in force as tags on its first block.
        Each source calls this in its start() before its publishing thread starts, after any
        teardown that joins the thread of the stream before.
    frame: the scheduler connects every edge of a graph before it starts a block. A port that
        reads unconnected here was never given a consumer.
*/
template <typename Block>
void noteOutputAtStart(Block& blk) {
    blk._outputState.connectedAtStart.store(blk.out.isConnected(), std::memory_order_release);
    blk._outputState.rateTagOwed.store(true, std::memory_order_release);
    blk._outputState.gapOwed = false;
}

/*| contract: true where the block's output port reaches nothing, having asked the block to
        stop. False where the port reaches something, or where disconnect_on_done is false.
        A port that had no reader at the start is named as a fault on the message port. A port
        that lost its reader is the ordinary end of a graph, and the stop carries no message.
    trap: a source whose output reaches nothing never ends its publish by itself: the edge
        fills once and the streaming thread retries while the device sheds. Each source calls
        this from its work() override, behind its own lifecycle test.
    verified-by: the device source's own case ending an-unconnected-output-ends-the-block,
        which also reads the source's droppedTags()
    verified-by: publish.a-block-kept-when-nothing-reads-it-is-not-ended
    verified-by: publish.an-ordinary-end-from-the-consumer-names-no-fault
*/
template <typename Block>
bool stopWhenOutputIsUnconnected(Block& blk) {
    if (!blk.disconnect_on_done || blk.out.isConnected()) {
        return false;
    }
    if (!blk._outputState.connectedAtStart.load(std::memory_order_acquire)) {
        blk.emitErrorMessage("work()", "the output port is connected to nothing");
    }
    blk.requestStop();
    return true;
}

/*| role: what a device block's start() throws when it cannot proceed.
    contract: what() answers the reason alone, and sourceLocation keeps the place of the throw.
*/
struct StartFailure : gr::exception {
    using gr::exception::exception;

    [[nodiscard]] const char* what() const noexcept override { return message.c_str(); }
};

/*| contract: end a start that cannot proceed. The reason goes to standard error under the
        block's tag, and a StartFailure carrying it leaves start(). The run then fails with the
        reason.
    invariant: the caller has released whatever its start opened. A block whose start threw is
        never given stop().
    verified-by: start.a-failed-source-start-ends-the-run
    verified-by: start.a-failed-sink-start-ends-the-run
*/
[[noreturn]] inline void throwStartFailure(std::string_view tag, std::string_view reason, std::source_location where = std::source_location::current()) {
    std::fprintf(stderr, "[%.*s] the start failed: %.*s\n", static_cast<int>(tag.size()), tag.data(), static_cast<int>(reason.size()), reason.data());
    throw StartFailure(reason, where);
}

/*| contract: carry the port's own count of refused tags into the block's copy. Called on the
        publishing thread after each tag the block publishes, which is the one thread that moves
        the port's count.
*/
template <typename Block>
void noteDroppedTags(Block& blk) {
    blk._outputState.droppedTags.store(blk.out.nTagsDropped, std::memory_order_relaxed);
}

/*| contract: the tags this block's output port refused since the graph was built, as of the
        last tag the block published. Safe from any thread. A refused gap marker is a gap the
        stream never reported. Each source answers this as droppedTags().
    trap: the port refuses a tag when its tag ring is full. A run of very short blocks that
        each carry a marker fills it.
    frame: the reading is the block's atomic copy, because the port's own counter is a plain
        integer only the publishing thread may read.
    verified-by: publish.the-lost-marker-count-is-read-from-the-block
*/
template <typename Block>
std::size_t droppedTagCount(const Block& blk) {
    return blk._outputState.droppedTags.load(std::memory_order_relaxed);
}

/*| contract: name the calling streaming thread <tag>:<blockName>.
    frame: tag names the device library and the thread's job in one short word, and blockName
        the block that owns the thread.
    trap: the whole name holds capture::threading::kMaxThreadNameLength characters. A tag over
        eight characters cuts into the block name.
    why: capture::threading::nameOwnThread shortens an over-long name. The framework's own
        setter leaves such a thread with the executable's name.
*/
inline void nameStreamingThread(std::string_view tag, std::string_view blockName) {
    std::string name(tag);
    name += ':';
    name += blockName;
    capture::threading::nameOwnThread(name);
}

/*| contract: place the calling streaming thread as priority and cpu ask, on the scale
        capture::threading::placeOwnThread states, and name on standard error under blockTag
        what was asked and what the system granted. A thread asked nothing is left alone and
        named nowhere. Answers the placement.
    contract: a placement the system refuses is a line on standard error, never a failed
        start.
    verified-by: threading.an-asked-placement-is-applied-or-named
*/
inline capture::threading::Placement placeStreamingThread(std::string_view blockTag, std::string_view role, double priority, double cpu) {
    const auto placed = capture::threading::placeOwnThread(priority, cpu);
    if (placed.asked()) {
        std::fprintf(stderr, "[%.*s] %.*s thread: %s\n", static_cast<int>(blockTag.size()), blockTag.data(), static_cast<int>(role.size()), role.data(), placed.describe(priority, cpu).c_str());
    }
    return placed;
}

/*| role: a block's streaming thread, stopped and joined when the member is destroyed.
    contract: the block's stop() joins it on the ordinary path. The destructor joins it where a
        stop raced a slow start and the block's stop() hook never ran.
    why: a block declares no destructor of its own, and a thread still joinable at its
        destruction ends the process.
*/
struct ConsumerThread {
    std::thread       thread;
    std::atomic<bool> stopFlag{true}; // true = not running / stop requested

    void requestStop() { stopFlag.store(true, std::memory_order_release); }
    bool stopRequested() const { return stopFlag.load(std::memory_order_acquire); }
    void clearStop() { stopFlag.store(false, std::memory_order_release); }

    /*| contract: run body on this member's thread. Whatever thread the member already holds is
            asked to stop and joined first, and the stop flag is clear before body can read it.
        trap: body catches what it throws. An exception that leaves a thread body ends the
            process.
        verified-by: consumerthread.a-start-replaces-the-thread-it-holds
    */
    template <typename Body>
    void start(Body&& body) {
        requestStop();
        joinIfRunning();
        clearStop();
        thread = std::thread(std::forward<Body>(body));
    }

    void joinIfRunning() {
        if (thread.joinable()) {
            thread.join();
        }
    }

    /*| invariant: the destructor's bare join assumes that a thread parked in a device-library
            call which ignores stopFlag has already been unblocked by the owner's
            hardwareTeardown().
        why: the owner declares its TeardownGuard after every ConsumerThread member. The
            teardown therefore runs first while these threads are still joinable.
    */
    ~ConsumerThread() {
        requestStop();
        joinIfRunning();
    }
};

/*| role: a thread of a block's own that makes the block's device control calls in the order
        they were asked for, away from the thread that streams.
    contract: start() runs the thread; post(key, work) queues work and returns at once. Work
        queued under a key that has not begun leaves its place, and the new work goes to the
        back, so the last write of a key wins and the writes that remain run in the order given.
        settle() returns once nothing is queued or running, and settleFor() does the same within
        a duration, answering whether it did. stop() drops what has not begun, waits for what is
        running and joins the thread; a post while stopped runs nothing.
    why: a device control call can outlast the buffering a stream has.
    invariant: one piece of work runs at a time, and the lock is not held across it, so the
        work may take the block's own locks and post() never waits for a device.
    trap: settle() and stop() wait for the running piece, so neither may be called while
        holding a lock that piece takes.
    verified-by: controlworker.writes-run-in-order-and-the-last-of-a-key-wins
*/
struct ControlWorker {
    ~ControlWorker() { stop(); }

    void start(std::string_view tag, std::string_view blockName) {
        stop();
        {
            std::lock_guard lock(_mutex);
            _stopping = false;
        }
        _thread = std::thread([this, name = std::string(tag) + ':' + std::string(blockName)] {
            capture::threading::nameOwnThread(name);
            run();
        });
    }

    void post(int key, std::function<void()> work) {
        {
            std::lock_guard lock(_mutex);
            if (_stopping) {
                return;
            }
            std::erase_if(_queue, [key](const auto& queued) { return queued.first == key; });
            _queue.emplace_back(key, std::move(work));
        }
        _wake.notify_all();
    }

    void settle() {
        std::unique_lock lock(_mutex);
        _wake.wait(lock, [this] { return _stopping || (_queue.empty() && !_running); });
    }

    /*| contract: settle() bounded by limit. True where nothing is queued or running at the
            return, or the worker is stopping; false where the limit passed first, the work
            still queued or running.
        verified-by: controlworker.a-settle-ends-by-its-bound
    */
    bool settleFor(std::chrono::duration<double> limit) {
        std::unique_lock lock(_mutex);
        return _wake.wait_for(lock, limit, [this] { return _stopping || (_queue.empty() && !_running); });
    }

    void stop() {
        {
            std::lock_guard lock(_mutex);
            _stopping = true;
            _queue.clear();
        }
        _wake.notify_all();
        if (_thread.joinable()) {
            _thread.join();
        }
    }

private:
    void run() {
        std::unique_lock lock(_mutex);
        for (;;) {
            _wake.wait(lock, [this] { return _stopping || !_queue.empty(); });
            if (_stopping) {
                break;
            }
            std::function<void()> work = std::move(_queue.front().second);
            _queue.pop_front();
            _running = true;
            lock.unlock();
            work();
            lock.lock();
            _running = false;
            _wake.notify_all();
        }
    }

    std::mutex                                        _mutex;
    std::condition_variable                           _wake;
    std::deque<std::pair<int, std::function<void()>>> _queue;
    bool                                              _running  = false;
    bool                                              _stopping = true;
    std::thread                                       _thread;
};

/*| role: the last data member of every device block. Its destructor runs the block's
        hardwareTeardown() while every other member is still alive, then drives an active block
        to STOPPED.
    contract: each block's hardwareTeardown() is its stop() body without the end-of-stream
        publish. It is idempotent and serialized, and it returns true to exactly one caller per
        open device, the one that released it. stop() publishes end-of-stream on true. A run
        therefore ends with one end-of-stream tag. This guard calls it where stop() never ran.
    why: a stop that races a slow start can skip stop(), and the teardown still has to join
        the block's threads, end a library callback into the block and close the device. A
        failure the stream detects and a caller's stop can reach the teardown at once.
    trap: an exception that leaves the destructor ends the process. The guard catches it, and
        the handle the teardown was closing leaks.
    trap: a graph whose scheduler ends in ERROR destroys its blocks while they are RUNNING, and
        the base destructor calls stop() on an active block after the derived members are gone.
        The drive to STOPPED leaves a state the base destructor does not act on.
*/
template <typename TBlock>
struct TeardownGuard {
    TBlock* self;
    ~TeardownGuard() {
        if (self == nullptr) {
            return;
        }
        try {
            self->hardwareTeardown();
        } catch (...) {
        }
        if (gr::lifecycle::isActive(self->state())) {
            std::ignore = self->changeStateTo(gr::lifecycle::State::REQUESTED_STOP);
            std::ignore = self->changeStateTo(gr::lifecycle::State::STOPPED);
        }
    }
};

/*| contract: take a block's control mutex and test device liveness under it. The guard is
        true while the device handle stays valid for its scope: CtrlGuard guard(_ctrlMutex,
        _deviceUp); if (!guard) { return false; }, then the device calls.
    invariant: every teardown clears the liveness flag first, joins the streaming thread, and
        only then takes the control mutex to close the device and null its handle.
    trap: a liveness test made before the lock can pass, wait on the mutex through the join,
        and then write through the handle the teardown closed.
*/
class CtrlGuard {
public:
    CtrlGuard(std::mutex& ctrlMutex, const std::atomic<bool>& deviceUp) : _lock(ctrlMutex), _live(deviceUp.load(std::memory_order_acquire)) {}

    /*| contract: the same guard with a deadline. It gives up after waitFor and is then false,
            holding no lock.
        why: for a read-only path on the thread that serves an interface. The mutex can be held
            across one sensor read that waits up to a second for a GPS sentence.
        verified-by: ctrlguard.a-bounded-wait-gives-up-and-holds-no-lock
    */
    CtrlGuard(std::mutex& ctrlMutex, const std::atomic<bool>& deviceUp, std::chrono::milliseconds waitFor) : _lock(ctrlMutex, std::defer_lock), _live(false) {
        const auto deadline = std::chrono::steady_clock::now() + waitFor;
        for (;;) {
            if (_lock.try_lock()) {
                _live = deviceUp.load(std::memory_order_acquire);
                return;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }

    explicit operator bool() const noexcept { return _live; }

private:
    std::unique_lock<std::mutex> _lock;
    bool                         _live;
};

} // namespace capture
