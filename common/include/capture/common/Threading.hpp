/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/Env.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <string>
#include <string_view>
#include <vector>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

/*| role: where a thread runs, and how hard it pushes.
    contract: a thread on the path somebody waits on, for samples arriving, sound leaving or a
        picture drawn, keeps what the process gives it. A thread doing background work calls
        runInBackground on entry.
    frame: the process narrows itself to the fast cores once, before any other thread starts.
        Every thread begins there, the ones this library does not create included, and a
        background thread widens itself back.
    frame: the processors the process was given are the set sched_getaffinity reports at
        startup, and the narrowing stays inside that set.
    trap: a background thread that does not call runInBackground stays on the fast cores.
    trap: on a machine with one kind of core this sets no mask. CAPTURE_CPU_POLICY=off turns
        it off for a run.
*/

namespace capture::threading {

// The scheduling class a thread asks for.
enum class Scheduler {
    // Ordinary time-sharing, weighted by the nice value.
    Normal,
    /*| role: the idle class, which weighs about 3 against a normal thread's 1024.
        trap: a cliff rather than a slope: inside a fixed set of processors a thread here is
            either unaffected or completely starved, with little in between. Right for work that
            must never compete — an observer that should stop observing under load — and wrong
            for work that should merely go last.
    */
    Idle,
};

/*| role: the longest thread name Linux stores.
    frame: /proc/<pid>/task/<tid>/comm is a sixteen-byte field including the terminator, so
        fifteen characters is the whole budget and every name in this library is built to fit
        it.
    why: spend the budget on what separates one thread from another — a worker index, which
        graph it serves, which device it drives — because a common prefix that crowds the
        distinguishing part out is worse than no prefix.
*/
inline constexpr int kMaxThreadNameLength = 15;

namespace detail {

// True when the policy is switched off for this run. Read once.
inline bool policyOff() {
    static const bool off = [] {
        const char* s = capture::envValue("CAPTURE_CPU_POLICY");
        return s != nullptr && std::strcmp(s, "off") == 0;
    }();
    return off;
}

#if defined(__linux__)

/*| role: the processors a sysfs list names.
    frame: the kernel's CPU list syntax, "0-11" or "0,2,4-7" — ranges and singles, comma
        separated, no spaces.
    trap: the whole line, however long. A fixed buffer cuts a list that does not fit, possibly
        mid-number, and the processor set then ends on an entry the file never named with
        nothing to say it was cut. The kernel writes these as ranges, so no machine in reach
        produces a line past a few dozen characters, and a set of individually named processors
        does.
    verified-by: threading.a-processor-list-longer-than-a-buffer
*/
inline std::vector<int> parseCpuList(const char* path) {
    std::vector<int> out;
    std::FILE*       f = std::fopen(path, "r");
    if (f == nullptr) {
        return out;
    }
    std::string text;
    char        chunk[256];
    while (std::fgets(chunk, sizeof(chunk), f) != nullptr) {
        text += chunk;
        if (text.find('\n') != std::string::npos) {
            break;
        }
    }
    std::fclose(f);
    if (text.empty()) {
        return out;
    }
    const char* p = text.c_str();
    while (*p != '\0') {
        char*     end = nullptr;
        const long a  = std::strtol(p, &end, 10);
        if (end == p) {
            break;
        }
        long b = a;
        if (*end == '-') {
            const char* q = end + 1;
            b             = std::strtol(q, &end, 10);
        }
        for (long i = a; i <= b; ++i) {
            out.push_back(static_cast<int>(i));
        }
        while (*end == ',' || *end == '\n' || *end == ' ') {
            ++end;
        }
        p = end;
    }
    return out;
}

// A processor's top frequency in kHz, or zero where the kernel does not say.
inline long maxFreqOf(int cpu) {
    char path[128];
    std::snprintf(path, sizeof(path),
                  "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", cpu);
    std::FILE* f = std::fopen(path, "r");
    if (f == nullptr) {
        return 0;
    }
    long v = 0;
    if (std::fscanf(f, "%ld", &v) != 1) {
        v = 0;
    }
    std::fclose(f);
    return v;
}

/*| role: everything the policy decided, settled once at startup and only read afterwards.
    invariant: captured before any of this program's own threads exist, so base holds the
        processors the process was given rather than the set it has narrowed itself to since.
*/
struct State {
    std::vector<int> base; // what sched_getaffinity said at startup
    std::vector<int> fast; // the quick cores within it, or all of base
    bool             hybrid = false;
    bool             narrowed = false;
};

inline State& state() {
    static State s;
    return s;
}

inline void fillSet(cpu_set_t& set, const std::vector<int>& cpus) {
    CPU_ZERO(&set);
    for (const int c : cpus) {
        if (c >= 0 && c < CPU_SETSIZE) {
            CPU_SET(c, &set);
        }
    }
}

#endif // __linux__

/*| contract: give the calling thread this name, shortened to what the kernel accepts rather
        than refused.
    trap: pthread_setname_np answers ERANGE for a name past kMaxThreadNameLength and leaves the
        thread carrying the executable's name, so a name one character too long does not read as
        a slightly clipped name in top -H — it reads as another anonymous copy of the process,
        and every such thread looks identical to every other. Keeping a prefix is always better
        than that.
    why: callers still fit the limit on purpose: clipping falls off the right, which is where an
        index or a device tag usually lives.
*/
inline void applyName(std::string_view name) {
#if defined(__linux__)
    char buf[kMaxThreadNameLength + 1];
    const std::size_t n = std::min(name.size(), static_cast<std::size_t>(kMaxThreadNameLength));
    std::memcpy(buf, name.data(), n);
    buf[n] = '\0';
    // Unchecked: a refused name costs attribution and never a result.
    (void)pthread_setname_np(pthread_self(), buf);
#else
    (void)name;
#endif
}

} // namespace detail

// What the policy found and what it did, for anything reporting it.
struct Topology {
    int  available = 0;     // processors the process was given at startup
    int  fast      = 0;     // of those, the quick ones
    bool hybrid    = false; // whether the two differ
    bool narrowed  = false; // whether the process was actually confined
};

inline Topology topology() {
#if defined(__linux__)
    const detail::State& s = detail::state();
    return Topology{static_cast<int>(s.base.size()), static_cast<int>(s.fast.size()), s.hybrid,
                    s.narrowed};
#else
    return Topology{};
#endif
}

/*| contract: take stock of the processors this process was given, confine it to the quick ones
        where they can be told apart, and return a line describing what was done for a caller
        that wants to log it.
    invariant: called once, as early in main as reaches it and before anything that starts a
        thread: everything created afterwards inherits the mask, which is the only way to reach
        a thread this program does not create. A later call returns the first one's line and
        changes nothing, the whole of the state being settled behind a static.
    trap: it never fails in a way worth acting on. A machine whose cores cannot be told apart, a
        kernel that will not say, or a refused mask all leave every thread where it would have
        been.
    trap: one copy of this policy per process. The base set is the affinity the process holds
        when the first call runs. A second copy of this code in the same program reads an
        affinity the first copy has already narrowed, finds every processor in it quick and
        takes the "all of one kind" exit. Its useAllCores() is then a no-op for the life of the
        process and a background thread demoted through it keeps the quick cores.
    verified-by: threading.the-policy-settles-once
*/
inline std::string adoptFastCoresForProcess() {
#if defined(__linux__)
    static const std::string settled = []() -> std::string {
    detail::State& s = detail::state();

    cpu_set_t have;
    CPU_ZERO(&have);
    if (sched_getaffinity(0, sizeof(have), &have) != 0) {
        return "processors: the affinity of this process could not be read; nothing is pinned";
    }
    for (int c = 0; c < CPU_SETSIZE; ++c) {
        if (CPU_ISSET(c, &have)) {
            s.base.push_back(c);
        }
    }
    if (s.base.empty()) {
        return "processors: no affinity to work from; nothing is pinned";
    }
    s.fast = s.base;

    if (detail::policyOff()) {
        return "processors: CAPTURE_CPU_POLICY=off; nothing is pinned";
    }

    /*| why: the quick cores, by the two routes a Linux machine offers. The first is exact where
            it exists: a hybrid x86 kernel names its two kinds outright. The second works
            anywhere the kernel reports a top frequency and is a clustering rather than a name —
            the highest figure present wins, and a machine whose processors all report the same
            figure has one kind.
    */
    std::vector<int> quick = detail::parseCpuList("/sys/devices/cpu_core/cpus");
    if (quick.empty()) {
        long best = 0;
        for (const int c : s.base) {
            best = std::max(best, detail::maxFreqOf(c));
        }
        if (best > 0) {
            for (const int c : s.base) {
                if (detail::maxFreqOf(c) == best) {
                    quick.push_back(c);
                }
            }
        }
    }

    // Only what the process was actually given: a cpuset that already excludes a
    // processor is an instruction from outside this program.
    std::vector<int> narrowed;
    for (const int c : quick) {
        if (std::find(s.base.begin(), s.base.end(), c) != s.base.end()) {
            narrowed.push_back(c);
        }
    }

    char line[256];
    if (narrowed.empty() || narrowed.size() == s.base.size()) {
        // One kind of core, or a set already confined to one kind. Nothing to do,
        // and saying so is worth more than a mask that changes nothing.
        std::snprintf(line, sizeof(line),
                      "processors: %zu available, all of one kind; nothing is pinned",
                      s.base.size());
        return line;
    }

    s.fast   = narrowed;
    s.hybrid = true;

    cpu_set_t set;
    detail::fillSet(set, s.fast);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
        s.fast = s.base;
        s.hybrid = false;
        return "processors: the mask was refused; nothing is pinned";
    }
    s.narrowed = true;
    std::snprintf(line, sizeof(line),
                  "processors: %zu available, %zu quick; this process and every thread it "
                  "starts run on the quick ones unless they ask otherwise",
                  s.base.size(), s.fast.size());
    return line;
    }();
    return settled;
#else
    return "processors: no policy on this system";
#endif
}

/*| contract: put the calling thread back on every processor the process was given. A no-op
        where nothing was narrowed.
    why: for background work: it undoes the narrowing for this thread alone. A demoted thread
        can then use the slow cores the sample path leaves to it.
*/
inline void useAllCores() {
#if defined(__linux__)
    const detail::State& s = detail::state();
    if (!s.narrowed || s.base.empty()) {
        return;
    }
    cpu_set_t set;
    detail::fillSet(set, s.base);
    // Unchecked: a refused mask costs this thread the slow cores, which is a
    // share of processor time and never a result.
    (void)pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
#endif
}

/*| role: name the calling thread and change nothing else about it, for a thread that keeps
        what it inherited.
    trap: lowerOwnPriority(name, 0) asks for nice zero, which promotes a thread that inherited
        nineteen.
*/
inline void nameOwnThread(std::string_view name) { detail::applyName(name); }

/*| contract: base with index appended, clipped so that the index survives.
    why: for the threads of a pool, where the interesting part of the name is which worker this
        is. Clipping falls off the right, so appending an index to a base already at the limit
        would throw the index away and leave every worker in the pool reading the same — the one
        thing an indexed name exists to prevent. The base gives way instead.
*/
inline std::string indexedName(std::string_view base, int index) {
    std::string suffix = std::to_string(index);
    if (suffix.size() >= static_cast<std::size_t>(kMaxThreadNameLength)) {
        return suffix.substr(0, kMaxThreadNameLength);
    }
    const std::size_t room = static_cast<std::size_t>(kMaxThreadNameLength) - suffix.size();
    std::string       out(base.substr(0, std::min(base.size(), room)));
    out += suffix;
    return out;
}

/*| role: the one place a thread says it is background work: name, how hard it pushes, and what
        it pushes against.
    frame: niceValue is the nice(2) value, 19 being as far back as it goes; policy chooses
        between the ordinary weighted class and the idle one, which is a different kind of thing
        rather than a degree of the same one.
    why: the name makes the cost attributable: a process running this library has dozens
        of threads carrying the executable's name, and "which of these is it" is the first
        question any measurement has to answer.
    trap: nothing here is checked, because a failure costs attribution or a share of processor
        time and never a result.
*/
inline void lowerOwnPriority(std::string_view name, int niceValue = 19,
                             Scheduler policy = Scheduler::Normal) {
#if defined(__linux__)
    detail::applyName(name);

    if (policy == Scheduler::Idle) {
        sched_param param{};
        param.sched_priority = 0; // the idle class has one priority
        if (pthread_setschedparam(pthread_self(), SCHED_IDLE, &param) == 0) {
            return;
        }
        // The weighted class is the fallback. The nice value below says the same thing
        // less absolutely.
    }
    /*| trap: PRIO_PROCESS against a thread id demotes that one thread: Linux carries a nice
            value per thread, and handing this the process id instead would slow down exactly
            the work this is staying out of the way of.
    */
    (void)setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), niceValue);
#else
    (void)name;
    (void)niceValue;
    (void)policy;
#endif
}

/*| role: what a background thread does on entry: name itself, go to the back of the queue, and
        get off the quick cores.
    why: the two calls together, because they are one decision and splitting them is how a
        thread ends up demoted but still occupying a processor the sample path wanted.
*/
inline void runInBackground(std::string_view name, int niceValue = 19,
                            Scheduler policy = Scheduler::Normal) {
    lowerOwnPriority(name, niceValue, policy);
    useAllCores();
}

/*| role: what a caller asked of one thread's placement and what the system granted.
    contract: an asked field that was not applied carries its reason in refused, one clause per
        refusal. A thread asked nothing keeps the scheduling and the processors it started with.
*/
struct Placement {
    bool        priorityAsked   = false;
    bool        priorityApplied = false;
    bool        cpuAsked        = false;
    bool        cpuApplied      = false;
    std::string refused;

    bool asked() const { return priorityAsked || cpuAsked; }

    // One line stating the placement asked and the outcome, empty where nothing was asked.
    std::string describe(double priority, double cpu) const {
        std::string out;
        if (priorityAsked) {
            out += std::format("real-time priority {:g} {}", priority, priorityApplied ? "applied" : "not applied");
        }
        if (cpuAsked) {
            out += std::format("{}processor {:g} {}", out.empty() ? "" : ", ", cpu, cpuApplied ? "applied" : "not applied");
        }
        if (!refused.empty()) {
            out += ": " + refused;
        }
        return out;
    }
};

/*| contract: place the calling thread as a caller asked. A priority above zero and at most one
        asks for the real-time round-robin class at that fraction of the class's priority range,
        min + priority x (max - min), rounded down. A cpu of zero or above pins the thread to
        that one processor. A priority of zero or below, a cpu below zero, and either one not a
        number ask for nothing. A request the system refuses, or one outside those ranges,
        leaves that part of the thread as it was and is named in the answer.
    trap: the real-time class needs a privilege an ordinary user lacks, CAP_SYS_NICE or an
        RLIMIT_RTPRIO limit above zero, and the system refuses it without one. A processor
        outside the set the process was given is refused as well.
    verified-by: threading.an-unasked-placement-leaves-the-thread
    verified-by: threading.an-asked-placement-is-applied-or-named
*/
inline Placement placeOwnThread(double priority, double cpu) {
    Placement out;
    out.priorityAsked = priority > 0.0;
    out.cpuAsked      = cpu >= 0.0;
    const auto refuse = [&out](const std::string& why) { out.refused += (out.refused.empty() ? "" : "; ") + why; };
#if defined(__linux__)
    if (out.priorityAsked) {
        if (priority > 1.0) {
            refuse("a priority takes 0 to 1");
        } else {
            const int   lo = sched_get_priority_min(SCHED_RR);
            const int   hi = sched_get_priority_max(SCHED_RR);
            sched_param param{};
            param.sched_priority = lo + static_cast<int>(priority * static_cast<double>(hi - lo));
            if (const int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &param); rc == 0) {
                out.priorityApplied = true;
            } else {
                refuse(std::string("the real-time class: ") + std::strerror(rc));
            }
        }
    }
    if (out.cpuAsked) {
        if (cpu >= static_cast<double>(CPU_SETSIZE) || cpu != static_cast<double>(static_cast<long long>(cpu))) {
            refuse("a processor takes a whole number below " + std::to_string(CPU_SETSIZE));
        } else {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(static_cast<int>(cpu), &set);
            if (const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set); rc == 0) {
                out.cpuApplied = true;
            } else {
                refuse("processor " + std::to_string(static_cast<long long>(cpu)) + ": " + std::strerror(rc));
            }
        }
    }
#else
    if (out.asked()) {
        refuse("thread placement is not available on this system");
    }
#endif
    return out;
}

} // namespace capture::threading
