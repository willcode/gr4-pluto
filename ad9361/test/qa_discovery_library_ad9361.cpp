/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
/*| role: the discovery library a top builds over the AD9361 family alone, linked as a program
        links it rather than compiled in, asked for its families. No device is asked.
*/
#include <capture/common/DiscoveryLibrary.hpp>

#include "support/Cases.hpp"

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    using namespace boost::ut;
    capture::test::Cases cases(argc, argv);

    cases("discovery.ad9361-the-linked-library-carries-the-ad9361-family", [] {
        const auto families = capture::discovery::families();
        expect(fatal(families.size() == 1UZ)) << "one family";
        expect(families[0].family == "ad9361") << "named as the AD9361 blocks' device setting names it: " << families[0].family;
        expect(families[0].receive && families[0].transmit) << "with its source and its sink";
    });

    const int  rc     = cases.report();
    const bool failed = boost::ut::cfg<boost::ut::override>.run({.report_errors = true});
    std::fflush(stdout);
    std::fflush(stderr);
    /*| trap: the linked library carries GNU Radio 4, whose global destructors run after its
            threads are gone, so the process ends without static teardown once the verdict is
            taken.
    */
    std::_Exit(failed ? 1 : rc);
}
