/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <boost/ut.hpp>

#include <cstdio>
#include <string_view>
#include <utility>

namespace capture::test {

/*| role: the cases one test binary holds, run whole or one at a time.
    contract: every case is registered with ctest under its own name and selected by naming it
        on the command line, so a failing run says which fact stopped holding without a reader
        having to open the binary, and a run with no argument is still the whole file.
    trap: a name that matches nothing is a failure. Boost.UT's own filter reports success for a
        run in which nothing ran, so a ctest entry whose name had drifted from the case it names
        would go green forever. This counts what it admitted, and report() refuses zero.
*/
class Cases {
public:
    Cases(int argc, char** argv) : _only(argc > 1 ? std::string_view{argv[1]} : std::string_view{}) {}

    // Register and run one case, unless the command line named another.
    template <typename Body>
    void operator()(std::string_view name, Body&& body) {
        if (!_only.empty() && _only != name) {
            return;
        }
        ++_ran;
        boost::ut::detail::test{"test", name} = std::forward<Body>(body);
    }

    // What main() returns: 0 when something ran, 2 when the name matched no case.
    [[nodiscard]] int report() const {
        if (_ran == 0) {
            std::fprintf(stderr, "no case named \"%.*s\" in this binary\n", static_cast<int>(_only.size()), _only.data());
            return 2;
        }
        return 0;
    }

private:
    std::string_view _only;
    int              _ran = 0;
};

} // namespace capture::test
