/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

/*| role: one reading of what an environment switch means, so that every switch this library
        reads answers the same way.
    why: a switch set to zero is off, as an unset one is. A presence check would read 0 as on.
*/

namespace capture {

/*| contract: whether the switch named by name is on. Off when unset, when empty, and when set
        to 0, no, off or false in any mixture of cases; on for anything else, 1 being the usual
        way to say so.
*/
inline bool envFlag(const char* name) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') {
        return false;
    }
    for (const char* off : {"0", "no", "off", "false"}) {
        if (std::strlen(v) == std::strlen(off)) {
            bool same = true;
            for (std::size_t i = 0; off[i] != '\0'; ++i) {
                const char a = v[i] >= 'A' && v[i] <= 'Z' ? static_cast<char>(v[i] - 'A' + 'a')
                                                          : v[i];
                if (a != off[i]) {
                    same = false;
                    break;
                }
            }
            if (same) {
                return false;
            }
        }
    }
    return true;
}

// The value of name, or nullptr where it is unset or empty. A variable set to nothing is
// a variable that was not set, which is the same rule the flag above follows.
inline const char* envValue(const char* name) {
    const char* v = std::getenv(name);
    return (v != nullptr && *v != '\0') ? v : nullptr;
}

} // namespace capture

