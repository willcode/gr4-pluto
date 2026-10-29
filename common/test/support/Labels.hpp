/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <boost/ut.hpp>

#ifdef CAPTURE_BLOCK_LABELS
#include <gnuradio-4.0/BlockAttributes.hpp>
#endif

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace capture::test {

/*| contract: expectLabels<TBlock>(labels) holds the labels TBlock declares, read through
        gr::block::attributesOf, to the given class/word texts in class order, and its version
        to 1. It holds the family word to the block's kDevice with a meaning to print. It holds
        the declared role to the role the stream ports read. A build without
        CAPTURE_BLOCK_LABELS holds TBlock to declaring no attributes member at all.
*/
template <typename TBlock>
void expectLabels([[maybe_unused]] std::initializer_list<std::string_view> expected) {
    using namespace boost::ut;
#ifdef CAPTURE_BLOCK_LABELS
    const gr::block::Attributes read = gr::block::attributesOf<TBlock>();
    std::vector<std::string>    texts;
    for (const gr::block::Label& label : read.labels) {
        texts.push_back(label.text());
    }
    expect(read.version == 1U) << "the first revision of the block";
    expect(texts == std::vector<std::string>(expected.begin(), expected.end())) << "the labels, in class order";
    expect(!read.labels.empty() && read.labels.front().cls == gr::block::LabelClass::Family) << "the family first";
    if (!read.labels.empty()) {
        expect(read.labels.front().word == std::string_view(TBlock::kDevice)) << "the family word is the device word";
        expect(!read.labels.front().meaning.empty()) << "a family word carries its meaning";
    }
    const auto role   = gr::block::roleOf<TBlock>();
    const auto ported = gr::block::portRoleOf<TBlock>();
    expect(role.has_value() && ported.has_value()) << "a declared role and a role the ports read";
    if (role.has_value() && ported.has_value()) {
        expect(*role == *ported) << "the declared role matches the stream ports";
    }
#else
    expect(!requires { TBlock::attributes; }) << "a core without the label form sees no declaration";
#endif
}

} // namespace capture::test
