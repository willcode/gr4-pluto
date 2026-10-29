/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <capture/common/ControlDesc.hpp>
#include <capture/common/Source.hpp>

#include <gnuradio-4.0/Tag.hpp>

#include <boost/ut.hpp>

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace capture::test {

/*| contract: the gap keys of the tags a reader took: gr:rx_overflow, rx_overflow_cause and
        gr:n_dropped_samples, each as the last tag holding it gives it. tags counts the tags
        taken, and markers those that hold gr:rx_overflow.
*/
struct Marker {
    bool                      overflow = false;
    std::string               cause;
    std::optional<gr::Size_t> count;
    std::size_t               tags    = 0UZ;
    std::size_t               markers = 0UZ;
};

/*| contract: take every tag waiting at the input port and answer its gap keys.
    trap: a read while an earlier span is still open gets that span's tags again. The span
        here closes when this returns.
*/
template <typename Reader>
Marker takeMarkers(Reader& reader) {
    auto   tags = reader.tagReader().get();
    Marker marker;
    marker.tags = tags.size();
    for (const gr::Tag& tag : tags) {
        for (const auto& [key, value] : tag.map) {
            const std::string_view k(key.data(), key.size());
            if (k == gr::tag::RX_OVERFLOW.key()) {
                marker.overflow = value.template get_if<bool>() != nullptr && *value.template get_if<bool>();
                ++marker.markers;
            } else if (k == "rx_overflow_cause") {
                marker.cause = std::string(value.value_or(std::string_view{}));
            } else if (k == gr::tag::N_DROPPED_SAMPLES.key() && value.template get_if<gr::Size_t>() != nullptr) {
                marker.count = *value.template get_if<gr::Size_t>();
            }
        }
    }
    std::ignore = tags.consume(tags.size());
    return marker;
}

// The count a marker carries for a loss of samples.
inline std::optional<gr::Size_t> countOf(std::uint64_t samples) { return static_cast<gr::Size_t>(samples); }

/*| contract: publish a short block through holdAndPublish, as a drain pass does, with the
        timing tag off, and answer the gap keys the reader took.
*/
template <typename Block, typename Reader>
Marker publishAndTake(Block& blk, Reader& reader) {
    blk.emit_timing_tags = false;
    const std::vector<std::complex<float>> block(4UZ);
    int                                    turns = 0;
    std::ignore                                  = holdAndPublish(blk, std::span<const std::complex<float>>(block), 0UL, [&turns] { return ++turns < 100; });
    return takeMarkers(reader);
}

/*| contract: a loss, a publish that places nothing, a second loss and then a publish leave
        one marker on the first sample placed, carrying both losses. markLoss raises the loss
        and answers its count.
*/
template <typename Block, typename Reader, typename MarkLoss>
void expectAnUnplacedMarkerJoinsTheNext(Block& blk, Reader& reader, MarkLoss&& markLoss, std::string_view who) {
    using namespace boost::ut;
    blk.emit_timing_tags                   = false;
    const std::uint64_t                    first = markLoss();
    const std::vector<std::complex<float>> block(4UZ);
    const std::size_t                      none = holdAndPublish(blk, std::span<const std::complex<float>>(block), 0UL, [] { return false; });
    expect(none == 0UZ && takeMarkers(reader).tags == 0UZ) << who << ": a publish that places nothing places no marker";
    const std::uint64_t second = markLoss();
    const Marker        marker = publishAndTake(blk, reader);
    expect(marker.tags == 1UZ && marker.markers == 1UZ) << who << ": the next publish places one marker";
    expect(marker.count == countOf(first + second)) << who << ": carrying both losses";
}

/*| contract: a reading set states rx_dropped_samples and, after it, rx_dropped_markers, with
        the values given. The first is a glance row labeled "Dropped samples", the second a
        detail row labeled "Lost markers", and both read good.
*/
inline void expectLossReadings(const std::vector<SensorReading>& readings, std::uint64_t samples, std::size_t markers, std::string_view who) {
    using namespace boost::ut;
    const auto at = std::ranges::find_if(readings, [](const SensorReading& r) { return r.id == kDroppedSamplesReading; });
    expect(fatal(at != readings.end())) << who << ": states rx_dropped_samples";
    const auto after = std::find_if(std::next(at), readings.end(), [](const SensorReading& r) { return r.id == kDroppedMarkersReading; });
    expect(fatal(after != readings.end())) << who << ": and rx_dropped_markers after it";
    const SensorReading& dropped = *at;
    const SensorReading& lost    = *after;
    expect(dropped.label == "Dropped samples" && dropped.value == std::format("{}", samples) && dropped.good && dropped.brief) << who << ": the dropped samples on a glance row, good, reading " << dropped.value;
    expect(lost.label == "Lost markers" && lost.value == std::format("{}", markers) && lost.good && !lost.brief) << who << ": the lost markers on a detail row, good, reading " << lost.value;
}

} // namespace capture::test
