/* -*- c++ -*- */
/*
 * Copyright 2026 Jeff Long
 * SPDX-License-Identifier: MIT
 */
#pragma once

#if defined(CAPTURE_AD9361_LIBIIO1)
#include <iio/iio.h>
#else
#include <iio.h>
#endif

#include <cerrno>
#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <sys/types.h>
#include <tuple>
#include <vector>

/*| role: the libiio calls of the AD9361 family whose form differs between libiio 0.x and 1.0:
        opening a context, scanning, attributes, and streaming. Every other libiio call the
        family makes (devices, channels, registers, the timeout, error text) has one form in
        both.
    contract: CAPTURE_AD9361_LIBIIO1 defined selects libiio 1.0 and <iio/iio.h>; otherwise
        libiio 0.x and <iio.h>. Failures are negative errno codes in both.
    trap: the build defines CAPTURE_AD9361_LIBIIO1 on capture-interface where it chose 1.0, so
        a program that compiles these headers through the package's targets follows it. A
        program that compiles them otherwise defines it itself where it links libiio 1.0.
*/
namespace capture::ad9361::iio {

#if defined(CAPTURE_AD9361_LIBIIO1)
inline constexpr bool kLibiio1 = true;
#else
inline constexpr bool kLibiio1 = false;
#endif

/*| contract: the context a URI names, or the default one for an empty URI: the one
        IIO_CONTEXT_URI names, or the local devices. nullptr and code set on failure.
*/
inline iio_context* createContext(const std::string& uri, int& code) {
#if defined(CAPTURE_AD9361_LIBIIO1)
    iio_context* made = iio_create_context(nullptr, uri.empty() ? nullptr : uri.c_str());
    if (const int err = iio_err(made); err != 0) {
        code = err;
        return nullptr;
    }
    return made;
#else
    errno             = 0;
    iio_context* made = uri.empty() ? iio_create_default_context() : iio_create_context_from_uri(uri.c_str());
    if (made == nullptr) {
        code = -(errno != 0 ? errno : EIO);
    }
    return made;
#endif
}

// The URIs a scan of the named backends lists, every backend for nullptr.
inline std::vector<std::string> scanUris(const char* backends) {
    std::vector<std::string> out;
#if defined(CAPTURE_AD9361_LIBIIO1)
    struct iio_scan* scan = iio_scan(nullptr, backends);
    if (iio_err(scan) != 0) {
        return out;
    }
    for (std::size_t i = 0; i < iio_scan_get_results_count(scan); ++i) {
        out.emplace_back(iio_scan_get_uri(scan, i));
    }
    iio_scan_destroy(scan);
#else
    iio_scan_context* scan = iio_create_scan_context(backends, 0);
    if (scan == nullptr) {
        return out;
    }
    iio_context_info** info = nullptr;
    const ssize_t      n    = iio_scan_context_get_info_list(scan, &info);
    for (ssize_t i = 0; i < n; ++i) {
        out.emplace_back(iio_context_info_get_uri(info[i]));
    }
    if (n > 0) {
        iio_context_info_list_free(info);
    }
    iio_scan_context_destroy(scan);
#endif
    return out;
}

// A context attribute's value, or nullptr where the context states none.
inline const char* contextAttr(const iio_context* ctx, const char* name) {
#if defined(CAPTURE_AD9361_LIBIIO1)
    const iio_attr* attr = iio_context_find_attr(ctx, name);
    return attr != nullptr && iio_err(attr) == 0 ? iio_attr_get_static_value(attr) : nullptr;
#else
    return iio_context_get_attr_value(ctx, name);
#endif
}

inline std::optional<std::string> readAttr(const iio_channel* chn, const char* name) {
    if (chn == nullptr) {
        return std::nullopt;
    }
    char buf[512] = {};
#if defined(CAPTURE_AD9361_LIBIIO1)
    const iio_attr* attr = iio_channel_find_attr(chn, name);
    if (attr == nullptr || iio_err(attr) != 0 || iio_attr_read_raw(attr, buf, sizeof buf - 1) < 0) {
        return std::nullopt;
    }
#else
    if (iio_channel_attr_read(chn, name, buf, sizeof buf) < 0) {
        return std::nullopt;
    }
#endif
    return std::string(buf);
}

inline bool writeAttr(const iio_channel* chn, const char* name, const std::string& value) {
    if (chn == nullptr) {
        return false;
    }
#if defined(CAPTURE_AD9361_LIBIIO1)
    const iio_attr* attr = iio_channel_find_attr(chn, name);
    return attr != nullptr && iio_err(attr) == 0 && iio_attr_write_string(attr, value.c_str()) >= 0;
#else
    return iio_channel_attr_write(chn, name, value.c_str()) >= 0;
#endif
}

inline bool writeAttr(const iio_device* dev, const char* name, const std::string& value) {
    if (dev == nullptr) {
        return false;
    }
#if defined(CAPTURE_AD9361_LIBIIO1)
    const iio_attr* attr = iio_device_find_attr(dev, name);
    return attr != nullptr && iio_err(attr) == 0 && iio_attr_write_string(attr, value.c_str()) >= 0;
#else
    return iio_device_attr_write(const_cast<iio_device*>(dev), name, value.c_str()) >= 0;
#endif
}

/*| role: one data device's stream: the channels it carries and the transfers queued to the
        kernel, kBlocks of them.
    contract: open() enables the channels given and no other scan channel of the device.
        Receive: refill() waits for the next filled transfer and answers its bytes; data() is
        that transfer until the next refill. Transmit: data() is the transfer to fill; push()
        queues it and makes the next one data(), waiting while every transfer is queued.
        cancel() ends a wait in another thread. Destruction drops the transfers still queued,
        frees them, and disables the channels it enabled.
    frame: libiio 0.x keeps the transfers inside one buffer whose memory moves with each
        push or refill. libiio 1.0 hands out blocks: each is enqueued and dequeued in turn,
        and the buffer starts after the first enqueue, as libiio's own stream helper does; the
        helper is not used because it queues a transmit block only when the next is asked
        for, and a burst's last would wait for a block nobody asks for.
    trap: libiio 1.0 returns from an enqueue before the block reaches the radio, so the first
        transmit push waits for its block, as 0.x's push does; without the wait the DAC's
        underflow flag read after the second and third push of a burst was still the empty
        DAC's.
    measured: a Pluto, firmware v0.39, over USB, 2.5 MS/s, 64k-sample transfers, libiio 1.0.0
        against iiod 0.26: without the wait the second and third push returned at once with the
        flag set; from the fifth on each push waited about 16 ms for a free block.
    invariant: refill and push run on one thread; cancel may run on another.
*/
class Stream {
public:
    static constexpr std::size_t kBlocks = 4UZ; // libiio 0.x's default kernel queue

    static std::unique_ptr<Stream> open(iio_device* dev, std::span<iio_channel* const> channels, std::size_t samples, bool output, int& code) {
        std::unique_ptr<Stream> s(new Stream());
#if defined(CAPTURE_AD9361_LIBIIO1)
        s->_mask = iio_create_channels_mask(iio_device_get_channels_count(dev));
        if (s->_mask == nullptr) {
            code = -ENOMEM;
            return nullptr;
        }
        for (iio_channel* chn : channels) {
            iio_channel_enable(chn, s->_mask);
        }
        iio_buffer* buf = iio_device_get_buffer(dev, 0U);
        if (const int err = iio_err(buf); err != 0 || buf == nullptr) {
            code = err != 0 ? err : -ENODEV;
            return nullptr;
        }
        const ssize_t sampleSize = iio_device_get_sample_size(dev, s->_mask);
        if (sampleSize <= 0) {
            code = sampleSize < 0 ? static_cast<int>(sampleSize) : -EINVAL;
            return nullptr;
        }
        s->_stream = iio_buffer_open(buf, s->_mask);
        if (const int err = iio_err(s->_stream); err != 0) {
            s->_stream = nullptr;
            code       = err;
            return nullptr;
        }
        for (std::size_t i = 0; i < kBlocks; ++i) {
            iio_block* block = iio_buffer_stream_create_block(s->_stream, samples * static_cast<std::size_t>(sampleSize));
            if (const int err = iio_err(block); err != 0) {
                code = err;
                return nullptr;
            }
            s->_blocks.push_back(block);
            s->_queued.push_back(false);
        }
        if (!output) {
            for (std::size_t i = 0; i < kBlocks; ++i) {
                if (const int err = s->enqueue(i); err < 0) {
                    code = err;
                    return nullptr;
                }
            }
        }
#else
        std::ignore = output;
        for (unsigned i = 0; i < iio_device_get_channels_count(dev); ++i) {
            if (iio_channel* chn = iio_device_get_channel(dev, i); chn != nullptr && iio_channel_is_scan_element(chn)) {
                iio_channel_disable(chn);
            }
        }
        for (iio_channel* chn : channels) {
            iio_channel_enable(chn);
        }
        s->_enabled.assign(channels.begin(), channels.end());
        errno   = 0;
        s->_buf = iio_device_create_buffer(dev, samples, false);
        if (s->_buf == nullptr) {
            code = -(errno != 0 ? errno : EIO);
            return nullptr;
        }
#endif
        return s;
    }

    Stream(const Stream&)            = delete;
    Stream& operator=(const Stream&) = delete;

    ~Stream() {
#if defined(CAPTURE_AD9361_LIBIIO1)
        if (_stream != nullptr) {
            iio_buffer_stream_cancel(_stream); // drops what is queued, as 0.x's buffer destruction does
        }
        for (std::size_t i = 0; i < _blocks.size(); ++i) {
            if (_queued[i]) {
                std::ignore = iio_block_dequeue(_blocks[i], false);
            }
            iio_block_destroy(_blocks[i]);
        }
        if (_started) {
            iio_buffer_stream_stop(_stream);
        }
        if (_stream != nullptr) {
            iio_buffer_close(_stream);
        }
        if (_mask != nullptr) {
            iio_channels_mask_destroy(_mask);
        }
#else
        if (_buf != nullptr) {
            iio_buffer_destroy(_buf);
        }
        for (iio_channel* chn : _enabled) {
            iio_channel_disable(chn);
        }
#endif
    }

    // Receive: the next filled transfer's bytes, or a negative code.
    ssize_t refill() {
#if defined(CAPTURE_AD9361_LIBIIO1)
        if (_holding) {
            if (const int err = enqueue(_cur); err < 0) {
                return err;
            }
            _cur = (_cur + 1UZ) % _blocks.size();
        }
        _holding = false;
        if (const int err = dequeue(_cur); err < 0) {
            return err;
        }
        _holding = true;
        return static_cast<ssize_t>(data().size());
#else
        return iio_buffer_refill(_buf);
#endif
    }

    // Transmit: queue data() and make the next transfer data(); its bytes, or a negative code.
    ssize_t push() {
#if defined(CAPTURE_AD9361_LIBIIO1)
        const auto bytes = static_cast<ssize_t>(data().size());
        const bool first = !_started;
        if (const int err = enqueue(_cur); err < 0) {
            return err;
        }
        if (first) {
            if (const int err = dequeue(_cur); err < 0) {
                return err;
            }
        }
        _cur = (_cur + 1UZ) % _blocks.size();
        if (_queued[_cur]) {
            if (const int err = dequeue(_cur); err < 0) {
                return err;
            }
        }
        return bytes;
#else
        return iio_buffer_push(_buf);
#endif
    }

    std::span<std::byte> data() const {
#if defined(CAPTURE_AD9361_LIBIIO1)
        auto* start = static_cast<std::byte*>(iio_block_start(_blocks[_cur]));
        auto* end   = static_cast<std::byte*>(iio_block_end(_blocks[_cur]));
#else
        auto* start = static_cast<std::byte*>(iio_buffer_start(_buf));
        auto* end   = static_cast<std::byte*>(iio_buffer_end(_buf));
#endif
        return end > start ? std::span<std::byte>(start, static_cast<std::size_t>(end - start)) : std::span<std::byte>{};
    }

    void cancel() {
#if defined(CAPTURE_AD9361_LIBIIO1)
        iio_buffer_stream_cancel(_stream);
#else
        iio_buffer_cancel(_buf);
#endif
    }

private:
    Stream() = default;

#if defined(CAPTURE_AD9361_LIBIIO1)
    int enqueue(std::size_t i) {
        const int err = iio_block_enqueue(_blocks[i], 0UZ, false);
        if (err < 0) {
            return err;
        }
        _queued[i] = true;
        if (!_started) {
            if (const int started = iio_buffer_stream_start(_stream); started < 0) {
                return started;
            }
            _started = true;
        }
        return 0;
    }

    int dequeue(std::size_t i) {
        const int err = iio_block_dequeue(_blocks[i], false);
        if (err < 0) {
            return err;
        }
        _queued[i] = false;
        return 0;
    }

    iio_channels_mask*       _mask    = nullptr;
    iio_buffer_stream*       _stream  = nullptr;
    std::vector<iio_block*>  _blocks;
    std::vector<bool>        _queued;
    std::size_t              _cur     = 0UZ;
    bool                     _holding = false; // receive: _blocks[_cur] is dequeued and read
    bool                     _started = false;
#else
    iio_buffer*               _buf = nullptr;
    std::vector<iio_channel*> _enabled;
#endif
};

} // namespace capture::ad9361::iio
