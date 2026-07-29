/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <linux/mctp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <span>
#include <system_error>

#ifndef MCTP_DEST_EID
#define MCTP_DEST_EID 82
#endif

namespace power_control
{

inline constexpr std::chrono::milliseconds mctpResponseTimeout{1000};
inline constexpr std::size_t mctpMaxResponseLen = 256;

using MctpResult = std::expected<std::span<const uint8_t>, std::error_code>;

// Send an MCTP message and hand the response to handler on the event loop.
// msgType goes in smctp_type (e.g. 0x7f for IANA VDM); packet is the message
// body without the type byte. Never blocks. The span is only valid inside the
// callback. On timeout the socket is closed, so handler sees operation_aborted.
inline void mctpSendAsync(boost::asio::io_context& ioContext, uint8_t msgType,
                          std::span<const uint8_t> packet,
                          std::function<void(MctpResult)> handler)
{
    auto fail = [&handler](int err) {
        handler(std::unexpected(std::error_code(err, std::generic_category())));
    };

    int fd = socket(AF_MCTP, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        fail(errno);
        return;
    }

    struct sockaddr_mctp addr{};
    addr.smctp_family = AF_MCTP;
    addr.smctp_network = MCTP_NET_ANY;
    addr.smctp_addr.s_addr = MCTP_DEST_EID;
    addr.smctp_type = msgType;
    addr.smctp_tag = MCTP_TAG_OWNER;

    if (sendto(fd, packet.data(), packet.size(), 0,
               reinterpret_cast<const struct sockaddr*>(&addr),
               sizeof(addr)) < 0)
    {
        int err = errno;
        close(fd);
        fail(err);
        return;
    }

    auto sock =
        std::make_shared<boost::asio::posix::stream_descriptor>(ioContext, fd);
    auto buf = std::make_shared<std::array<uint8_t, mctpMaxResponseLen>>();
    auto timer =
        std::make_shared<boost::asio::steady_timer>(ioContext,
                                                    mctpResponseTimeout);

    // Closing the socket completes the pending wait below with an error, so a
    // missing response cannot leak the fd.
    timer->async_wait([sock](const boost::system::error_code& ec) {
        if (!ec)
        {
            boost::system::error_code ignored;
            sock->close(ignored);
        }
    });

    sock->async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [sock, buf, timer,
         handler = std::move(handler)](const boost::system::error_code& ec) {
            timer->cancel();
            if (ec)
            {
                handler(std::unexpected(
                    std::error_code(ec.value(), std::generic_category())));
                return;
            }
            ssize_t received =
                recv(sock->native_handle(), buf->data(), buf->size(), 0);
            if (received < 0)
            {
                handler(std::unexpected(
                    std::error_code(errno, std::generic_category())));
                return;
            }
            handler(std::span<const uint8_t>(buf->data(), received));
        });
}

} // namespace power_control
