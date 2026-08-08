#include "sim/UdpTelemetry.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <system_error>

namespace sim {
namespace telemetry {

namespace {

[[noreturn]] void throwErrno(const char* what) {
    throw std::system_error(errno, std::generic_category(), what);
}

/// Max serialized datagram: header + the record cap.
constexpr std::size_t kMaxDatagram =
    sizeof(FrameHeader) + kUdpMaxRecords * sizeof(TelemetryRecord);

} // namespace

// ---------------------------------------------------------------------------
// Sender
// ---------------------------------------------------------------------------
UdpTelemetrySender::UdpTelemetrySender(const std::string& host,
                                       std::uint16_t port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ == -1) {
        throwErrno("socket");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        const int e = errno;
        ::close(fd_);
        throw std::system_error(e ? e : EINVAL, std::generic_category(),
                                "inet_pton");
    }

    // connect() on a UDP socket fixes the default destination for send().
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == -1) {
        const int e = errno;
        ::close(fd_);
        throw std::system_error(e, std::generic_category(), "connect");
    }
}

UdpTelemetrySender::~UdpTelemetrySender() {
    if (fd_ != -1) {
        ::close(fd_);
    }
}

long UdpTelemetrySender::send(const FrameHeader& header,
                              const TelemetryRecord* records,
                              std::uint32_t count) {
    const std::uint32_t n = std::min(count, kUdpMaxRecords);

    // Fixed stack buffer — no heap allocation on the send path.
    unsigned char buf[kMaxDatagram];
    FrameHeader hdr = header;
    hdr.recordCount = n; // datagram reflects what actually fits
    std::memcpy(buf, &hdr, sizeof(hdr));
    if (n > 0) {
        std::memcpy(buf + sizeof(hdr), records,
                    static_cast<std::size_t>(n) * sizeof(TelemetryRecord));
    }
    const std::size_t bytes = sizeof(hdr) + static_cast<std::size_t>(n) *
                                                sizeof(TelemetryRecord);
    return static_cast<long>(::send(fd_, buf, bytes, 0));
}

// ---------------------------------------------------------------------------
// Receiver
// ---------------------------------------------------------------------------
UdpTelemetryReceiver::UdpTelemetryReceiver(std::uint16_t port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ == -1) {
        throwErrno("socket");
    }

    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == -1) {
        const int e = errno;
        ::close(fd_);
        throw std::system_error(e, std::generic_category(), "bind");
    }

    // Recover the actual port when bound to 0 (ephemeral).
    sockaddr_in bound{};
    socklen_t len = sizeof(bound);
    if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&bound), &len) == 0) {
        boundPort_ = ntohs(bound.sin_port);
    } else {
        boundPort_ = port;
    }
}

UdpTelemetryReceiver::~UdpTelemetryReceiver() {
    if (fd_ != -1) {
        ::close(fd_);
    }
}

bool UdpTelemetryReceiver::receive(FrameHeader& headerOut,
                                   TelemetryRecord* recordsOut,
                                   std::uint32_t maxRecords,
                                   std::uint32_t& outCount,
                                   int timeoutMs) {
    if (timeoutMs >= 0) {
        timeval tv{};
        tv.tv_sec  = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    unsigned char buf[kMaxDatagram];
    const ssize_t got = ::recv(fd_, buf, sizeof(buf), 0);
    if (got < static_cast<ssize_t>(sizeof(FrameHeader))) {
        return false; // timeout, error, or runt datagram
    }

    FrameHeader hdr;
    std::memcpy(&hdr, buf, sizeof(hdr));
    if (hdr.magic != kMagic || hdr.version != kProtocolVersion) {
        return false;
    }

    // Trust the smaller of what the header claims and what actually arrived.
    const std::size_t payload = static_cast<std::size_t>(got) - sizeof(hdr);
    const std::uint32_t arrived =
        static_cast<std::uint32_t>(payload / sizeof(TelemetryRecord));
    const std::uint32_t n =
        std::min({hdr.recordCount, arrived, maxRecords});
    if (n > 0) {
        std::memcpy(recordsOut, buf + sizeof(hdr),
                    static_cast<std::size_t>(n) * sizeof(TelemetryRecord));
    }

    headerOut = hdr;
    outCount  = n;
    return true;
}

} // namespace telemetry
} // namespace sim
