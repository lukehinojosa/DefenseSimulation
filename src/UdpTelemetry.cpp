#include "sim/UdpTelemetry.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

#include "sim/TelemetryCodec.hpp"

// ---------------------------------------------------------------------------
// Cross-platform socket layer: POSIX sockets on Unix, Winsock2 on Windows.
// The rest of the file uses the small shims defined here.
// ---------------------------------------------------------------------------
#if defined(_WIN32)
// Suppress the windows.h min/max macros (pulled in via winsock2.h) so they do
// not collide with std::min/std::max, and trim the header surface.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
using socket_t = int;
static constexpr socket_t kInvalidSocket = -1;
#endif

namespace sim {
namespace telemetry {

namespace {

// Encoded datagrams are variable length but bounded by the byte budget; the
// receive buffer adds one record's worth of slack for safety.
constexpr std::size_t kMaxDatagram = kUdpDatagramBudget + codec::kMaxRecordBytes;

#if defined(_WIN32)
// One-time Winsock initialization for the process.
struct WinsockInit {
    WinsockInit() {
        WSADATA data;
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
    }
    ~WinsockInit() { WSACleanup(); }
};
void ensureWinsock() { static WinsockInit init; }

int lastError() { return WSAGetLastError(); }
void closeSocket(socket_t s) { ::closesocket(s); }
#else
void ensureWinsock() {}
int lastError() { return errno; }
void closeSocket(socket_t s) { ::close(s); }
#endif

[[noreturn]] void throwSockError(const char* what) {
    throw std::system_error(lastError(), std::generic_category(), what);
}

socket_t toSock(std::intptr_t h) { return static_cast<socket_t>(h); }

} // namespace

// ---------------------------------------------------------------------------
// Sender
// ---------------------------------------------------------------------------
UdpTelemetrySender::UdpTelemetrySender(const std::string& host,
                                       std::uint16_t port) {
    ensureWinsock();
    socket_t s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s == kInvalidSocket) {
        throwSockError("socket");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        closeSocket(s);
        throw std::system_error(EINVAL, std::generic_category(), "inet_pton");
    }

    // connect() on a UDP socket fixes the default destination for send().
    if (::connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        const int e = lastError();
        closeSocket(s);
        throw std::system_error(e, std::generic_category(), "connect");
    }
    fd_ = static_cast<std::intptr_t>(s);
}

UdpTelemetrySender::~UdpTelemetrySender() {
    if (fd_ != -1) {
        closeSocket(toSock(fd_));
    }
}

long UdpTelemetrySender::send(const FrameHeader& header,
                              const TelemetryRecord* records,
                              std::uint32_t count) {
    // Encode compactly into a fixed stack buffer (no heap allocation). The
    // codec greedily packs the highest-priority records that fit the budget.
    unsigned char buf[kMaxDatagram];
    std::uint32_t encoded = 0;
    const std::size_t bytes = codec::encodeFrame(
        buf, kUdpDatagramBudget, header, records, count, encoded);
    return static_cast<long>(
        ::send(toSock(fd_), reinterpret_cast<const char*>(buf),
               static_cast<int>(bytes), 0));
}

// ---------------------------------------------------------------------------
// Receiver
// ---------------------------------------------------------------------------
UdpTelemetryReceiver::UdpTelemetryReceiver(std::uint16_t port) {
    ensureWinsock();
    socket_t s = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s == kInvalidSocket) {
        throwSockError("socket");
    }

    int one = 1;
    ::setsockopt(s, SOL_SOCKET, SO_REUSEADDR,
                 reinterpret_cast<const char*>(&one), sizeof(one));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);
    if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        const int e = lastError();
        closeSocket(s);
        throw std::system_error(e, std::generic_category(), "bind");
    }

    sockaddr_in bound{};
    socklen_t len = sizeof(bound);
    if (::getsockname(s, reinterpret_cast<sockaddr*>(&bound), &len) == 0) {
        boundPort_ = ntohs(bound.sin_port);
    } else {
        boundPort_ = port;
    }
    fd_ = static_cast<std::intptr_t>(s);
}

UdpTelemetryReceiver::~UdpTelemetryReceiver() {
    if (fd_ != -1) {
        closeSocket(toSock(fd_));
    }
}

bool UdpTelemetryReceiver::receive(FrameHeader& headerOut,
                                   TelemetryRecord* recordsOut,
                                   std::uint32_t maxRecords,
                                   std::uint32_t& outCount,
                                   int timeoutMs) {
    const socket_t s = toSock(fd_);
    int recvFlags = 0;

    // timeoutMs semantics: <0 block indefinitely; 0 poll (return immediately if
    // nothing is queued); >0 block up to that many milliseconds. Note that a
    // SO_RCVTIMEO of zero means "block forever", so the poll case must use a
    // genuinely non-blocking recv instead.
    if (timeoutMs == 0) {
#if defined(_WIN32)
        u_long nb = 1;
        ::ioctlsocket(s, FIONBIO, &nb);
#else
        recvFlags = MSG_DONTWAIT;
#endif
    } else if (timeoutMs > 0) {
#if defined(_WIN32)
        DWORD tv = static_cast<DWORD>(timeoutMs);
#else
        timeval tv{};
        tv.tv_sec  = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
#endif
        ::setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
                     reinterpret_cast<const char*>(&tv), sizeof(tv));
    }

    unsigned char buf[kMaxDatagram];
    const long got = static_cast<long>(
        ::recv(s, reinterpret_cast<char*>(buf), static_cast<int>(sizeof(buf)),
               recvFlags));

#if defined(_WIN32)
    if (timeoutMs == 0) {
        u_long nb = 0;
        ::ioctlsocket(s, FIONBIO, &nb); // restore blocking mode
    }
#endif

    if (got <= 0) {
        return false; // timeout, would-block, or error
    }

    return codec::decodeFrame(buf, static_cast<std::size_t>(got), headerOut,
                              recordsOut, maxRecords, outCount);
}

} // namespace telemetry
} // namespace sim
