#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace fpvd::dynlink {

/* UDP listener for PixelPilot IDR-token bursts.
 *
 * PixelPilot_rk sends short UDP datagrams (typically 6 bytes, a random
 * 3-byte ASCII token + newline) to the drone on a configured port when it
 * detects an RTP sequence gap or a decode stall. Anything arriving on this
 * socket is treated as an IDR request; the caller is responsible for
 * throttling the encoder API.
 *
 * port == 0 disables the listener (fd() returns -1, drain() returns 0).
 */
class IdrListener {
public:
    IdrListener(const std::string& bindAddr, uint16_t port);  // port==0 disables
    ~IdrListener();
    IdrListener(const IdrListener&) = delete;
    IdrListener& operator=(const IdrListener&) = delete;

    int    fd()    const { return fd_; }  // -1 if disabled
    size_t drain();                       // recvfrom until EAGAIN; returns count

private:
    int fd_{-1};
};

} // namespace fpvd::dynlink
