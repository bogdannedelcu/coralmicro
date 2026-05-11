// gz_to_uds_bridge.cc — Garden 7.9 (gz-transport12) → UDS frame forwarder.
//
// Why C++ and not Python: the Garden distrobox (Ubuntu 22.04 + OSRF apt)
// ships gz-transport12 / gz-msgs9 as C++ libraries but does NOT package
// Python bindings for these versions (only Harmonic transport13/msgs10
// have python3-gz-* apt pkgs and Harmonic is banned for SentAI — see
// Sim.md "3 known-issue").  So the subscriber half of the bridge stays
// in C++ inside the distrobox; the cflib half stays in Python on the host.
// Image bytes never touch Python in the realtime path (per the C/C++
// realtime rule).
//
// Subscribes to a gz transport image topic, forwards each frame to the
// SentAI UDS in the SAME protocol as the prior Python bridge:
//
//   request:
//     uint32_t magic = 0x53434D31  ('SCM1')
//     uint32_t seq
//     uint32_t width / height / pix_fmt(=0) / payload_bytes
//     uint8_t  payload[width*height*3]    (RGB888 packed)
//   reply (sentai_sim → us):
//     uint32_t reply_magic = 0x46524C31  ('FRL1')
//     uint32_t seq
//     int32_t  dx_q1000, dy_q1000
//     uint32_t conf
//     uint64_t latency_us
//
// The reply is forwarded to a SECOND Unix socket (default
// /tmp/sentai_flow_out.sock) where the host-side Python (cflib bridge)
// reads it.  This split lets cflib run in its own Python venv on the host
// without needing any gz Python bindings.
//
// Build (inside distrobox):
//   distrobox enter crazysim-garden
//   g++ -O2 -std=c++17 sim/gazebo/gz_to_uds_bridge.cc \
//       $(pkg-config --cflags --libs gz-transport12 gz-msgs9) \
//       -o /tmp/gz_to_uds_bridge
//
// Run:
//   /tmp/gz_to_uds_bridge \
//     --topic /downward_cam/image \
//     --in-sock /tmp/sentai_cam.sock \
//     --out-sock /tmp/sentai_flow_out.sock

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <errno.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

namespace {

constexpr uint32_t MAGIC       = 0x53434D31u;  // 'SCM1'
constexpr uint32_t REPLY_MAGIC = 0x46524C31u;  // 'FRL1'

#pragma pack(push, 1)
struct Header {
    uint32_t magic;
    uint32_t seq;
    uint32_t width;
    uint32_t height;
    uint32_t pix_fmt;
    uint32_t payload_bytes;
};
struct Reply {
    uint32_t reply_magic;
    uint32_t seq;
    int32_t  dx_q1000;          // L0 wide
    int32_t  dy_q1000;
    uint32_t conf;
    uint64_t latency_us;
    int32_t  dz_q1000;          // sub-block divergence
    uint32_t dz_conf;
    int32_t  dx_center_q1000;   // L1 mid (320→80)
    int32_t  dy_center_q1000;
    uint32_t conf_center;
    int32_t  dx_fine_q1000;     // L2 fine (160→80) — 2026-05-11
    int32_t  dy_fine_q1000;
    uint32_t conf_fine;
};
#pragma pack(pop)

int connect_uds(const std::string& path, int retries = 30) {
    for (int i = 0; i < retries; ++i) {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0) return -1;
        struct sockaddr_un addr = {};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
        if (::connect(fd, (sockaddr*)&addr, sizeof(addr)) == 0) {
            // CRITICAL fix (code review HIGH): cap reply read so a hung
            // sentai_sim can't deadlock the gz callback thread (we hold
            // g_in_mu around read_full).  500 ms covers worst-case PXP+FFT
            // pipeline (max measured 3.4 ms) plus generous slack.
            struct timeval tv = { .tv_sec = 0, .tv_usec = 500 * 1000 };
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
            return fd;
        }
        close(fd);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return -1;
}

int listen_uds(const std::string& path) {
    ::unlink(path.c_str());
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un addr = {};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::fprintf(stderr, "bind(%s) errno=%d\n", path.c_str(), errno);
        close(fd); return -1;
    }
    chmod(path.c_str(), 0666);
    if (::listen(fd, 1) < 0) { close(fd); return -1; }
    return fd;
}

bool write_full(int fd, const void* p, size_t n) {
    const uint8_t* b = (const uint8_t*)p;
    size_t put = 0;
    while (put < n) {
        ssize_t w = ::write(fd, b + put, n - put);
        if (w > 0) { put += (size_t)w; continue; }
        if (w < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

bool read_full(int fd, void* p, size_t n) {
    uint8_t* b = (uint8_t*)p;
    size_t got = 0;
    while (got < n) {
        ssize_t r = ::read(fd, b + got, n - got);
        if (r > 0) { got += (size_t)r; continue; }
        if (r < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

// Globals — single-camera, single-sentai_sim, single-flow-consumer.
std::mutex                   g_in_mu;
int                          g_in_fd     = -1;
std::atomic<int>             g_out_client{-1};
std::atomic<uint32_t>        g_seq{0};
std::atomic<uint32_t>        g_frames_in{0};
std::atomic<uint32_t>        g_frames_pub{0};

void on_image(const gz::msgs::Image& msg) {
    const uint32_t seq = ++g_seq;

    // DEBUG every 30 frames: print msg timestamp + payload checksum to
    // distinguish "Gazebo coalescing identical frames" vs "messages
    // changing but pixel data same".
    if ((seq % 30) == 0) {
        uint64_t stamp_ns = (uint64_t)msg.header().stamp().sec() * 1000000000ull
                          + (uint64_t)msg.header().stamp().nsec();
        uint32_t ck = 0;
        const auto& d = msg.data();
        for (size_t i = 0; i < d.size(); i += 19) ck = ck * 31u + (uint8_t)d[i];
        std::fprintf(stderr,
            "[bridge_recv] seq=%u stamp=%llu.%09llu data_crc=0x%08x size=%zu\n",
            seq,
            (unsigned long long)(stamp_ns/1000000000ull),
            (unsigned long long)(stamp_ns%1000000000ull),
            ck, d.size());
    }

    // Build header + payload, send under lock so we don't interleave
    // multiple frames on the UDS.  Then read reply, forward to flow-out
    // client (if present).
    Header hdr;
    hdr.magic         = MAGIC;
    hdr.seq           = seq;
    hdr.width         = msg.width();
    hdr.height        = msg.height();
    hdr.pix_fmt       = 0;  // RGB888
    hdr.payload_bytes = (uint32_t)(msg.width() * msg.height() * 3u);

    if (msg.data().size() != hdr.payload_bytes) {
        std::fprintf(stderr, "frame size mismatch: got %zu expect %u — drop\n",
                     msg.data().size(), hdr.payload_bytes);
        return;
    }

    Reply reply{};
    {
        std::lock_guard<std::mutex> lk(g_in_mu);
        if (g_in_fd < 0) return;
        if (!write_full(g_in_fd, &hdr, sizeof(hdr))) {
            std::fprintf(stderr, "in-sock write hdr failed errno=%d\n", errno);
            close(g_in_fd); g_in_fd = -1; return;
        }
        if (!write_full(g_in_fd, msg.data().data(), msg.data().size())) {
            std::fprintf(stderr, "in-sock write payload failed\n");
            close(g_in_fd); g_in_fd = -1; return;
        }
        if (!read_full(g_in_fd, &reply, sizeof(reply))) {
            std::fprintf(stderr, "in-sock read reply failed\n");
            close(g_in_fd); g_in_fd = -1; return;
        }
    }
    g_frames_in++;

    // CRITICAL fix (code review HIGH): atomic exchange-out before write
    // prevents a use-after-close race where accept_loop replaces the fd
    // while we're writing to it.  Pattern: take ownership (-1), write,
    // put back if still valid.
    int oc = g_out_client.exchange(-1);
    if (oc >= 0) {
        bool ok = write_full(oc, &reply, sizeof(reply));
        if (!ok) {
            std::fprintf(stderr, "out-sock write failed; closing client\n");
            close(oc);
        } else {
            g_frames_pub++;
            // Put back unless accept_loop installed a newer client meanwhile.
            int expected = -1;
            if (!g_out_client.compare_exchange_strong(expected, oc)) {
                close(oc);   // newer client took over; drop ours
            }
        }
    }

    if ((seq % 30u) == 0u || reply.dx_q1000 != 0 || reply.dy_q1000 != 0) {
        std::fprintf(stderr,
                     "[bridge] seq=%u  dx=%+5d dy=%+5d dz=%+5d conf=%u lat=%llu us  "
                     "(in=%u pub=%u)\n",
                     seq, reply.dx_q1000, reply.dy_q1000, reply.dz_q1000,
                     reply.conf, (unsigned long long)reply.latency_us,
                     g_frames_in.load(), g_frames_pub.load());
    }
}

void out_accept_loop(int listen_fd) {
    while (true) {
        int cfd = ::accept(listen_fd, nullptr, nullptr);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            std::fprintf(stderr, "out-accept errno=%d\n", errno);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        std::fprintf(stderr, "[bridge] flow-out client connected\n");
        // If a previous client is still around, drop it.
        int prev = g_out_client.exchange(cfd);
        if (prev >= 0) close(prev);
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string topic    = "/downward_cam/image";
    std::string in_sock  = "/tmp/sentai_cam.sock";
    std::string out_sock = "/tmp/sentai_flow_out.sock";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto eat = [&](const char* k) -> const char* {
            return (a == k && i + 1 < argc) ? argv[++i] : nullptr;
        };
        if (auto v = eat("--topic"))     topic    = v;
        else if (auto v = eat("--in-sock"))  in_sock  = v;
        else if (auto v = eat("--out-sock")) out_sock = v;
    }

    std::fprintf(stderr, "[bridge] topic=%s in=%s out=%s\n",
                 topic.c_str(), in_sock.c_str(), out_sock.c_str());

    g_in_fd = connect_uds(in_sock);
    if (g_in_fd < 0) {
        std::fprintf(stderr, "could not connect to %s — is sentai_sim running?\n",
                     in_sock.c_str());
        return 2;
    }
    std::fprintf(stderr, "[bridge] connected to sentai_sim on %s\n",
                 in_sock.c_str());

    int out_listen = listen_uds(out_sock);
    if (out_listen < 0) return 3;
    std::fprintf(stderr, "[bridge] listening flow-out on %s\n",
                 out_sock.c_str());
    std::thread(out_accept_loop, out_listen).detach();

    gz::transport::Node node;
    if (!node.Subscribe<gz::msgs::Image>(topic, on_image)) {
        std::fprintf(stderr, "subscribe(%s) failed\n", topic.c_str());
        return 4;
    }
    std::fprintf(stderr, "[bridge] subscribed; Ctrl-C to stop\n");

    while (true) std::this_thread::sleep_for(std::chrono::seconds(1));
    return 0;
}
