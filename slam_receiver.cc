/**
 * slam_receiver.cc  —  Monocular-Inertial ORB-SLAM3
 *
 * Listens:  9997 (JPEG frames from pc_receiver.py)
 *           9998 (IMU binary packets from pc_receiver.py)
 * Sends:    9999 (pose text   -> pc_receiver.py)
 *           9996 (map points binary -> pc_receiver.py, chunked)
 *
 * Build:
 *   cd ~/ORB_SLAM3/Examples/Monocular-Inertial
 *   mkdir -p build && cd build && cmake .. && make -j4
 *
 * Run:
 *   cd ~/ORB_SLAM3
 *   export LD_LIBRARY_PATH=~/ORB_SLAM3/lib:/usr/local/lib:$LD_LIBRARY_PATH
 *   ./Examples/Monocular-Inertial/build/slam_receiver \
 *       Vocabulary/ORBvoc.txt \
 *       Examples/Monocular-Inertial/webcam2.yaml
 *
 * FIXES in this version:
 *   [1] Viewer flag corrected — false = OFF (headless). Was documented wrong.
 *   [2] Map point UDP chunking — 36KB in one UDP send was silently dropped.
 *       Now sends 200 points (2400 bytes) per packet with a 1-byte chunk header.
 *   [3] FPS pacing decoupled from recvfrom — next_frame now resets to now()
 *       when it falls more than one period behind, preventing burst catch-up.
 *   [4] IMU buffer trim changed from erase-front to swap — O(1) instead of O(N).
 */

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <atomic>
#include <cmath>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include "System.h"
#include "ImuTypes.h"

// ══════════════════════════════════════════════════════
// PORTS
// ══════════════════════════════════════════════════════
#define FRAME_PORT  9997
#define IMU_PORT    9998
#define POSE_PORT   9999
#define MAP_PORT    9996

// ══════════════════════════════════════════════════════
// FPS TARGET
// ══════════════════════════════════════════════════════
#define TARGET_FPS        25
#define FRAME_PERIOD_MS   (1000 / TARGET_FPS)   // 40 ms per frame

// ══════════════════════════════════════════════════════
// MAP POINT CHUNKING
// UDP safe payload: keep each chunk well under 8KB.
// 200 points x 3 floats x 4 bytes = 2400 bytes + 2-byte header = 2402 bytes.
// Header: uint8_t chunk_index, uint8_t total_chunks
// ══════════════════════════════════════════════════════
#define MAP_CHUNK_POINTS  200
#define MAP_MAX_POINTS    1000   // send at most 1000 points total (was 3000 — UDP fragmentation)

// ══════════════════════════════════════════════════════
// IMU PACKET — must match struct.pack('d6f', ...) in Python
// double ts (8 bytes) + 6 x float (24 bytes) = 32 bytes
// ══════════════════════════════════════════════════════
#pragma pack(push,1)
struct ImuPacket {
    double ts;
    float  ax, ay, az;   // m/s^2
    float  gx, gy, gz;   // rad/s
};
#pragma pack(pop)
static_assert(sizeof(ImuPacket)==32, "ImuPacket must be 32 bytes");

// ══════════════════════════════════════════════════════
// GLOBALS
// ══════════════════════════════════════════════════════
std::mutex                         imu_mtx;
std::vector<ORB_SLAM3::IMU::Point> imu_buffer;
ORB_SLAM3::IMU::Point              last_imu(0,0,9.81, 0,0,0, 0.0);
std::atomic<bool>                  running(true);

// ══════════════════════════════════════════════════════
// SOCKET HELPERS
// ══════════════════════════════════════════════════════
int make_recv_sock(int port, int timeout_ms=100)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd<0){ perror("socket"); exit(1); }
    int reuse=1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    int bsz = 8*1024*1024;
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bsz, sizeof(bsz));
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in a{};
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port        = htons(port);
    if (bind(fd,(sockaddr*)&a,sizeof(a))<0){ perror("bind"); exit(1); }
    return fd;
}

int make_send_sock()
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd<0){ perror("socket-send"); exit(1); }
    int bsz = 4*1024*1024;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &bsz, sizeof(bsz));
    return fd;
}

// ══════════════════════════════════════════════════════
// send_map_points — chunked UDP sender
//
// FIX [2]: Previously sent all map points in one UDP datagram.
// At 3000 points that is 36KB — larger than most OS UDP send
// limits (~8KB default on Linux) so the datagram was silently
// dropped. Now splits into chunks of MAP_CHUNK_POINTS points,
// each chunk fits in a single safe UDP packet.
//
// Packet layout per chunk:
//   byte 0: chunk index (0-based)
//   byte 1: total chunks
//   bytes 2+: float[3 * MAP_CHUNK_POINTS]
// ══════════════════════════════════════════════════════
void send_map_points(int fd, const sockaddr_in& addr,
                     const std::vector<float>& flat)
{
    if (flat.empty()) return;
    const int pts_per_chunk = MAP_CHUNK_POINTS;
    const int floats_per_chunk = pts_per_chunk * 3;
    int total_floats = (int)flat.size();
    int total_chunks = (total_floats / 3 + pts_per_chunk - 1) / pts_per_chunk;
    if (total_chunks > 255) total_chunks = 255;  // header is uint8_t

    static std::vector<uint8_t> pkt;
    pkt.resize(2 + floats_per_chunk * sizeof(float));

    int sent_floats = 0;
    for (int ci = 0; ci < total_chunks && sent_floats < total_floats; ci++) {
        int chunk_floats = std::min(floats_per_chunk, total_floats - sent_floats);
        pkt[0] = (uint8_t)ci;
        pkt[1] = (uint8_t)total_chunks;
        std::memcpy(pkt.data() + 2, flat.data() + sent_floats,
                    chunk_floats * sizeof(float));
        sendto(fd, pkt.data(), 2 + chunk_floats * sizeof(float), 0,
               (const sockaddr*)&addr, sizeof(addr));
        sent_floats += chunk_floats;
    }
}

// ══════════════════════════════════════════════════════
// IMU THREAD — 100 Hz
// ══════════════════════════════════════════════════════
void imu_thread_fn()
{
    int fd = make_recv_sock(IMU_PORT, 50);
    std::cout << "[IMU ] Listening on port " << IMU_PORT << "\n";

    ImuPacket   pkt;
    sockaddr_in src{};
    socklen_t   srclen = sizeof(src);

    while (running) {
        ssize_t n = recvfrom(fd, &pkt, sizeof(pkt), 0,
                             (sockaddr*)&src, &srclen);
        if (n != (ssize_t)sizeof(pkt)) continue;
        if (!std::isfinite(pkt.ax) || !std::isfinite(pkt.gx)) continue;

        ORB_SLAM3::IMU::Point pt(
            pkt.ax, pkt.ay, pkt.az,
            pkt.gx, pkt.gy, pkt.gz,
            pkt.ts);

        std::lock_guard<std::mutex> lk(imu_mtx);
        imu_buffer.push_back(pt);
        last_imu = pt;

        // FIX [4]: swap trim — O(1) instead of O(N) erase-from-front.
        // Keep last 1000 samples (~10 seconds at 100Hz).
        if (imu_buffer.size() > 2000) {
            std::vector<ORB_SLAM3::IMU::Point> trimmed(
                imu_buffer.end() - 1000, imu_buffer.end());
            imu_buffer.swap(trimmed);
        }
    }
    close(fd);
}

// ══════════════════════════════════════════════════════
// SLAM THREAD — runs in background so main thread is free
// (viewer is OFF — set use_viewer=false in main)
// ══════════════════════════════════════════════════════
void slam_thread_fn(ORB_SLAM3::System* SLAM)
{
    int pose_fd = make_send_sock();
    int map_fd  = make_send_sock();

    sockaddr_in pose_addr{};
    pose_addr.sin_family      = AF_INET;
    pose_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    pose_addr.sin_port        = htons(POSE_PORT);

    sockaddr_in map_addr{};
    map_addr.sin_family      = AF_INET;
    map_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    map_addr.sin_port        = htons(MAP_PORT);

    int frame_fd = make_recv_sock(FRAME_PORT, FRAME_PERIOD_MS + 10);
    std::cout << "[FRAME] Listening on port " << FRAME_PORT
              << "  (target " << TARGET_FPS << " FPS)\n";

    // Wait for enough IMU data before starting SLAM
    std::cout << "[IMU ] Waiting for 50 samples...\n";
    int waited = 0;
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        {
            std::lock_guard<std::mutex> lk(imu_mtx);
            if (imu_buffer.size() >= 50) break;
        }
        if (++waited % 200 == 0) {
            std::lock_guard<std::mutex> lk(imu_mtx);
            std::cout << "  [IMU ] waiting... "
                      << imu_buffer.size() << " samples so far\n";
        }
    }
    std::cout << "[IMU ] Ready! Starting SLAM...\n\n";

    std::vector<uint8_t> recv_buf(512*1024);
    sockaddr_in src{};
    socklen_t   srclen = sizeof(src);

    int  frame_count = 0;
    int  track_ok    = 0;
    auto t0          = std::chrono::steady_clock::now();

    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::milliseconds;

    // FIX [3]: next_frame starts at now() so the first frame is not
    // immediately "behind" and we don't burst through multiple frames.
    auto next_frame = Clock::now();

    while (running) {

        // FPS throttle
        auto now = Clock::now();
        if (now < next_frame)
            std::this_thread::sleep_until(next_frame);

        // FIX [3]: if we fell more than one full period behind (e.g. a
        // slow frame or temporary stall), reset instead of bursting.
        // Without this, next_frame falls arbitrarily far behind after any
        // blocking recvfrom delay, and then all subsequent frames are
        // processed without any sleep, spiking CPU and starving the IMU thread.
        now = Clock::now();
        if (now > next_frame + Ms(FRAME_PERIOD_MS))
            next_frame = now;
        next_frame += Ms(FRAME_PERIOD_MS);

        // Receive JPEG frame
        ssize_t n = recvfrom(frame_fd, recv_buf.data(),
                             recv_buf.size(), 0,
                             (sockaddr*)&src, &srclen);
        if (n <= 8) continue;

        double ts;
        std::memcpy(&ts, recv_buf.data(), 8);

        std::vector<uint8_t> jpg(recv_buf.begin()+8,
                                  recv_buf.begin()+n);
        cv::Mat img = cv::imdecode(jpg, cv::IMREAD_GRAYSCALE);
        if (img.empty()) continue;

        // Collect IMU measurements up to this frame's timestamp
        std::vector<ORB_SLAM3::IMU::Point> imu_meas;
        {
            std::lock_guard<std::mutex> lk(imu_mtx);
            auto it = imu_buffer.begin();
            while (it != imu_buffer.end() && it->t <= ts)
                imu_meas.push_back(*it++);
            imu_buffer.erase(imu_buffer.begin(), it);

            if (imu_meas.empty()) {
                // No fresh IMU data — repeat last known measurement at a
                // slightly earlier timestamp so SLAM still has something to
                // integrate. This happens if network jitter delays an IMU packet.
                last_imu.t = ts - 0.005;
                imu_meas.push_back(last_imu);
            }
        }

        // Feed to ORB-SLAM3
        Sophus::SE3f pose;
        try {
            pose = SLAM->TrackMonocular(img, ts, imu_meas);
        } catch (const std::exception& e) {
            std::cerr << "[SLAM] Exception: " << e.what() << "\n";
            continue;
        }
        frame_count++;

        // Extract position
        int   raw_state = SLAM->GetTrackingState();
        float x=0, y=0, z=0;
        bool  pose_valid = false;
        try {
            Eigen::Vector3f t = pose.translation();
            if (std::isfinite(t(0)) && std::isfinite(t(1)) && std::isfinite(t(2))) {
                x = t(0); y = t(1); z = t(2);
                pose_valid = (x != 0.0f || y != 0.0f || z != 0.0f);
            }
        } catch(...) {}

        // State mapping
        // ORB-SLAM3 GetTrackingState():
        //   0=SYSTEM_NOT_READY  1=NO_IMAGES_YET  2=NOT_INITIALIZED
        //   3=OK(TRACKING)      4=RECENTLY_LOST  5=LOST
        //
        // Sent to pc_receiver:
        //   0=TRACKING  1=RECENTLY_LOST  2=LOST  3=INITIALIZING
        //
        // raw_state==2 lags behind map creation by 1-2 frames.
        // If the pose is already valid, treat it as TRACKING.
        int send_state;
        if (raw_state == 3) {
            send_state = 0; track_ok++;
        } else if (raw_state == 4) {
            send_state = 1;
        } else if (raw_state == 5) {
            send_state = 2;
        } else {
            send_state = pose_valid ? 0 : 3;
            if (pose_valid) track_ok++;
        }

        // Send pose
        char pose_buf[128];
        int  pose_len = snprintf(pose_buf, sizeof(pose_buf),
                                 "%.6f %.6f %.6f %d",
                                 x, y, z, send_state);
        sendto(pose_fd, pose_buf, pose_len, 0,
               (sockaddr*)&pose_addr, sizeof(pose_addr));

        // Send map points every 15 frames — chunked
        if (frame_count % 15 == 0) {
            auto mps = SLAM->GetTrackedMapPoints();
            std::vector<float> flat;
            flat.reserve(std::min((size_t)(MAP_MAX_POINTS*3), mps.size()*3));
            for (auto* mp : mps) {
                if (!mp || mp->isBad()) continue;
                Eigen::Vector3f wp = mp->GetWorldPos();
                if (!std::isfinite(wp(0))) continue;
                flat.push_back(wp(0));
                flat.push_back(wp(1));
                flat.push_back(wp(2));
                if ((int)flat.size() >= MAP_MAX_POINTS * 3) break;
            }
            // FIX [2]: send_map_points chunks into safe UDP packet sizes
            send_map_points(map_fd, map_addr, flat);
        }

        // Status log every 25 frames (one second at 25 FPS)
        if (frame_count % TARGET_FPS == 0) {
            auto elapsed = std::chrono::duration<double>(
                Clock::now() - t0).count();
            double fps = frame_count / elapsed;

            const char* sname;
            switch(send_state) {
                case 0:  sname="TRACKING";      break;
                case 1:  sname="RECENTLY_LOST"; break;
                case 2:  sname="LOST";          break;
                default: sname="INITIALIZING";  break;
            }
            std::cout << "[SLAM] Frame:" << frame_count
                      << "  FPS:"   << (int)fps
                      << "  IMU:"   << imu_meas.size()
                      << "  State:" << sname
                      << "  Pos:("  << x << "," << y << "," << z << ")\n";

            if (frame_count == TARGET_FPS)
                std::cout << "\n[HINT] Move camera SLOWLY left/right "
                             "over a textured scene to initialize!\n\n";
        }
    }

    close(frame_fd);
    close(pose_fd);
    close(map_fd);
}

// ══════════════════════════════════════════════════════
// MAIN
// FIX [1]: use_viewer is false — viewer is OFF (headless).
// The original comment said "Pangolin ON" but passed false,
// which disables it. This is correct for a headless PC receiver.
// To enable the viewer, change false -> true AND ensure this
// binary runs on a machine with a display (export DISPLAY=:0).
// ══════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "\nUsage: ./slam_receiver ORBvoc.txt webcam2.yaml\n\n";
        return 1;
    }

    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║  Monocular-Inertial SLAM Receiver    ║\n";
    std::cout << "╚══════════════════════════════════════╝\n";
    std::cout << "  Vocab : " << argv[1] << "\n";
    std::cout << "  Config: " << argv[2] << "\n";
    std::cout << "  Target: " << TARGET_FPS << " FPS  ("
              << FRAME_PERIOD_MS << " ms/frame)\n";
    std::cout << "  Viewer: OFF (headless)\n\n";

    // Init ORB-SLAM3, viewer OFF
    ORB_SLAM3::System SLAM(
        argv[1], argv[2],
        ORB_SLAM3::System::IMU_MONOCULAR,
        false   // use_viewer = false -> headless, no Pangolin window
    );

    // Start IMU listener thread
    std::thread imu_th(imu_thread_fn);
    imu_th.detach();

    // Start SLAM + socket thread
    std::thread slam_th(slam_thread_fn, &SLAM);
    slam_th.detach();

    std::cout << "[MAIN] SLAM + sockets running in background threads\n\n";

    // Keep main alive
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    running = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    SLAM.Shutdown();
    return 0;
}
