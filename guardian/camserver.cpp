// camserver: publish the newest normal-exposure SLAM camera frames to shared memory. Runs on a thread inside vrserver
// (driver.cpp): pidfd_getfd on XRService needs ptrace rights, which Yama (ptrace_scope=1) only grants to XRService's
// ancestors (vrserver is its parent) or CAP_SYS_PTRACE.
// Read-only with respect to XRService: VIDIOC_QUERYBUF + mmap of its dmabufs, nothing else.
// (video0 = og01a1b 4-0036 = slam_right, video3 = og01a1b 4-0060 = slam_left; verified by rectifying
// both with calib.txt: only this assignment gives an upright pair with purely horizontal disparity.)
#include <dirent.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

#include "shm.h"

static std::atomic<bool> g_quit{false};

void (*camserver_log)(const char* line) = [](const char* line) {
    fputs(line, stderr);
};

__attribute__((format(printf, 1, 2))) static void log_line(const char* fmt, ...) {
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    camserver_log(line);
}

static uint64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

static int find_xrservice() {
    DIR* d = opendir("/proc");
    int found = -1;
    while (dirent* e = readdir(d)) {
        int pid = atoi(e->d_name);
        if (pid <= 0) {
            continue;
        }
        // Match the executable: the main thread renames itself (comm is "XRServiceLoopTh").
        char path[64], exe[256] = {};
        snprintf(path, sizeof path, "/proc/%d/exe", pid);
        ssize_t n = readlink(path, exe, sizeof exe - 1);
        const char* base = n > 0 ? strrchr(exe, '/') : nullptr;
        if (base && strcmp(base, "/XRService") == 0) {
            found = pid;
            break;
        }
    }
    closedir(d);
    return found;
}

static int find_fd(int pid, const std::string& target) {
    char dir[64];
    snprintf(dir, sizeof dir, "/proc/%d/fd", pid);
    DIR* d = opendir(dir);
    if (!d) {
        return -1;
    }
    int found = -1;
    while (dirent* e = readdir(d)) {
        char path[128], link[256] = {};
        snprintf(path, sizeof path, "%s/%s", dir, e->d_name);
        ssize_t n = readlink(path, link, sizeof link - 1);
        if (n > 0 && target == link) {
            found = atoi(e->d_name);
            break;
        }
    }
    closedir(d);
    return found;
}

struct Mapping {
    int fd;  // our dup of the dmabuf
    ino_t ino;
    uint8_t* p;
};

struct Camera {
    std::string node;
    int remote_vfd = -1;          // XRService's fd for /dev/videoN
    std::map<int, Mapping> maps;  // by XRService's dmabuf fd
    uint32_t last_published = UINT32_MAX;
    double normal_mean = 0;               // running brightness of normal-exposure frames
    int64_t clock_offset_ns = INT64_MIN;  // camera clock - CLOCK_MONOTONIC (at arrival)

    void unmap(const Mapping& m) {
        munmap(m.p, shm::kStride * shm::kH);
        close(m.fd);
    }
    void close_all() {
        for (auto& [_, m] : maps) {
            unmap(m);
        }
        maps.clear();
        remote_vfd = -1;
        last_published = UINT32_MAX;  // a restarted XRService counts from 0 again
    }
};

struct Buf {
    uint32_t seq;
    uint64_t ts_ns;
    int remote_fd;
};

// The video fd is dup'd only for the duration of the query: a dup we kept would keep XRService's open file (and its
// capture queue) alive after XRService closes it or exits, and the restarted XRService would get EBUSY on the camera.
// False when XRService is gone or the fd isn't the camera any more.
static bool query(const Camera& c, int pidfd, std::vector<Buf>& out) {
    out.clear();
    int vfd = syscall(SYS_pidfd_getfd, pidfd, c.remote_vfd, 0);
    if (vfd < 0) {
        return false;
    }
    bool ok = true;
    for (uint32_t i = 0; i < 64; i++) {
        v4l2_plane planes[VIDEO_MAX_PLANES] = {};
        v4l2_buffer b = {};
        b.index = i;
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        b.length = VIDEO_MAX_PLANES;
        b.m.planes = planes;
        if (ioctl(vfd, VIDIOC_QUERYBUF, &b) < 0) {
            ok = errno == EINVAL;  // EINVAL = past last index
            break;
        }
        if (b.flags & V4L2_BUF_FLAG_QUEUED || b.timestamp.tv_sec == 0) {
            continue;
        }
        out.push_back(
            {b.sequence, uint64_t(b.timestamp.tv_sec) * 1000000000ull + b.timestamp.tv_usec * 1000ull, planes[0].m.fd});
    }
    close(vfd);
    return ok;
}

// The cached mapping is only reused while XRService's fd still names the same dmabuf (each has its own inode): fd
// numbers come back for new buffers when XRService reallocates, and the old mapping would serve a frozen frame.
static const Mapping* map_buf(Camera& c, int pidfd, int remote_fd) {
    int fd = syscall(SYS_pidfd_getfd, pidfd, remote_fd, 0);
    struct stat st;
    if (fd < 0 || fstat(fd, &st) < 0) {
        if (fd >= 0) {
            close(fd);
        }
        return nullptr;
    }
    auto it = c.maps.find(remote_fd);
    if (it != c.maps.end()) {
        if (it->second.ino == st.st_ino) {
            close(fd);
            return &it->second;
        }
        c.unmap(it->second);
        c.maps.erase(it);
    }
    void* p = mmap(nullptr, shm::kStride * shm::kH, PROT_READ, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        close(fd);
        return nullptr;
    }
    return &(c.maps[remote_fd] = {fd, st.st_ino, (uint8_t*)p});
}

static double sample_mean(const uint8_t* y) {
    uint64_t s = 0, n = 0;
    for (int r = 32; r < shm::kH - 32; r += 16) {
        for (int x = 32; x < shm::kW - 32; x += 16) {
            s += y[r * shm::kStride + x];
            n++;
        }
    }
    return double(s) / n;
}

void camserver_stop() {
    g_quit = true;
}

// Runs until camserver_stop(). nodes: left, right.
int camserver_run(const std::string (&nodes)[shm::kNumCams]) {
    int shm_fd = open(shm::kPath, O_RDWR | O_CREAT, 0644);
    if (shm_fd < 0 || fchmod(shm_fd, 0644) < 0 || ftruncate(shm_fd, sizeof(shm::Header)) < 0) {
        log_line("%s: %s\n", shm::kPath, strerror(errno));
        return 1;
    }
    auto* hdr = (shm::Header*)mmap(nullptr, sizeof(shm::Header), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (hdr == MAP_FAILED) {
        log_line("mmap %s: %s\n", shm::kPath, strerror(errno));
        close(shm_fd);
        return 1;
    }
    hdr->magic = shm::kMagic;

    Camera cams[shm::kNumCams];
    for (int i = 0; i < shm::kNumCams; i++) {
        cams[i].node = nodes[i];
    }
    std::vector<uint8_t> frame(shm::kStride * shm::kH);
    std::vector<Buf> bufs;
    int pid = -1, pidfd = -1;
    bool waiting = false;
    uint64_t published = 0, last_report = now_ns();

    while (!g_quit) {
        if (pid < 0) {
            for (auto& c : cams) {
                c.close_all();
            }
            if (pidfd >= 0) {
                close(pidfd);
            }
            pid = find_xrservice();
            pidfd = pid > 0 ? syscall(SYS_pidfd_open, pid, 0) : -1;
            bool ok = pidfd >= 0;
            for (auto& c : cams) {
                c.remote_vfd = ok ? find_fd(pid, c.node) : -1;
                ok = ok && c.remote_vfd >= 0;
            }
            if (!ok) {
                if (!waiting) {
                    log_line("waiting for XRService (%s)\n", pidfd < 0 ? "not running" : strerror(errno));
                }
                waiting = true;
                pid = -1;
                sleep(1);
                continue;
            }
            waiting = false;
            log_line("attached to XRService pid %d (%s=left, %s=right)\n", pid, cams[0].node.c_str(),
                     cams[1].node.c_str());
        }

        for (int ci = 0; ci < shm::kNumCams; ci++) {
            Camera& c = cams[ci];
            // XRService went away or reset its queue
            if (!query(c, pidfd, bufs)) {
                pid = -1;
                break;
            }
            uint64_t arrival = now_ns();
            std::sort(bufs.begin(), bufs.end(), [](const Buf& a, const Buf& b) {
                return a.seq > b.seq;
            });
            if (bufs.empty() || bufs[0].seq == c.last_published) {
                continue;
            }
            // The camera clock runs ~210 ms ahead of CLOCK_MONOTONIC. Track the offset as the
            // max of (timestamp - arrival): the least-delayed observation bounds it best.
            int64_t off = int64_t(bufs[0].ts_ns) - int64_t(arrival);
            c.clock_offset_ns = c.clock_offset_ns == INT64_MIN ? off : std::max(c.clock_offset_ns - 1000, off);

            // Newest completed frame that is a normal exposure (not the ~37us controller-LED one).
            for (const Buf& b : bufs) {
                if (b.seq <= c.last_published && c.last_published != UINT32_MAX) {
                    break;
                }
                const Mapping* mb = map_buf(c, pidfd, b.remote_fd);
                if (!mb) {
                    continue;
                }
                dma_buf_sync sync = {DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ};
                ioctl(mb->fd, DMA_BUF_IOCTL_SYNC, &sync);
                memcpy(frame.data(), mb->p, frame.size());
                sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
                ioctl(mb->fd, DMA_BUF_IOCTL_SYNC, &sync);
                double m = sample_mean(frame.data());
                // ponytail: brightness heuristic; black level ~16, controller frames sit right on it
                bool normal = c.normal_mean == 0 ? m > 20 : m > 16 + 0.3 * (c.normal_mean - 16);
                if (!normal) {
                    continue;
                }
                c.normal_mean = c.normal_mean == 0 ? m : 0.9 * c.normal_mean + 0.1 * m;
                shm::write(hdr->slots[ci], frame.data(), b.ts_ns - c.clock_offset_ns, b.seq);
                c.last_published = b.seq;
                published++;
                break;
            }
        }

        uint64_t t = now_ns();
        if (t - last_report > 5000000000ull) {
            log_line("published %.1f frames/s; clock offset %.2f / %.2f ms; mean %.0f / %.0f\n",
                     published / ((t - last_report) / 1e9), cams[0].clock_offset_ns / 1e6,
                     cams[1].clock_offset_ns / 1e6, cams[0].normal_mean, cams[1].normal_mean);
            published = 0;
            last_report = t;
        }
        usleep(1000);
    }
    for (auto& c : cams) {
        c.close_all();
    }
    if (pidfd >= 0) {
        close(pidfd);
    }
    munmap(hdr, sizeof(shm::Header));
    close(shm_fd);
    return 0;
}
