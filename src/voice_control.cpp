#include "voice_control.h"
#include "control_router.h"
#include "arm_power_control.h"
#include "cloud_report.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

static std::atomic<bool> running{false};
static std::thread worker;
static std::string path;
static std::atomic<int> active_fd{-1};

static long long mono_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static std::string field(const std::string &line, const char *name) {
    std::string key = std::string("\"") + name + "\":\"";
    auto p = line.find(key);
    if (p == std::string::npos) return {};
    p += key.size();
    auto e = line.find('"', p);
    return e == std::string::npos ? std::string() : line.substr(p, e - p);
}

static long long number(const std::string &line, const char *name) {
    std::string key = std::string("\"") + name + "\":";
    auto p = line.find(key);
    return p == std::string::npos ? -1 : std::strtoll(line.c_str() + p + key.size(), nullptr, 10);
}

static void dispatch(const std::string &line) {
    auto keyword = field(line, "keyword");
    if (!keyword.empty() && keyword[0] == '@') keyword.erase(0, 1);
    auto at = keyword.rfind('@');
    if (at != std::string::npos) keyword = keyword.substr(at + 1);
    long long detected = number(line, "detected_ms");
    if (keyword.empty() || detected < 0 || mono_ms() - detected > 500) return;
    if (keyword == "原画" || keyword == "标注") {
        int enabled = keyword == "标注";
        control_set_annotation_enabled(enabled, CONTROL_SOURCE_VOICE);
        cloud_report_enqueue(enabled ?
            "{\"type\":\"annotation_mode\",\"annotationMode\":true}" :
            "{\"type\":\"annotation_mode\",\"annotationMode\":false}");
    } else if (keyword == "拉远" || keyword == "拉近") {
        int profile = keyword == "拉近" ? 0 : 1;
        control_request_profile(profile, CONTROL_SOURCE_VOICE);
        cloud_report_enqueue(profile == 0 ?
            "{\"type\":\"zoom\",\"zoom\":2}" :
            "{\"type\":\"zoom\",\"zoom\":0}");
    } else if (keyword == "介绍" || keyword == "采访" ||
               keyword == "正面" || keyword == "正脸") {
        PoseMode old = get_pose_mode();
        PoseMode mode = keyword == "介绍" ? MODE_INTRO :
                        (keyword == "采访" ? MODE_INTERVIEW : MODE_FACE);
        if (old == MODE_FIRST_PERSON)
            cloud_report_enqueue("{\"type\":\"track_obj\",\"trackObj\":1}");
        control_request_mode(mode, CONTROL_SOURCE_VOICE);
        cloud_report_enqueue(mode == MODE_INTRO ?
            "{\"type\":\"view_mode\",\"viewMode\":2}" :
            (mode == MODE_INTERVIEW ?
             "{\"type\":\"view_mode\",\"viewMode\":0}" :
             "{\"type\":\"view_mode\",\"viewMode\":1}"));
    } else if (keyword == "并肩") {
        if (get_pose_mode() != MODE_FIRST_PERSON) {
            control_request_mode(MODE_FIRST_PERSON, CONTROL_SOURCE_VOICE);
            cloud_report_enqueue("{\"type\":\"track_obj\",\"trackObj\":0}");
        }
    } else if (keyword == "开机") {
        std::printf("[VoiceCmd] 开机 -> power_on result=%d\n", arm_power_on());
    } else if (keyword == "关机") {
        std::printf("[VoiceCmd] 关机 -> wait OK result=%d\n",
                    arm_power_request_voice_off(3.0));
    } else if (keyword == "录制" || keyword == "停止") {
        static std::atomic<unsigned int> request_id{0};
        char packet[256];
        std::snprintf(packet, sizeof(packet),
            "{\"type\":\"record_control\",\"action\":\"%s\","
            "\"requestId\":\"voice-%lld-%u\","
            "\"metadata\":{\"source\":\"voice\",\"keyword\":\"%s\"}}",
            keyword == "录制" ? "start" : "stop", mono_ms(),
            ++request_id, keyword.c_str());
        cloud_report_enqueue(packet);
    } else {
        std::printf("[VoiceCmd] ignored keyword=%s\n", keyword.c_str());
    }
}

static void loop() {
    while (running) {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        sockaddr_un addr{}; addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path.c_str());
        if (fd < 0 || connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
            if (fd >= 0) close(fd);
            for (int i = 0; i < 10 && running; ++i) usleep(100000);
            continue;
        }
        active_fd.store(fd);
        std::string pending;
        char buf[1024];
        while (running) {
            ssize_t n = read(fd, buf, sizeof(buf));
            if (n <= 0) break;
            pending.append(buf, n);
            for (;;) {
                auto e = pending.find('\n');
                if (e == std::string::npos) break;
                dispatch(pending.substr(0, e));
                pending.erase(0, e + 1);
            }
            if (pending.size() > 4096) pending.clear();
        }
        close(fd);
        active_fd.store(-1);
    }
}

int voice_control_start(const char *socket_path) {
    if (!socket_path || running.exchange(true)) return -1;
    path = socket_path;
    worker = std::thread(loop);
    return 0;
}
void voice_control_stop(void) {
    if (!running.exchange(false)) return;
    int fd = active_fd.load();
    if (fd >= 0) shutdown(fd, SHUT_RDWR);
    if (worker.joinable()) worker.join();
}
