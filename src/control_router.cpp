#include "control_router.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>

static std::atomic<unsigned int> g_mode_generation{1};
static std::atomic<int> g_annotation_enabled{1};
static std::mutex g_control_mutex;

const char *control_source_name(ControlSource source)
{
    switch (source) {
        case CONTROL_SOURCE_REST: return "rest";
        case CONTROL_SOURCE_CLOUD: return "cloud";
        case CONTROL_SOURCE_BLUETOOTH: return "bluetooth";
        case CONTROL_SOURCE_GESTURE: return "gesture";
        case CONTROL_SOURCE_VOICE: return "voice";
        case CONTROL_SOURCE_RULE_ENGINE: return "rule_engine";
        case CONTROL_SOURCE_LOCAL: return "local";
    }
    return "unknown";
}

int control_mode_from_string(const char *name, PoseMode *mode)
{
    if (!name || !mode) return -1;
    if (strcmp(name, "face") == 0) *mode = MODE_FACE;
    else if (strcmp(name, "body") == 0) *mode = MODE_BODY;
    else if (strcmp(name, "intro") == 0) *mode = MODE_INTRO;
    else if (strcmp(name, "interview") == 0) *mode = MODE_INTERVIEW;
    else if (strcmp(name, "first_person") == 0 || strcmp(name, "first-person") == 0)
        *mode = MODE_FIRST_PERSON;
    else return -1;
    return 0;
}

int control_request_mode(PoseMode mode, ControlSource source)
{
    if (mode < MODE_FACE || mode > MODE_FIRST_PERSON) return -1;
    std::lock_guard<std::mutex> lock(g_control_mutex);
    PoseMode old = get_pose_mode();
    if (old == mode) return 0;
    unsigned int generation = g_mode_generation.fetch_add(1) + 1;
    printf("[ControlRouter] source=%s mode=%s generation=%u\n",
           control_source_name(source), pose_mode_name(mode), generation);
    set_pose_mode(mode);
    return 0;
}

int control_request_profile(int profile, ControlSource source)
{
    if (profile < 0 || profile > 1) return -1;
    std::lock_guard<std::mutex> lock(g_control_mutex);
    printf("[ControlRouter] source=%s profile=%d\n",
           control_source_name(source), profile);
    set_arm_profile(profile);
    return 0;
}

int control_request_mode_if_generation(PoseMode mode, ControlSource source,
                                       unsigned int expected_generation)
{
    if (mode < MODE_FACE || mode > MODE_FIRST_PERSON) return -1;
    std::lock_guard<std::mutex> lock(g_control_mutex);
    if (g_mode_generation.load() != expected_generation) return 1;
    PoseMode old = get_pose_mode();
    if (old == mode) return 0;
    unsigned int generation = g_mode_generation.fetch_add(1) + 1;
    printf("[ControlRouter] source=%s mode=%s generation=%u\n",
           control_source_name(source), pose_mode_name(mode), generation);
    set_pose_mode(mode);
    return 0;
}

int control_request_profile_if_generation(int profile, ControlSource source,
                                          unsigned int expected_generation)
{
    if (profile < 0 || profile > 1) return -1;
    std::lock_guard<std::mutex> lock(g_control_mutex);
    if (g_mode_generation.load() != expected_generation) return 1;
    printf("[ControlRouter] source=%s profile=%d\n",
           control_source_name(source), profile);
    set_arm_profile(profile);
    return 0;
}

int control_request_first_person_target(float x, float y, float z,
                                        ControlSource source)
{
    if (x < -100.0f || x > 100.0f || y < 10.0f || y > 150.0f ||
        z < -50.0f || z > 100.0f) {
        return -1;
    }
    std::lock_guard<std::mutex> lock(g_control_mutex);
    set_first_person_target(x, y, z);
    printf("[ControlRouter] source=%s first_person_target=%.1f/%.1f/%.1f\n",
           control_source_name(source), x, y, z);
    return 0;
}

void control_set_annotation_enabled(int enabled, ControlSource source)
{
    std::lock_guard<std::mutex> lock(g_control_mutex);
    g_annotation_enabled.store(enabled ? 1 : 0);
    printf("[ControlRouter] source=%s annotation=%d\n",
           control_source_name(source), enabled ? 1 : 0);
}

int control_get_annotation_enabled(void)
{
    return g_annotation_enabled.load();
}

unsigned int control_get_mode_generation(void)
{
    return g_mode_generation.load();
}
