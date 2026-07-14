#include "cloud_command.h"
#include "control_router.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static const char *find_value(const char *json, const char *key)
{
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(json, needle);
    if (!p) return NULL;
    p = strchr(p + strlen(needle), ':');
    if (!p) return NULL;
    do { ++p; } while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n');
    return p;
}

static int json_string(const char *json, const char *key, char *out, size_t cap)
{
    const char *p = find_value(json, key);
    if (!p || *p != '"' || cap == 0) return -1;
    ++p;
    const char *end = strchr(p, '"');
    if (!end) return -1;
    size_t n = (size_t)(end - p);
    if (n >= cap) n = cap - 1;
    memcpy(out, p, n);
    out[n] = '\0';
    return 0;
}

static int json_number(const char *json, const char *key, double *out)
{
    const char *p = find_value(json, key);
    if (!p) return -1;
    char *end = NULL;
    double value = strtod(p, &end);
    if (end == p) return -1;
    *out = value;
    return 0;
}

static int json_bool(const char *json, const char *key, int *out)
{
    const char *p = find_value(json, key);
    if (!p) return -1;
    if (strncmp(p, "true", 4) == 0 || *p == '1') { *out = 1; return 0; }
    if (strncmp(p, "false", 5) == 0 || *p == '0') { *out = 0; return 0; }
    return -1;
}

int control_handle_cloud_json(const char *json)
{
    if (!json) return -1;
    char type[48] = "";
    char mode_name[48] = "";
    (void)json_string(json, "type", type, sizeof(type));

    if (strcmp(type, "track_obj") == 0) {
        double value;
        if (json_number(json, "trackObj", &value) < 0) return -1;
        return control_request_mode(value < 0.5 ? MODE_FIRST_PERSON : MODE_FACE,
                                    CONTROL_SOURCE_CLOUD) == 0 ? 1 : -1;
    }
    if (strcmp(type, "set_view_mode") == 0) {
        double value;
        if (json_number(json, "viewMode", &value) < 0) return -1;
        PoseMode mode = value < 0.5 ? MODE_INTERVIEW :
                        (value >= 1.5 ? MODE_INTRO : MODE_FACE);
        return control_request_mode(mode, CONTROL_SOURCE_CLOUD) == 0 ? 1 : -1;
    }
    if (strcmp(type, "set_zoom") == 0) {
        double value;
        if (json_number(json, "zoom", &value) < 0) return -1;
        int profile = value >= 1.5 ? 0 : 1; /* trial0 protocol mapping */
        return control_request_profile(profile, CONTROL_SOURCE_CLOUD) == 0 ? 1 : -1;
    }
    if (strcmp(type, "set_annotation_mode") == 0) {
        int enabled;
        if (json_bool(json, "annotationMode", &enabled) < 0) return -1;
        control_set_annotation_enabled(enabled, CONTROL_SOURCE_CLOUD);
        return 1;
    }
    if (strcmp(type, "set_target") == 0 || strcmp(type, "target_pose") == 0) {
        double x, y, z;
        if (json_number(json, "x", &x) < 0 || json_number(json, "y", &y) < 0 ||
            json_number(json, "z", &z) < 0) return -1;
        /* Cloud D-pad values are -5..5; preserve trial0's cm mapping. */
        if (x >= -5 && x <= 5 && y >= -5 && y <= 5 && z >= -5 && z <= 5) {
            x = -20.0 + x * 3.0;
            y = 30.0 + y * 5.0;
            z = 20.0 + z * 3.0;
        }
        if (control_request_first_person_target((float)x, (float)y, (float)z,
                                                CONTROL_SOURCE_CLOUD) < 0) return -1;
        control_request_mode(MODE_FIRST_PERSON, CONTROL_SOURCE_CLOUD);
        return 1;
    }

    /* Generic command form for future REST/voice gateways. */
    if (json_string(json, "mode", mode_name, sizeof(mode_name)) == 0) {
        PoseMode mode;
        if (control_mode_from_string(mode_name, &mode) < 0) return -1;
        return control_request_mode(mode, CONTROL_SOURCE_CLOUD) == 0 ? 1 : -1;
    }
    return 0;
}
