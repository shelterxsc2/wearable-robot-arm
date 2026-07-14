#ifndef CLOUD_COMMAND_H
#define CLOUD_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

/* Dispatch one cloud JSON packet. Returns 1 handled, 0 ignored, -1 invalid. */
int control_handle_cloud_json(const char *json);

#ifdef __cplusplus
}
#endif

#endif
