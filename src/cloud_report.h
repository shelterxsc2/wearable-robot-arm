#ifndef CLOUD_REPORT_H
#define CLOUD_REPORT_H
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
int cloud_report_enqueue(const char *json);
int cloud_report_take(char *json, size_t capacity);
#ifdef __cplusplus
}
#endif
#endif
