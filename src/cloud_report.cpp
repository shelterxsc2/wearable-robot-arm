#include "cloud_report.h"
#include <deque>
#include <mutex>
#include <string>

namespace { std::mutex m; std::deque<std::string> q; }

int cloud_report_enqueue(const char *json)
{
    if (!json || !*json) return -1;
    std::lock_guard<std::mutex> lock(m);
    if (q.size() >= 32) q.pop_front();
    q.emplace_back(json);
    return 0;
}

int cloud_report_take(char *json, size_t capacity)
{
    if (!json || capacity == 0) return -1;
    std::lock_guard<std::mutex> lock(m);
    if (q.empty()) return 0;
    std::string value = std::move(q.front()); q.pop_front();
    if (value.size() >= capacity) return -1;
    value.copy(json, value.size()); json[value.size()] = '\0';
    return 1;
}
