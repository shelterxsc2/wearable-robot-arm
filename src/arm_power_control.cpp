#include "arm_power_control.h"

#include "control_router.h"
#include "rga_npu.h"
#include "uart_comm.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>
#include <unistd.h>

namespace {
std::atomic<bool> powered{false};
std::atomic<bool> shutting_down{false};
std::atomic<long long> confirm_deadline_ms{0};
std::mutex power_mutex;
std::thread reinit_thread;

long long now_ms()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

bool send_safe_face_pose(bool wait_before_off)
{
    control_request_profile(1, CONTROL_SOURCE_LOCAL);
    control_request_mode(MODE_FACE, CONTROL_SOURCE_LOCAL);
    g_uart_block_tx = 0;
    if (send_face_home_pose() != 0) return false;
    g_uart_block_tx = 1;
    if (wait_before_off) {
        std::printf("[Power] safe FACE pose sent; waiting 3.0s before retract\n");
        for (int i = 0; i < 30 && powered.load(); ++i) usleep(100000);
    } else {
        std::printf("[PowerConfirm] safe FACE pose sent; show OK within 3.0s\n");
    }
    return true;
}

void finish_power_on()
{
    std::printf("[Power] waiting 7s for unfold/homing\n");
    for (int i = 0; i < 70 && powered.load(); ++i) usleep(100000);
    if (!powered.load()) return;
    request_nrf_rebaseline();
    g_wait_a_init = 1;
    for (int i = 0; i < 200 && powered.load() && !g_r_init_set; ++i) usleep(50000);
    if (!powered.load()) return;
    g_uart_block_tx = 0;
    shutting_down.store(false);
    std::printf("[Power] unfold complete; A-init=%d, NRF/UART control enabled\n",
                (int)g_r_init_set);
}
} // namespace

int arm_power_init(void)
{
    powered.store(false);
    shutting_down.store(false);
    confirm_deadline_ms.store(0);
    g_uart_block_tx = 1;
    return 0;
}

void arm_power_shutdown(void)
{
    confirm_deadline_ms.store(0);
    if (powered.load()) arm_power_off_safe();
    if (reinit_thread.joinable()) reinit_thread.join();
}

int arm_power_on(void)
{
    std::lock_guard<std::mutex> lock(power_mutex);
    if (powered.load() || !g_uart_init_success) return -1;
    if (reinit_thread.joinable()) reinit_thread.join();
    if (uart_send_power_on() != 0) return -1;
    powered.store(true);
    shutting_down.store(false);
    confirm_deadline_ms.store(0);
    g_uart_block_tx = 1;
    request_nrf_rebaseline();
    reinit_thread = std::thread(finish_power_on);
    std::printf("[Power] unfold frame sent\n");
    return 0;
}

int arm_power_request_voice_off(double timeout_seconds)
{
    std::lock_guard<std::mutex> lock(power_mutex);
    if (!powered.load() || get_pose_mode() != MODE_FACE ||
        confirm_deadline_ms.load() > now_ms()) return -1;
    shutting_down.store(true);
    if (!send_safe_face_pose(false)) {
        shutting_down.store(false);
        g_uart_block_tx = 0;
        return -1;
    }
    long long timeout_ms = (long long)(timeout_seconds * 1000.0);
    if (timeout_ms < 100) timeout_ms = 100;
    confirm_deadline_ms.store(now_ms() + timeout_ms);
    return 0;
}

int arm_power_confirm_gesture(void)
{
    std::lock_guard<std::mutex> lock(power_mutex);
    long long deadline = confirm_deadline_ms.exchange(0);
    if (!powered.load() || deadline <= 0 || now_ms() > deadline) return -1;
    if (uart_send_power_off() != 0) {
        shutting_down.store(false);
        g_uart_block_tx = 0;
        return -1;
    }
    powered.store(false);
    std::printf("[PowerConfirm] OK confirmed; retract frame sent\n");
    return 0;
}

int arm_power_off_safe(void)
{
    std::lock_guard<std::mutex> lock(power_mutex);
    if (!powered.load()) return -1;
    shutting_down.store(true);
    confirm_deadline_ms.store(0);
    if (!send_safe_face_pose(true) || uart_send_power_off() != 0) return -1;
    powered.store(false);
    std::printf("[Power] retract frame sent\n");
    return 0;
}

int arm_power_is_on(void) { return powered.load() ? 1 : 0; }
int arm_power_shutdown_in_progress(void) { return shutting_down.load() ? 1 : 0; }

void arm_power_tick(void)
{
    long long deadline = confirm_deadline_ms.load();
    if (deadline > 0 && now_ms() > deadline &&
        confirm_deadline_ms.compare_exchange_strong(deadline, 0)) {
        shutting_down.store(false);
        if (powered.load()) g_uart_block_tx = 0;
        std::printf("[PowerConfirm] timeout; retract cancelled\n");
    }
}
