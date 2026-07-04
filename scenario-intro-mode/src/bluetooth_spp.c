/**
 * bluetooth_spp.c - BLE GATT 客户端：连接 HC-08 (ffe0/ffe1)
 *
 * HC-08 是 BLE UART 透传模块，通信方式：
 *   Service UUID:    0000ffe0-0000-1000-8000-00805f9b34fb
 *   Characteristic:  0000ffe1-0000-1000-8000-00805f9b34fb (read/write)
 *
 * 程序作为 BLE Central 主动连接 HC-08，通过 GATT ReadValue 轮询
 * 接收数据，WriteValue 发送数据。
 */
#include "bluetooth_spp.h"
#include "rga_npu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdint.h>
#include <sys/wait.h>
#include <dbus/dbus.h>

#define BT24_NAME       "HC-08"
#define BT24_MAC        "F8:2E:0C:E3:99:C8"
#define BT24_CHR_UUID   "0000ffe1-0000-1000-8000-00805f9b34fb"
#define BT_BUF_SIZE     256

static volatile int     g_bt_running = 0;
static pthread_t        g_client_thread;
static DBusConnection  *g_dbus_conn = NULL;

/* 连接状态 */
static volatile int     g_bt_connected = 0;
static pthread_mutex_t  g_conn_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t   g_conn_cond = PTHREAD_COND_INITIALIZER;

/* HC-08 设备路径和 characteristic 路径 */
static char             g_device_path[128] = "";
static char             g_char_path[256] = "";
static volatile int     g_notify_enabled = 0;

/* ============================================================
 * 内部辅助函数
 * ============================================================ */
static int run_shell_cmd(const char *cmd)
{
    int ret = system(cmd);
    if (ret == -1) return -1;
    return WEXITSTATUS(ret);
}

static void str_trim(char *s)
{
    if (!s || !*s) return;
    char *p = s;
    while (isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[len - 1] = '\0';
        len--;
    }
}

static int str_ieq(const char *a, const char *b)
{
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
            return 0;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

/* ============================================================
 * 三字节遥控协议:
 *   55 00 00       idle
 *   55 01 00/01    arm profile: 00=L3-40, 01=L3-55
 *   55 02 00/01/02 scene reserved, log only
 *   55 03 xx       toggle IMU pitch sign
 * ============================================================ */
static void handle_remote_frame(uint8_t cmd, uint8_t value)
{
    static int active_non_idle = 0;
    static uint8_t last_cmd = 0;
    static uint8_t last_value = 0;

    if (cmd == 0x00) {
        active_non_idle = 0;
        last_cmd = 0;
        last_value = 0;
        return;
    }

    if (active_non_idle && cmd == last_cmd &&
        (cmd == 0x03 || value == last_value)) {
        return;
    }
    active_non_idle = 1;
    last_cmd = cmd;
    last_value = value;

    switch (cmd) {
        case 0x01:
            if (value == 0x00) {
                set_arm_profile(0);
                printf("[BT] Remote profile -> near L3=40\n");
            } else if (value == 0x01) {
                set_arm_profile(1);
                printf("[BT] Remote profile -> mid L3=55\n");
            } else {
                printf("[BT] Remote profile value 0x%02X ignored\n", value);
            }
            break;

        case 0x02:
            printf("[BT] Remote scene value 0x%02X received (reserved)\n", value);
            break;

        case 0x03: {
            int sign = toggle_head_pitch_sign();
            printf("[BT] Remote toggled IMU pitch sign -> %+d\n", sign);
            break;
        }

        default:
            printf("[BT] Unknown remote command cmd=0x%02X value=0x%02X\n",
                   cmd, value);
            break;
    }
}

static void handle_raw_data(const char *data, int len)
{
    if (len <= 0) return;
    static uint8_t frame[3];
    static int frame_pos = 0;

    for (int i = 0; i < len; i++) {
        unsigned char c = (unsigned char)data[i];

        if (frame_pos == 0) {
            if (c != 0x55) {
                continue;
            }
            frame[frame_pos++] = c;
            continue;
        }

        frame[frame_pos++] = c;
        if (frame_pos == 3) {
            handle_remote_frame(frame[1], frame[2]);
            frame_pos = 0;
        }
    }
}

/* ============================================================
 * D-Bus 辅助函数
 * ============================================================ */

/* Notify 信号过滤器：接收 ffe1 的 Value 属性变化 */
static DBusHandlerResult notify_filter(DBusConnection *conn, DBusMessage *msg, void *user_data)
{
    (void)conn; (void)user_data;
    if (strlen(g_char_path) == 0)
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

    if (dbus_message_is_signal(msg, "org.freedesktop.DBus.Properties", "PropertiesChanged")) {
        const char *path = dbus_message_get_path(msg);
        if (!path || strcmp(path, g_char_path) != 0)
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

        DBusMessageIter args, dict_iter;
        if (!dbus_message_iter_init(msg, &args))
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

        /* 跳过接口名 */
        if (dbus_message_iter_get_arg_type(&args) != DBUS_TYPE_STRING)
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
        dbus_message_iter_next(&args);

        /* 解析属性字典 */
        if (dbus_message_iter_get_arg_type(&args) != DBUS_TYPE_ARRAY)
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
        dbus_message_iter_recurse(&args, &dict_iter);

        while (dbus_message_iter_get_arg_type(&dict_iter) == DBUS_TYPE_DICT_ENTRY) {
            DBusMessageIter entry, variant;
            char *prop_name = NULL;
            dbus_message_iter_recurse(&dict_iter, &entry);
            if (dbus_message_iter_get_arg_type(&entry) == DBUS_TYPE_STRING)
                dbus_message_iter_get_basic(&entry, &prop_name);

            if (prop_name && strcmp(prop_name, "Value") == 0) {
                dbus_message_iter_next(&entry);
                if (dbus_message_iter_get_arg_type(&entry) == DBUS_TYPE_VARIANT) {
                    dbus_message_iter_recurse(&entry, &variant);
                    DBusMessageIter byte_iter;
                    uint8_t buf[BT_BUF_SIZE];
                    int n = 0;
                    if (dbus_message_iter_get_arg_type(&variant) == DBUS_TYPE_ARRAY) {
                        dbus_message_iter_recurse(&variant, &byte_iter);
                        while (dbus_message_iter_get_arg_type(&byte_iter) == DBUS_TYPE_BYTE && n < BT_BUF_SIZE) {
                            dbus_message_iter_get_basic(&byte_iter, &buf[n++]);
                            dbus_message_iter_next(&byte_iter);
                        }
                    }
                    if (n > 0) {
                        // printf("[BT] Notify: %d bytes", n);
                        // for (int i = 0; i < n && i < 8; i++) printf(" %02X", buf[i]);
                        // if (n > 8) printf(" ...");
                        // printf("\n");
                        handle_raw_data((char *)buf, n);
                    }
                }
            }
            dbus_message_iter_next(&dict_iter);
        }
        return DBUS_HANDLER_RESULT_HANDLED;
    }
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

static int dbus_init(void)
{
    DBusError err;
    dbus_error_init(&err);
    g_dbus_conn = dbus_bus_get(DBUS_BUS_SYSTEM, &err);
    if (!g_dbus_conn) {
        fprintf(stderr, "[BT] dbus_bus_get failed: %s\n", err.message);
        dbus_error_free(&err);
        return -1;
    }

    /* 订阅 PropertiesChanged 信号 */
    dbus_bus_add_match(g_dbus_conn,
        "type='signal',interface='org.freedesktop.DBus.Properties',member='PropertiesChanged'",
        &err);
    if (dbus_error_is_set(&err)) {
        fprintf(stderr, "[BT] dbus_bus_add_match failed: %s\n", err.message);
        dbus_error_free(&err);
    }
    dbus_connection_add_filter(g_dbus_conn, notify_filter, NULL, NULL);
    return 0;
}

static int get_property_bool(const char *obj_path, const char *iface, const char *prop)
{
    DBusError err;
    dbus_error_init(&err);
    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", obj_path,
        "org.freedesktop.DBus.Properties", "Get");
    if (!msg) return 0;

    dbus_message_append_args(msg,
        DBUS_TYPE_STRING, &iface,
        DBUS_TYPE_STRING, &prop,
        DBUS_TYPE_INVALID);

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) dbus_error_free(&err);
        if (reply) dbus_message_unref(reply);
        return 0;
    }

    DBusMessageIter args, variant;
    dbus_bool_t val = 0;
    if (dbus_message_iter_init(reply, &args) &&
        dbus_message_iter_get_arg_type(&args) == DBUS_TYPE_VARIANT) {
        dbus_message_iter_recurse(&args, &variant);
        if (dbus_message_iter_get_arg_type(&variant) == DBUS_TYPE_BOOLEAN)
            dbus_message_iter_get_basic(&variant, &val);
    }
    dbus_message_unref(reply);
    return val ? 1 : 0;
}

/* 等待属性变为期望值，超时返回 -1 */
static int wait_property(const char *obj_path, const char *iface, const char *prop,
                         dbus_bool_t expected, int timeout_ms)
{
    for (int i = 0; i < timeout_ms / 100; i++) {
        if (get_property_bool(obj_path, iface, prop) == (expected ? 1 : 0))
            return 0;
        usleep(100000);
    }
    return -1;
}

/* 通过 GetManagedObjects 查找指定 UUID 的 characteristic 路径 */
static int find_characteristic_path(const char *uuid, char *out_path, size_t out_len)
{
    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
    if (!msg) return -1;

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) dbus_error_free(&err);
        if (reply) dbus_message_unref(reply);
        return -1;
    }

    DBusMessageIter array_iter, dict_iter;
    dbus_message_iter_init(reply, &array_iter);

    if (dbus_message_iter_get_arg_type(&array_iter) != DBUS_TYPE_ARRAY) {
        dbus_message_unref(reply);
        return -1;
    }

    int found = 0;
    dbus_message_iter_recurse(&array_iter, &dict_iter);
    while (dbus_message_iter_get_arg_type(&dict_iter) == DBUS_TYPE_DICT_ENTRY) {
        DBusMessageIter entry_iter, iface_array;
        char *obj_path = NULL;

        dbus_message_iter_recurse(&dict_iter, &entry_iter);
        if (dbus_message_iter_get_arg_type(&entry_iter) == DBUS_TYPE_OBJECT_PATH)
            dbus_message_iter_get_basic(&entry_iter, &obj_path);

        dbus_message_iter_next(&entry_iter);
        if (dbus_message_iter_get_arg_type(&entry_iter) == DBUS_TYPE_ARRAY) {
            dbus_message_iter_recurse(&entry_iter, &iface_array);
            while (dbus_message_iter_get_arg_type(&iface_array) == DBUS_TYPE_DICT_ENTRY) {
                DBusMessageIter iface_entry, prop_array;
                char *iface_name = NULL;

                dbus_message_iter_recurse(&iface_array, &iface_entry);
                if (dbus_message_iter_get_arg_type(&iface_entry) == DBUS_TYPE_STRING)
                    dbus_message_iter_get_basic(&iface_entry, &iface_name);

                if (iface_name && strcmp(iface_name, "org.bluez.GattCharacteristic1") == 0) {
                    dbus_message_iter_next(&iface_entry);
                    if (dbus_message_iter_get_arg_type(&iface_entry) == DBUS_TYPE_ARRAY) {
                        dbus_message_iter_recurse(&iface_entry, &prop_array);
                        while (dbus_message_iter_get_arg_type(&prop_array) == DBUS_TYPE_DICT_ENTRY) {
                            DBusMessageIter prop_entry, prop_val;
                            char *prop_name = NULL;

                            dbus_message_iter_recurse(&prop_array, &prop_entry);
                            if (dbus_message_iter_get_arg_type(&prop_entry) == DBUS_TYPE_STRING)
                                dbus_message_iter_get_basic(&prop_entry, &prop_name);

                            if (prop_name && strcmp(prop_name, "UUID") == 0) {
                                dbus_message_iter_next(&prop_entry);
                                if (dbus_message_iter_get_arg_type(&prop_entry) == DBUS_TYPE_VARIANT) {
                                    dbus_message_iter_recurse(&prop_entry, &prop_val);
                                    if (dbus_message_iter_get_arg_type(&prop_val) == DBUS_TYPE_STRING) {
                                        char *chr_uuid = NULL;
                                        dbus_message_iter_get_basic(&prop_val, &chr_uuid);
                                        if (chr_uuid && str_ieq(chr_uuid, uuid)) {
                                            if (obj_path) {
                                                strncpy(out_path, obj_path, out_len - 1);
                                                out_path[out_len - 1] = '\0';
                                                found = 1;
                                            }
                                        }
                                    }
                                }
                            }
                            dbus_message_iter_next(&prop_array);
                        }
                    }
                }
                dbus_message_iter_next(&iface_array);
            }
        }
        dbus_message_iter_next(&dict_iter);
    }

    dbus_message_unref(reply);
    return found ? 0 : -1;
}

/* ============================================================
 * BLE GATT 操作
 * ============================================================ */

/* 检查设备是否已存在于 BlueZ D-Bus 对象树中 */
static int device_exists(const char *device_path)
{
    DBusError err;
    dbus_error_init(&err);
    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
    if (!msg) return 0;
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) dbus_error_free(&err);
        if (reply) dbus_message_unref(reply);
        return 0;
    }

    DBusMessageIter array_iter, dict_iter;
    dbus_message_iter_init(reply, &array_iter);
    if (dbus_message_iter_get_arg_type(&array_iter) != DBUS_TYPE_ARRAY) {
        dbus_message_unref(reply);
        return 0;
    }

    int found = 0;
    dbus_message_iter_recurse(&array_iter, &dict_iter);
    while (dbus_message_iter_get_arg_type(&dict_iter) == DBUS_TYPE_DICT_ENTRY) {
        DBusMessageIter entry_iter;
        char *obj_path = NULL;
        dbus_message_iter_recurse(&dict_iter, &entry_iter);
        if (dbus_message_iter_get_arg_type(&entry_iter) == DBUS_TYPE_OBJECT_PATH)
            dbus_message_iter_get_basic(&entry_iter, &obj_path);
        if (obj_path && strcmp(obj_path, device_path) == 0) {
            found = 1;
            break;
        }
        dbus_message_iter_next(&dict_iter);
    }
    dbus_message_unref(reply);
    return found;
}

static int ble_start_discovery(void)
{
    DBusError err;
    dbus_error_init(&err);
    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", "/org/bluez/hci0",
        "org.bluez.Adapter1", "StartDiscovery");
    if (!msg) return -1;
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) dbus_error_free(&err);
        if (reply) dbus_message_unref(reply);
        return -1;
    }
    if (reply) dbus_message_unref(reply);
    return 0;
}

static int ble_stop_discovery(void)
{
    DBusError err;
    dbus_error_init(&err);
    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", "/org/bluez/hci0",
        "org.bluez.Adapter1", "StopDiscovery");
    if (!msg) return -1;
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) dbus_error_free(&err);
        if (reply) dbus_message_unref(reply);
        return -1;
    }
    if (reply) dbus_message_unref(reply);
    return 0;
}

static int ble_connect_device(void)
{
    DBusError err;
    dbus_error_init(&err);

    /* 构建设备路径 */
    snprintf(g_device_path, sizeof(g_device_path),
             "/org/bluez/hci0/dev_%s", BT24_MAC);
    for (char *p = g_device_path; *p; p++) {
        if (*p == ':') *p = '_';
    }

    printf("[BT] Connecting to %s (%s)...\n", BT24_NAME, BT24_MAC);

    /* 如果设备不在 BlueZ 缓存中，先扫描 */
    if (!device_exists(g_device_path)) {
        printf("[BT] Device not in cache, starting discovery...\n");
        if (ble_start_discovery() != 0) {
            fprintf(stderr, "[BT] StartDiscovery failed\n");
            return -1;
        }
        int found = 0;
        for (int i = 0; i < 150; i++) {
            if (device_exists(g_device_path)) {
                found = 1;
                break;
            }
            usleep(100000);
        }
        ble_stop_discovery();
        if (!found) {
            fprintf(stderr, "[BT] Device not found during discovery\n");
            return -1;
        }
        printf("[BT] Device found in cache\n");
    }

    /* 调用 Device.Connect() */
    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", g_device_path,
        "org.bluez.Device1", "Connect");
    if (!msg) return -1;

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 15000, &err);
    dbus_message_unref(msg);
    if (!reply) {
        if (dbus_error_is_set(&err)) {
            fprintf(stderr, "[BT] Connect failed: %s\n", err.message);
            dbus_error_free(&err);
        } else {
            fprintf(stderr, "[BT] Connect timeout\n");
        }
        return -1;
    }
    dbus_message_unref(reply);

    /* 等待 Connected = true */
    if (wait_property(g_device_path, "org.bluez.Device1", "Connected", TRUE, 5000) != 0) {
        fprintf(stderr, "[BT] Wait Connected timeout\n");
        return -1;
    }
    printf("[BT] Connected to %s\n", BT24_NAME);

    /* 等待 ServicesResolved = true */
    if (wait_property(g_device_path, "org.bluez.Device1", "ServicesResolved", TRUE, 8000) != 0) {
        fprintf(stderr, "[BT] Wait ServicesResolved timeout\n");
        return -1;
    }
    printf("[BT] Services resolved\n");
    usleep(500000);  /* 给 BlueZ 一点时间缓存 GATT 对象 */

    /* 查找 ffe1 characteristic */
    if (find_characteristic_path(BT24_CHR_UUID, g_char_path, sizeof(g_char_path)) != 0) {
        fprintf(stderr, "[BT] ffe1 characteristic not found\n");
        return -1;
    }
    printf("[BT] Found characteristic: %s\n", g_char_path);

    return 0;
}

/* 启动 ffe1 的 Notify */
static int ble_start_notify(void)
{
    if (strlen(g_char_path) == 0) return -1;

    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", g_char_path,
        "org.bluez.GattCharacteristic1", "StartNotify");
    if (!msg) return -1;

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) {
            fprintf(stderr, "[BT] StartNotify failed: %s\n", err.message);
            dbus_error_free(&err);
        }
        if (reply) dbus_message_unref(reply);
        return -1;
    }
    if (reply) dbus_message_unref(reply);
    printf("[BT] StartNotify success\n");
    return 0;
}

/* 停止 ffe1 的 Notify */
static int ble_stop_notify(void)
{
    if (strlen(g_char_path) == 0) return -1;

    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", g_char_path,
        "org.bluez.GattCharacteristic1", "StopNotify");
    if (!msg) return -1;

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) dbus_error_free(&err);
        if (reply) dbus_message_unref(reply);
        return -1;
    }
    if (reply) dbus_message_unref(reply);
    return 0;
}

static void ble_disconnect(void)
{
    if (strlen(g_device_path) == 0) return;

    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", g_device_path,
        "org.bluez.Device1", "Disconnect");
    if (!msg) return;

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (reply) dbus_message_unref(reply);
    if (dbus_error_is_set(&err)) dbus_error_free(&err);

    g_char_path[0] = '\0';
}

static int ble_is_connected(void)
{
    if (strlen(g_device_path) == 0) return 0;
    return get_property_bool(g_device_path, "org.bluez.Device1", "Connected");
}

/* ReadValue from ffe1 characteristic */
static int ble_read_data(uint8_t *buf, int max_len)
{
    if (strlen(g_char_path) == 0) return -1;

    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", g_char_path,
        "org.bluez.GattCharacteristic1", "ReadValue");
    if (!msg) return -1;

    /* 参数: a{sv} (空字典) */
    DBusMessageIter args, dict;
    dbus_message_iter_init_append(msg, &args);
    dbus_message_iter_open_container(&args, DBUS_TYPE_ARRAY, "{sv}", &dict);
    dbus_message_iter_close_container(&args, &dict);

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) {
            /* 静默处理常见错误，不每次都打印 */
            if (strcmp(err.name, "org.bluez.Error.NotConnected") != 0 &&
                strcmp(err.name, "org.bluez.Error.InProgress") != 0) {
                fprintf(stderr, "[BT] ReadValue failed: %s\n", err.message);
            }
            dbus_error_free(&err);
        }
        if (reply) dbus_message_unref(reply);
        return -1;
    }

    /* 解析返回值: ay */
    DBusMessageIter arr_iter, byte_iter;
    int n = 0;
    if (dbus_message_iter_init(reply, &arr_iter) &&
        dbus_message_iter_get_arg_type(&arr_iter) == DBUS_TYPE_ARRAY) {
        dbus_message_iter_recurse(&arr_iter, &byte_iter);
        while (dbus_message_iter_get_arg_type(&byte_iter) == DBUS_TYPE_BYTE && n < max_len) {
            dbus_message_iter_get_basic(&byte_iter, &buf[n++]);
            dbus_message_iter_next(&byte_iter);
        }
    }
    dbus_message_unref(reply);
    return n;
}

/* WriteValue to ffe1 characteristic */
static int ble_write_data(const uint8_t *data, int len)
{
    if (strlen(g_char_path) == 0 || len <= 0) return -1;

    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", g_char_path,
        "org.bluez.GattCharacteristic1", "WriteValue");
    if (!msg) return -1;

    /* 参数: (aya{sv}) */
    DBusMessageIter args, arr_iter, dict_iter;
    dbus_message_iter_init_append(msg, &args);

    /* ay: 字节数组 */
    dbus_message_iter_open_container(&args, DBUS_TYPE_ARRAY, "y", &arr_iter);
    for (int i = 0; i < len; i++) {
        dbus_message_iter_append_basic(&arr_iter, DBUS_TYPE_BYTE, &data[i]);
    }
    dbus_message_iter_close_container(&args, &arr_iter);

    /* a{sv}: 空字典 */
    dbus_message_iter_open_container(&args, DBUS_TYPE_ARRAY, "{sv}", &dict_iter);
    dbus_message_iter_close_container(&args, &dict_iter);

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(g_dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);
    if (!reply || dbus_error_is_set(&err)) {
        if (dbus_error_is_set(&err)) {
            fprintf(stderr, "[BT] WriteValue failed: %s\n", err.message);
            dbus_error_free(&err);
        }
        if (reply) dbus_message_unref(reply);
        return -1;
    }
    dbus_message_unref(reply);
    return 0;
}

/* ============================================================
 * 线程函数
 * ============================================================ */
static void *client_thread_func(void *arg)
{
    uint8_t buf[BT_BUF_SIZE];
    int first_connect = 1;
    int check_counter = 0;
    (void)arg;

    while (g_bt_running) {
        if (!ble_is_connected()) {
            g_notify_enabled = 0;
            if (ble_connect_device() != 0) {
                printf("[BT] Connection failed, retry in 5s...\n");
                sleep(5);
                continue;
            }

            /* 首次连接成功，通知主线程 */
            if (first_connect) {
                pthread_mutex_lock(&g_conn_mutex);
                g_bt_connected = 1;
                pthread_cond_broadcast(&g_conn_cond);
                pthread_mutex_unlock(&g_conn_mutex);
                first_connect = 0;
                printf("[BT] ===== First connection established, proceeding =====\n");
            } else {
                printf("[BT] ===== Reconnected to %s =====\n", BT24_NAME);
            }

            /* 尝试启动 Notify */
            if (ble_start_notify() == 0) {
                g_notify_enabled = 1;
            } else {
                g_notify_enabled = 0;
                printf("[BT] Notify not available, using ReadValue polling\n");
            }
        }

        if (g_notify_enabled) {
            /* Notify 模式：等待 D-Bus 信号（阻塞最多 500ms） */
            if (g_dbus_conn) {
                dbus_connection_read_write_dispatch(g_dbus_conn, 500);
            }

            /* 定期检查连接状态（每 ~5 秒） */
            check_counter++;
            if (check_counter >= 10) {
                check_counter = 0;
                if (!ble_is_connected()) {
                    printf("[BT] Connection lost\n");
                    ble_stop_notify();
                    ble_disconnect();
                    g_notify_enabled = 0;
                }
            }
        } else {
            /* 轮询模式：ReadValue */
            int bytes = ble_read_data(buf, sizeof(buf));
            if (bytes > 0) {
                // printf("[BT] ReadValue: %d bytes", bytes);
                // for (int i = 0; i < bytes && i < 8; i++)
                //     printf(" %02X", buf[i]);
                // if (bytes > 8) printf(" ...");
                // printf("\n");
                handle_raw_data((char *)buf, bytes);
            } else if (bytes < 0) {
                if (!ble_is_connected()) {
                    printf("[BT] Connection lost\n");
                    ble_disconnect();
                }
            }
            usleep(100000);
        }
    }

    if (g_notify_enabled) ble_stop_notify();
    ble_disconnect();
    return NULL;
}

/* ============================================================
 * 对外接口
 * ============================================================ */
int bluetooth_spp_init(void)
{
    printf("\n========== Bluetooth BLE Client Init ==========\n");
    printf("[BT] Unblocking rfkill...\n");
    run_shell_cmd("rfkill unblock bluetooth 2>/dev/null");
    usleep(200000);
    printf("[BT] Bringing up hci0...\n");
    run_shell_cmd("hciconfig hci0 up 2>/dev/null");
    usleep(200000);
    run_shell_cmd("hciconfig hci0 name 'ELF2-AI-Camera' 2>/dev/null");
    usleep(100000);

    FILE *fp = popen("hciconfig hci0 | grep 'BD Address' | awk '{print $3}'", "r");
    if (fp) {
        char addr[32] = {0};
        if (fgets(addr, sizeof(addr), fp)) {
            str_trim(addr);
            if (strlen(addr) > 0)
                printf("[BT] Local BD Address: %s\n", addr);
        }
        pclose(fp);
    }

    if (dbus_init() != 0) {
        fprintf(stderr, "[BT] D-Bus init failed\n");
        return -1;
    }

    printf("[BT] Init complete. Target: %s (%s)\n", BT24_NAME, BT24_MAC);
    printf("================================================\n\n");
    return 0;
}

int bluetooth_spp_start(void)
{
    if (g_bt_running) {
        printf("[BT] Already running\n");
        return 0;
    }

    g_bt_running = 1;
    g_bt_connected = 0;
    g_device_path[0] = '\0';
    g_char_path[0] = '\0';

    if (pthread_create(&g_client_thread, NULL, client_thread_func, NULL) != 0) {
        perror("[BT] pthread_create client_thread failed");
        g_bt_running = 0;
        return -1;
    }

    printf("[BT] Client thread started; connecting to %s in background\n", BT24_NAME);
    return 0;
}

void bluetooth_spp_stop(void)
{
    if (!g_bt_running) return;
    printf("[BT] Stopping...\n");
    g_bt_running = 0;

    /* 唤醒可能在等待的主线程 */
    pthread_mutex_lock(&g_conn_mutex);
    pthread_cond_broadcast(&g_conn_cond);
    pthread_mutex_unlock(&g_conn_mutex);

    pthread_join(g_client_thread, NULL);
    if (g_notify_enabled) ble_stop_notify();
    ble_disconnect();

    if (g_dbus_conn) {
        dbus_connection_remove_filter(g_dbus_conn, notify_filter, NULL);
        dbus_bus_remove_match(g_dbus_conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',member='PropertiesChanged'",
            NULL);
        dbus_connection_unref(g_dbus_conn);
        g_dbus_conn = NULL;
    }

    printf("[BT] Stopped\n");
}

void bluetooth_spp_cleanup(void)
{
    bluetooth_spp_stop();
    run_shell_cmd("hciconfig hci0 noscan 2>/dev/null");
    printf("[BT] Cleanup complete\n");
}
