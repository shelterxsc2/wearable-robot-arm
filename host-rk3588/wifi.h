#ifndef WIFI_H
#define WIFI_H

#include <stddef.h>   // for size_t

#ifdef __cplusplus
extern "C" {
#endif

int wifi_enable_interface(const char *ifname);
int wifi_connect_wpa(const char *ifname, const char *ssid, const char *pwd);
int wifi_dhcp(const char *ifname);
void wifi_print_ip(const char *ifname, char *ip_str, size_t ip_len);

#ifdef __cplusplus
}
#endif

#endif // WIFI_H
