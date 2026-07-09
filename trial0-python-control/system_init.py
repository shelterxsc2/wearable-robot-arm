# -*- coding: utf-8 -*-
"""System-level initialization helpers: WiFi and Bluetooth HCI.

These mirror the early boot steps in Base src/main.cpp (wifi.cpp and
bluetooth_spp.c) but use common Linux userspace tools so they work on the
Intel/OpenVINO target platform without porting the C code.
"""
import json
import os
import re
import subprocess
import tempfile
import threading
import time
from typing import Optional


LAST_WIFI_FILE = os.path.expanduser("~/.config/elf_wifi/last.json")
DEFAULT_WIFI_SSID = "iQOO 12"
DEFAULT_WIFI_PASSWD = "070103xsc"


def _run(cmd: list[str], timeout: float = 10.0, check: bool = False) -> subprocess.CompletedProcess:
    """Run a command, swallowing errors unless check=True."""
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=check
        )
    except Exception as e:
        return subprocess.CompletedProcess(cmd, returncode=-1, stdout="", stderr=str(e))


def _has_ip(iface: str) -> bool:
    """Return True if the interface already has an IPv4 address."""
    try:
        out = subprocess.check_output(["ip", "addr", "show", iface], text=True)
        return bool(re.search(r"inet\s+\d+\.\d+\.\d+\.\d+", out))
    except Exception:
        return False


def _current_ssid(iface: str) -> Optional[str]:
    """Return the SSID currently associated with the interface, if known."""
    if not _nmcli_available():
        return None
    r = _run(
        ["nmcli", "-t", "-f", "ACTIVE,SSID", "device", "wifi", "list", "ifname", iface],
        timeout=5.0,
    )
    for line in r.stdout.strip().splitlines():
        if line.startswith("yes:"):
            return line.split(":", 1)[1]
    return None


def _nmcli_available() -> bool:
    return _run(["which", "nmcli"], timeout=2.0).returncode == 0


def _wait_for_ip(iface: str, timeout: float = 30.0) -> bool:
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if _has_ip(iface):
            return True
        time.sleep(0.5)
    return False


def load_last_wifi() -> Optional[dict]:
    """Load the last successfully connected WiFi credentials."""
    try:
        with open(LAST_WIFI_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return None


def save_last_wifi(ssid: str, passwd: str, iface: str) -> None:
    """Persist WiFi credentials for the next boot."""
    os.makedirs(os.path.dirname(LAST_WIFI_FILE), exist_ok=True)
    try:
        with open(LAST_WIFI_FILE, "w") as f:
            json.dump({"ssid": ssid, "passwd": passwd, "iface": iface}, f)
    except Exception as e:
        print(f"[WiFi] Failed to save credentials: {e}")


def _try_one_network(
    ssid: str,
    passwd: str,
    iface: str,
    timeout: float,
    static_fallback: bool,
) -> bool:
    """Attempt to connect to a single SSID. Return True on success."""
    print(f"[WiFi] Trying '{ssid}' on {iface} ...")
    _run(["ip", "link", "set", iface, "up"], timeout=5.0)

    if _nmcli_available():
        # Disconnect any stale connection first.
        _run(["nmcli", "device", "disconnect", iface], timeout=10.0)
        r = _run(
            ["nmcli", "device", "wifi", "connect", ssid,
             "password", passwd, "ifname", iface],
            timeout=timeout,
        )
        if r.returncode == 0 and _wait_for_ip(iface, timeout=timeout):
            print(f"[WiFi] Connected to '{ssid}' via NetworkManager")
            return True
        else:
            print(f"[WiFi] nmcli failed for '{ssid}': {r.stderr.strip()}")

    print(f"[WiFi] Trying '{ssid}' via wpa_supplicant + dhclient ...")
    if _init_wifi_wpa(ssid, passwd, iface, timeout=timeout):
        print(f"[WiFi] Connected to '{ssid}' via wpa_supplicant")
        return True

    if static_fallback:
        print("[WiFi] DHCP failed, trying static IP fallback ...")
        if _static_ip_fallback(iface):
            return True

    return False


def init_wifi(
    ssid: Optional[str] = None,
    passwd: Optional[str] = None,
    iface: str = "wlan0",
    timeout: float = 30.0,
    static_fallback: bool = True,
    fallback_list: Optional[list[tuple[str, str]]] = None,
) -> bool:
    """Bring up WiFi and connect.

    If ssid/passwd are not provided, the last successfully connected network
    is tried first. If that fails (or does not exist), each network in
    ``fallback_list`` is attempted in order. Finally the built-in default
    SSID is used as the last resort.

    Tries NetworkManager first for each candidate, then falls back to
    wpa_supplicant + dhclient. If static_fallback is True, assigns
    192.168.1.100/24 or 192.168.0.100/24 when DHCP fails, matching Base
    main.cpp behaviour.
    """
    if fallback_list is None:
        fallback_list = []

    # Build candidate list: explicit > last saved > fallbacks > default.
    candidates: list[tuple[str, str]] = []
    if ssid is not None and passwd is not None:
        candidates.append((ssid, passwd))

    last = load_last_wifi()
    if last:
        candidates.append((last.get("ssid", ""), last.get("passwd", "")))
        iface = last.get("iface", iface)

    candidates.extend(fallback_list)
    candidates.append((DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASSWD))

    # Deduplicate while preserving order.
    seen: set[str] = set()
    unique_candidates: list[tuple[str, str]] = []
    for s, p in candidates:
        if s and s not in seen:
            seen.add(s)
            unique_candidates.append((s, p))

    print(f"[WiFi] Bringing up {iface} ...")
    _run(["ip", "link", "set", iface, "up"], timeout=5.0)

    if _has_ip(iface):
        print(f"[WiFi] {iface} already has an IP address")
        # If NetworkManager already brought up a known network, remember it
        # instead of overwriting with the default credentials.
        current_ssid = _current_ssid(iface)
        if current_ssid:
            last_passwd = ""
            if last and last.get("ssid") == current_ssid:
                last_passwd = last.get("passwd", "")
            save_last_wifi(current_ssid, last_passwd, iface)
        return True

    for s, p in unique_candidates:
        if _try_one_network(s, p, iface, timeout, static_fallback):
            save_last_wifi(s, p, iface)
            ip = _run(["ip", "addr", "show", iface], timeout=5.0).stdout
            print(f"[WiFi] Interface {iface} is up:\n{ip}")
            return True

    print(f"[WiFi] Failed to configure {iface}")
    return False


def _init_wifi_wpa(ssid: str, passwd: str, iface: str, timeout: float = 30.0) -> bool:
    """wpa_supplicant + dhclient fallback."""
    # Clean up stale processes.
    _run(["pkill", "-f", f"wpa_supplicant.*{iface}"], timeout=2.0)
    _run(["pkill", "-f", f"dhclient.*{iface}"], timeout=2.0)
    _run(["pkill", "-f", f"udhcpc.*{iface}"], timeout=2.0)
    time.sleep(0.2)

    conf = f"""ctrl_interface=/var/run/wpa_supplicant
update_config=1
network={{
    ssid="{ssid}"
    psk="{passwd}"
    key_mgmt=WPA-PSK
}}
"""
    fd, conf_path = tempfile.mkstemp(prefix="wpa_supplicant_", suffix=".conf")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(conf)
        r = _run(
            ["wpa_supplicant", "-B", "-i", iface, "-c", conf_path],
            timeout=5.0,
        )
        if r.returncode != 0:
            print(f"[WiFi] wpa_supplicant failed: {r.stderr.strip()}")
            return False

        # Wait for association.
        assoc_deadline = time.perf_counter() + timeout
        while time.perf_counter() < assoc_deadline:
            r = _run(["wpa_cli", "-i", iface, "status"], timeout=2.0)
            if "wpa_state=COMPLETED" in r.stdout:
                break
            time.sleep(0.5)
        else:
            print("[WiFi] wpa_supplicant association timeout")
            return False

        # DHCP.
        for dhcp_client in (["dhclient", "-v", iface], ["udhcpc", "-i", iface, "-t", "10", "-T", "3", "-n", "-q"]):
            r = _run(dhcp_client, timeout=timeout)
            if _wait_for_ip(iface, timeout=10.0):
                return True
        return False
    finally:
        try:
            os.unlink(conf_path)
        except OSError:
            pass


def _static_ip_fallback(iface: str) -> bool:
    """Try static IPs matching Base main.cpp fallback."""
    for ip, gw in [("192.168.1.100/24", "192.168.1.1"), ("192.168.0.100/24", "192.168.0.1")]:
        _run(["ip", "addr", "flush", "dev", iface], timeout=2.0)
        r1 = _run(["ip", "addr", "add", ip, "dev", iface], timeout=2.0)
        r2 = _run(["ip", "route", "add", "default", "via", gw], timeout=2.0)
        if r1.returncode == 0:
            try:
                with open("/etc/resolv.conf", "a") as f:
                    f.write("nameserver 114.114.114.114\n")
            except Exception as e:
                print(f"[WiFi] failed to write DNS: {e}")
            print(f"[WiFi] Static fallback set {ip} gateway {gw}")
            return True
    return False


def init_bluetooth_hci(iface: str = "hci0", name: str = "ELF2-AI-Camera") -> bool:
    """Unblock Bluetooth and bring up the HCI interface."""
    print(f"[BT] Initializing {iface} ...")
    _run(["rfkill", "unblock", "bluetooth"], timeout=5.0)
    status = _run(["hciconfig", iface], timeout=5.0)
    if status.returncode == 0 and "UP RUNNING" in status.stdout:
        print(f"[BT] {iface} is already up")
    else:
        up = _run(["hciconfig", iface, "up"], timeout=5.0)
        if up.returncode != 0:
            print(f"[BT] Failed to bring up {iface}: {up.stderr.strip()}")
            return False

    # Renaming requires CAP_NET_ADMIN on many systems and is unnecessary when
    # this host only acts as a BLE client.
    rename = _run(["hciconfig", iface, "name", name], timeout=5.0)
    if rename.returncode != 0:
        print(f"[BT] {iface} is up; local-name change skipped: {rename.stderr.strip()}")
    else:
        print(f"[BT] {iface} is up as '{name}'")
    return True


def start_wifi_thread(
    ssid: Optional[str] = None,
    passwd: Optional[str] = None,
    iface: str = "wlan0",
    timeout: float = 30.0,
) -> threading.Thread:
    """Start WiFi initialization in a background thread."""
    t = threading.Thread(
        target=init_wifi,
        args=(ssid, passwd, iface, timeout),
        daemon=True,
    )
    t.start()
    return t


def start_ble_remote(
    mac: str = "F8:2E:0C:E3:99:C8",
    ctrl_url: str = "http://127.0.0.1:8080",
) -> Optional["BleRemoteListener"]:
    """Start the HC-08 BLE remote listener if bleak is available."""
    try:
        from tools.ble_remote import BleRemoteListener
    except Exception as e:
        print(f"[BLE-Remote] import failed: {e}")
        return None
    listener = BleRemoteListener(mac=mac, ctrl_url=ctrl_url)
    listener.start()
    return listener
