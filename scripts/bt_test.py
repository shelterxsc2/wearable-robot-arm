#!/usr/bin/env python3
"""
最小化蓝牙测试脚本：自动配对 Agent + SPP Profile
不依赖 cc 程序，纯命令行测试蓝牙 SPP 是否可连接
"""
import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
import os

PROFILE_PATH = "/org/bluez/spp_test"
SPP_UUID = "00001101-0000-1000-8000-00805f9b34fb"
AGENT_PATH = "/org/bluez/auto_agent"

def set_trusted(path):
    props = dbus.Interface(bus.get_object("org.bluez", path), "org.freedesktop.DBus.Properties")
    props.Set("org.bluez.Device1", "Trusted", True)

class Agent(dbus.service.Object):
    """自动配对 Agent：Legacy PIN 返回 0000，SSP 自动确认"""

    def __init__(self, bus, path):
        dbus.service.Object.__init__(self, bus, path)

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        print(f"[AGENT] RequestPinCode from {device} -> returning '0000'")
        set_trusted(device)
        return "0000"

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        print(f"[AGENT] RequestPasskey from {device} -> returning 0")
        set_trusted(device)
        return dbus.UInt32(0)

    @dbus.service.method("org.bluez.Agent1", in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        print(f"[AGENT] RequestConfirmation from {device}, passkey={passkey} -> auto confirm")
        set_trusted(device)

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        print(f"[AGENT] RequestAuthorization from {device} -> auto authorize")
        set_trusted(device)

    @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        print(f"[AGENT] AuthorizeService from {device}, uuid={uuid} -> auto authorize")
        set_trusted(device)

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Cancel(self):
        print("[AGENT] Cancel")

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Release(self):
        print("[AGENT] Release")

class Profile(dbus.service.Object):
    """SPP Profile：收到连接后打印收到的数据"""

    def __init__(self, bus, path):
        dbus.service.Object.__init__(self, bus, path)

    @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
    def NewConnection(self, device, fd, properties):
        print(f"\n[SPP] ===== NewConnection from {device}, fd={fd} =====")
        try:
            while True:
                data = os.read(fd, 256)
                if not data:
                    break
                hex_str = data.hex()
                text = data.decode('utf-8', errors='replace').strip()
                print(f"[SPP] Received {len(data)} bytes: hex=[{hex_str}] text='{text}'")
                os.write(fd, b"OK: received\n")
        except Exception as e:
            print(f"[SPP] Connection error: {e}")
        finally:
            os.close(fd)
        print("[SPP] ===== Connection closed =====\n")

    @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
    def RequestDisconnection(self, device):
        print(f"[SPP] RequestDisconnection from {device}")

    @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
    def Release(self):
        print("[SPP] Release")

DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()

# 注册 Agent
agent = Agent(bus, AGENT_PATH)
agent_manager = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.AgentManager1")
agent_manager.RegisterAgent(AGENT_PATH, "DisplayYesNo")
agent_manager.RequestDefaultAgent(AGENT_PATH)
print("[+] Auto-pairing agent registered (PIN=0000, auto-confirm)")

# 注册 SPP Profile
profile = Profile(bus, PROFILE_PATH)
profile_manager = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"), "org.bluez.ProfileManager1")
profile_manager.RegisterProfile(PROFILE_PATH, SPP_UUID, {
    "Channel": dbus.Byte(1),
    "Name": "Serial Port"
})
print("[+] SPP Profile registered")
print("[+] Ready. Now pair your phone with 'ELF2-AI-Camera' and connect SPP.\n")

try:
    GLib.MainLoop().run()
except KeyboardInterrupt:
    print("\n[-] Exiting...")
    profile_manager.UnregisterProfile(PROFILE_PATH)
    agent_manager.UnregisterAgent(AGENT_PATH)
