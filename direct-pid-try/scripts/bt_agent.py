#!/usr/bin/env python3
"""最小化配对测试脚本：只注册 Agent，自动返回 PIN=0000，自动确认所有配对请求"""
import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

AGENT_PATH = "/org/bluez/auto_agent"

def set_trusted(path):
    try:
        props = dbus.Interface(
            bus.get_object("org.bluez", path),
            "org.freedesktop.DBus.Properties"
        )
        props.Set("org.bluez.Device1", "Trusted", True)
    except Exception as e:
        print(f"[WARN] set_trusted failed: {e}")

class Agent(dbus.service.Object):
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

DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()
agent = Agent(bus, AGENT_PATH)

agent_manager = dbus.Interface(
    bus.get_object("org.bluez", "/org/bluez"),
    "org.bluez.AgentManager1"
)
agent_manager.RegisterAgent(AGENT_PATH, "DisplayYesNo")
agent_manager.RequestDefaultAgent(AGENT_PATH)

print("[+] Agent registered. Ready for pairing.")
print("[+] Go ahead and pair your phone with 'ELF2-AI-Camera'")
print("[+] If it asks for PIN, enter: 0000")

try:
    GLib.MainLoop().run()
except KeyboardInterrupt:
    print("\n[-] Exiting...")
    agent_manager.UnregisterAgent(AGENT_PATH)
