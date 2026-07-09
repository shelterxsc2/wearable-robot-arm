# Hardware firmware reference

This directory only contains third-party/binary firmware blobs (e.g. AX201 Wi-Fi
ucode).  The STM32F103 DataHub bridge firmware is **not maintained here**.

Use the upstream firmware written by the project owner:

```text
/home/time/work/stm32f103_datahub/
```

That firmware implements the unified sensor bridge used by this project.  The
active source files are under `UserApp/Src/` and `UserApp/Inc/`; `Core/Src/` and
`Core/Inc/` are older snapshot-protocol leftovers and are not compiled into the
MDK-ARM project.

See its `BRIDGE_PROTOCOL.md` for the raw forwarding description and the new
host downlink protocol (`AA 55 LEN CMD PAYLOAD CRC`).  The Python-side parser
and sender live in `elf_control_chain_stm32.py`.

## Wiring / crosstalk notes

When using two CH341 USB-to-TTL adapters for the F103 bridge (USART3 @ 460800
for host commands and USART2 @ 115200 for the H7 arm):

> **Baud-rate note:** The bridge link was originally run at 921600, but with
> unshielded DuPont wiring the bridge TX signal coupled into the arm RX line and
> corrupted framed downlink payloads.  Dropping the link to 460800 keeps the
> throughput sufficient for control while reducing the crosstalk to a usable
> level.  If you rebuild the wiring with short, separated, or shielded cables,
> you can raise the rate back to 921600 and update `DEFAULT_BRIDGE_BAUD` in
> `stm32_bridge_utils.py` accordingly.

- Keep the bridge TX/RX pair and the arm TX/RX pair as far apart as practical.
- Make sure both adapters share a solid common ground with the F103 board.
- Use short jumper wires; long unshielded DuPont cables at 460800 are prone to
capacitive coupling between adjacent wires.
- If the arm RX line picks up the bridge TX signal, framed downlink payloads
will appear shifted or duplicated.  Use
`tools/stm32_bridge_crosstalk_probe.py` to confirm: it sends a CRC-bad frame
that the F103 must discard; any bytes still seen on the arm port are physical
crosstalk rather than firmware forwarding.
- Opening the serial port with pyserial asserts DTR, which may reset the F103 if
the adapter's DTR pin is wired to NRST.  The Python helpers wait 2 s after
opening to let the F103 boot before sending traffic.
