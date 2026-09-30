# FLRC 2.4 GHz throughput demo (LR2021 LoRa Plus EVK)

One-way FLRC link between two boards: XIAO nRF54L15 + Wio-LR2021 on the LoRa Plus
Expansion Board (SSD1306 OLED). The TX board streams packets back to back. The RX
board shows goodput and packet success rate once a second.

The directory is still called `flrc_pingpong`, but the app is one-way.

Measured on the bench (boards about 1 m apart, RSSI about -40 dBm):

- TX: 526 packets/s, 2.15 Mbps payload.
- RX: 2.06–2.12 Mbps goodput, 96–98 % packets received.

## Radio settings

- 2440 MHz. The HF path and PA are selected automatically above 1.5 GHz. The sub-GHz path is never used.
- 2.6 Mbps raw bit rate, CR 1/1 (no FEC).
- BT 0.5 pulse shape, 32-bit preamble, 4-byte sync word.
- Variable length, 511-byte payload, 2-byte CRC.
- TX power +5 dBm.

Payload: a 4-byte little-endian sequence number, then filler bytes.
You can change these settings in `Kconfig` (`CONFIG_APP_*`).

After start-up, the app takes the radio from the RAC scheduler with
`smtc_rac_immediate_radio_access()` and drives the LR20xx directly:

- The radio is configured once.
- TX: on each TX_DONE, the app calls `SetTx` at once. The next payload is already in
  the 1024-byte TX FIFO, so the app then writes the payload after it.
- RX: continuous RX. The radio stays in RX, and the app reads the FIFO on each RX_DONE.

The per-packet RAC transaction path cost about 3 ms per packet, which limited goodput to
0.84 Mbps (TX) and 0.5 Mbps (RX).

Fit a 2.4 GHz antenna on the Wio-LR2021 HF port on both boards.

## Bench boards

| Role | pyOCD unique ID | Serial port | OLED |
|---|---|---|---|
| RX | `3EF60302` | `/dev/cu.usbmodem3EF603023` | works |
| TX | `7110B0C1` | `/dev/cu.usbmodem7110B0C13` | works at 0x3D (see below) |

The OLED on `7110B0C1` does not answer at I2C address 0x3C (the SSD1306 driver reports
-EIO). At boot the app probes the bus, and a device ACKs at 0x3D. That module is strapped to
the other address, which is why it was also dark with the factory firmware. Build for
that board with `boards/oled_addr_0x3d.overlay` (see Build). The overlay is tied to the
board, not the role: add it whenever `7110B0C1` is flashed, whether as TX or RX.

## Build

From the west workspace root (`lr2021/`):

```bash
source .venv/bin/activate

# Transmitter (default role)
west build -p -b xiao_nrf54l15/nrf54l15/cpuapp \
  --shield semtech_loraplus_expansion_board --shield semtech_wio_lr2021 \
  flrc_pingpong -d flrc_pingpong/build_tx \
  -- -DEXTRA_DTC_OVERLAY_FILE=boards/oled_addr_0x3d.overlay   # 7110B0C1's OLED is at 0x3D

# Receiver
west build -p -b xiao_nrf54l15/nrf54l15/cpuapp \
  --shield semtech_loraplus_expansion_board --shield semtech_wio_lr2021 \
  flrc_pingpong -d flrc_pingpong/build_rx -- -DCONFIG_APP_ROLE_RX=y
```

## Flash

```bash
pyocd flash -t nrf54l -u 3EF60302 flrc_pingpong/build_rx/zephyr/zephyr.hex
pyocd flash -t nrf54l -u 7110B0C1 flrc_pingpong/build_tx/zephyr/zephyr.hex
```

Run `pyocd list` to see the connected boards. If a board reports a lock error, run
`pyocd erase -c -t nrf54l -u <ID>` first.

## Screens

TX:

```
   FLRC TX  2.4 GHz
2440 MHz  +5 dBm
2.6 Mbps raw CR1/1 511B
Pkts/s: 526
TX: 2.15 Mbps          <- payload bits sent per second
Seq: 123456  err: 0
```

RX (large font):

```
   FLRC RX  2440 MHz
      2.10 Mbps        <- goodput (good-CRC payload bits per second)
     97.9% pkts        <- good packets / packets sent in the window
RSSI -40  CRC 0  lost 11
```

- Success % is good packets divided by the sequence span seen in that 1 s window.
  CRC-failed and missed packets both appear as gaps.
- A backward jump in the sequence (TX reboot), or a jump larger than
  `CONFIG_APP_SEQ_RESYNC_GAP`, re-syncs the counter and does not count as loss.

## Console

Both roles print one line per second on the USB serial port (115200 baud), for example:

```
TX 1000 ms: 526 pkt (526 pkt/s), 2.15 Mbps payload, seq 4209, err 0, disp 0
RX 1000 ms: ok 515 / exp 526, 2.10 Mbps goodput, crc 0, tmo 0, resync 0, fifo 0, seq 3910, rssi -40, disp 135
```

`disp` counts the display refreshes. If it keeps increasing, the OLED thread is running.
It stays at 0 when the OLED driver failed to start.
