# AGENTS.md

This is a west workspace root for LR2021 LoRa Plus EVK firmware: XIAO nRF54L15 +
Wio-LR2021 + SSD1306 expansion board, Zephyr v4.4.0, Semtech USP v1.2.1. Projects,
setup, build and flash commands are in `README.md` and each project's README.

## Working here

- Start every shell with `source .venv/bin/activate`, from the repo root.
- Build with `west build … <project> -d <project>/build_<role>`, keeping build dirs inside the project.
- `zephyr/`, `modules/`, `usp_zephyr/`, `bootloader/` and `tools/` are west checkouts. They are read-only reference: to change a Semtech board definition, add an overlay in the project's `boards/`.
- Flash with `pyocd flash -t nrf54l -u <unique ID> <hex>`. `west flash` fails because the Zephyr board file for this board has no pyocd runner. `pyocd list` shows the unique IDs.
- Read results from the board's USB serial at 115200 baud with pyserial. A board prints a stats line every second.

## Bench hardware

| pyOCD ID | Serial | Note |
|---|---|---|
| `3EF60302` | `/dev/cu.usbmodem3EF603023` | OLED at the shield default, 0x3C |
| `7110B0C1` | `/dev/cu.usbmodem7110B0C13` | OLED strapped to 0x3D; always build for it with `-DEXTRA_DTC_OVERLAY_FILE=boards/oled_addr_0x3d.overlay` |

- Both boards have sub-GHz and 2.4 GHz antennas fitted.
- Transmit only at 2.4 GHz unless the user asks otherwise; the sub-GHz antenna is a CN490 part.
- The 2.4 GHz PA range is -17 to +12 dBm.

## Radio gotchas

- The USP RAC scheduler works in whole milliseconds, so each packet costs about 3 ms of overhead. For high-throughput FLRC, take the radio with `smtc_rac_immediate_radio_access()` and drive `ral_lr20xx_*` directly, as `flrc_pingpong/src/app_flrc.c` does.
- RAC and radio callbacks run in the USP thread. They hand data to the display thread and never call LVGL or I2C.
- FLRC RX setup writes all three sync words, so all three pointers must be valid.
- The RX buffer must be at least `max_rx_size`; a smaller buffer panics on submit.

## Git

- Leave committing and pushing to the user.
- Before a commit, confirm the author email with the user, who switches between work and personal identities.
