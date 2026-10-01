# LR2021 as a BLE beacon / scanner: research

**Status (2026-09-30): parked.** The research is done and a prototype compiles. Nothing
has been flashed or tested over the air. The prototype sources are in
[`ble_beacon_proto/`](ble_beacon_proto/). They are not a complete app: to build them,
copy `flrc_throughput/`, then drop these files over it (see section 5).

## Verdict

Feasible. The LR20xx has a native Bluetooth LE packet engine, and USP v1.2.1 ships a driver for it. The RAL/RAC layers do not wrap it, so the app calls the `lr20xx_radio_bluetooth_le_*` functions directly after `smtc_rac_immediate_radio_access()`, as `flrc_throughput` does for FLRC. A prototype for both roles compiles. It has not been flashed or tested over the air.

## 1. Driver support (evidence)

- Packet type: `modules/lib/usp/smtc_rac_lib/radio_drivers/lr20xx_driver/inc/lr20xx_radio_common_types.h:179`
  `LR20XX_RADIO_COMMON_PKT_TYPE_BLUETOOTH_LE = 0x03,  //!< Bluetooth_LE packet engine`
- API: `.../lr20xx_driver/inc/lr20xx_radio_bluetooth_le.h`. The functions are `set_modulation_params(phy)`, `set_pkt_params()`, `set_modulation_pkt_params()`, `set_tx(ctx, len)` (sets the PDU length and starts TX), `set_pdu_length()`, `get_pkt_status()` and `get_rx_stats()`.
- Packet params (`lr20xx_radio_bluetooth_le_types.h`):
  ```c
  lr20xx_radio_bluetooth_le_phy_channel_pdu_t phy_channel_pdu;  // ADVERTISING / DATA_16_BIT_HEADER / DATA_24_BIT_HEADER
  uint8_t whitening_init;  //!< ... (for instance 0x53 for channel 37). Setting to 0 disable whitening
  bool     is_crc_in_fifo;
  uint32_t crc_init;        //!< Bluetooth_LE CRC initialization value (0x555555 for advertising)
  uint32_t sync_word;       //!< Bluetooth_LE syncword (0x8e89bed6 for advertising)
  ```
- PHYs: `LE_1M`, `LE_2M`, `LE_CODED_500KB`, `LE_CODED_125KB`.
- `set_tx` doc: "len PDU length - including 16-bit header, excluding the 3-byte CRC".
- **The RAL has no BLE support.** `ral_defs.h:586-590` has only GFSK, LORA, FLRC and RTTOF, and `ral_lr20xx_set_pkt_type` returns UNKNOWN_VALUE for any other type. The generic RAL calls still work: `ral_lr20xx_set_rf_freq` selects the HF RX path, `ral_lr20xx_set_tx_cfg` sets the HF PA from the frequency, and `set_standby`, `set_rx`, `set_dio_irq_params` and `set_rx_tx_fallback_mode` behave as usual.
- **Build gate:** `usp_zephyr/modules/usp_drivers/lr20xx.cmake` compiles `lr20xx_radio_bluetooth_le.c` only when `CONFIG_SEMTECH_LR20XX_BLE` is set (`usp_zephyr/drivers/usp/Kconfig.lr20xx:17`, default n). Without it, the link fails with undefined `lr20xx_radio_bluetooth_le_*`. Add `CONFIG_SEMTECH_LR20XX_BLE=y` to prj.conf.
- Samples: none. A grep of `modules/lib/usp/examples` and `usp_zephyr/samples` for bluetooth, `_ble_` or `0x8e89bed6` finds nothing relevant.

## 2. Valid legacy advertising: hardware vs software

| Item | Who does it |
|---|---|
| Preamble (0xAA/0x55 per AA LSB) | Hardware (BLE packet engine) |
| Access address 0x8E89BED6 | Hardware: `sync_word` |
| CRC24, init 0x555555 | Hardware: `crc_init`, `is_crc_in_fifo=false` |
| Whitening, per channel | Hardware: `whitening_init`. Software must set it for each channel |
| 1M GFSK modulation | Hardware: `LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_1M` |
| Channel hopping 37/38/39 (2402/2426/2480 MHz) | Software: `ral_lr20xx_set_rf_freq` and new pkt params before each TX |
| PDU header, AdvA, AdvData | Software: written to the TX FIFO |
| advInterval + 0-10 ms advDelay | Software timer/thread |

Whitening init: the driver gives 0x53 for channel 37. That is the BLE LFSR (position 0 = 1, positions 1-6 = channel index, MSB first) stored with position 0 in bit 0, so `init = reverse6(ch) << 1 | 1`. This gives **37 = 0x53, 38 = 0x33, 39 = 0x73**. (Nordic's convention, `0x40 | ch`, is the bit-reverse of this.) Only 0x53 appears in the source, so 38 and 39 are derived values.

PDU used (ADV_NONCONN_IND): `hdr0 = 0x42` (type 0x2, TxAdd = random), `hdr1 = len`, AdvA (6 bytes, LSB first, random static address with the top 2 bits set), then the AD structures `02 01 06`, `07 09 "LR2021"` and `07 FF FF FF <u32 counter>` (company 0xFFFF is reserved for tests). Total PDU is 2 + 25 = 27 bytes. On air at 1 Mbps it takes about 0.3 ms per channel.

Use low power: `-17..+12 dBm`. The prototype defaults to 0 dBm.

## 3. RX scanning

Feasible with the same engine. Set the channel 37 pkt params (whitening 0x53), use `ral_lr20xx_set_rx(CONTINUOUS)`, and enable the IRQs RX_DONE, CRC_ERROR and LEN_ERROR. On RX_DONE, read the FIFO (PDU: header + AdvA + data) and call `lr20xx_radio_bluetooth_le_get_pkt_status()` for `rssi_avg_in_dbm`. Then dedupe AdvA into a table (32 entries, 10 s time to live) and show the count and the best RSSI. Phones and other BLE devices advertise on 37 constantly, so it should fill fast. Unknowns: what `set_pdu_length` means for RX (the prototype assumes it is the max length and sets 255), and whether `get_rx_packet_length` / the FIFO level include the header. The prototype reads the whole FIFO level and checks it against `hdr[1] + 2`.

## 4. Implementation plan: `ble_beacon/` (modeled on flrc_throughput)

Copy `boards/`, `src/oled_display/`, `CMakeLists.txt` (project name, `src/app_ble.c`), `prj.conf` + `CONFIG_SEMTECH_LR20XX_BLE=y` + `CONFIG_HWINFO=y`, but not `west.yml` (the workspace manifest stays in `flrc_throughput/`).

- `Kconfig`: choice `APP_ROLE_BEACON` / `APP_ROLE_SCAN`, plus `APP_TX_POWER_DBM` (default 0), `APP_ADV_INTERVAL_MS` (100) and `APP_ADV_NAME` ("LR2021").
- `app_ble.c`:
  - `app_ble_init()`: `smtc_rac_immediate_radio_access(RAC_VERY_HIGH_PRIORITY, irq_callback)`, `smtc_rac_get_radio_driver_context()`, `ral_lr20xx_set_standby(XOSC)`, `lr20xx_radio_common_set_pkt_type(BLUETOOTH_LE)`, `lr20xx_radio_bluetooth_le_set_modulation_params(LE_1M)`.
  - Beacon: `ral_lr20xx_set_tx_cfg(pwr, 2426 MHz)`, `set_rx_tx_fallback_mode(STDBY_XOSC)`, `set_dio_irq_params(TX_DONE|TIMEOUT)`, then start a `ble_beacon` thread (prio 4). For each event and each channel: `ral_lr20xx_set_rf_freq`, `lr20xx_radio_bluetooth_le_set_pkt_params` (whitening per channel), `lr20xx_radio_fifo_clear_tx`, `lr20xx_radio_fifo_write_tx(pdu)`, `lr20xx_radio_bluetooth_le_set_tx(len)`, then `k_sem_take` (10 ms). The IRQ callback (USP thread) only gets and clears the IRQ and gives the semaphore, so two threads never use SPI at once. Sleep until `start + interval + rand(0..10)`.
  - Scanner: as in section 3.
- `main.c`: same as flrc. The OLED shows name, address, channels/power and the advert counter, or for the scanner, device count, adverts per second and best RSSI with its address.

Effort: about 0.5 day to get a phone seeing it, plus about 0.5 day for scanner polish.

## 5. Prototype

Sources: [`ble_beacon_proto/`](ble_beacon_proto/) (`src/app_ble.c`, `src/app_ble.h`, `src/main.c`, `Kconfig`, `prj.conf`).

To rebuild: copy `flrc_throughput/` to `ble_beacon/`, delete `src/app_flrc.*`, copy the prototype files over it, delete its `west.yml` (the workspace manifest stays in `flrc_throughput/`), and in `CMakeLists.txt` set `project(ble_beacon)` and replace `src/app_flrc.c` with `src/app_ble.c`. Then:

- Beacon: `west build -b xiao_nrf54l15/nrf54l15/cpuapp --shield semtech_loraplus_expansion_board --shield semtech_wio_lr2021 ble_beacon -d ble_beacon/build_beacon`. **Builds OK, no warnings.** FLASH 390140 B (25.0%), RAM 50044 B (19.1%).
- Scanner: same command with `-d ble_beacon/build_scan -- -DCONFIG_APP_ROLE_SCAN=y`. **Builds OK.** FLASH 389840 B, RAM 48020 B.
- The first link failed until `CONFIG_SEMTECH_LR20XX_BLE=y` was added.
- Not flashed.

## Risks

1. The whitening values for channels 38 and 39 (0x33, 0x73) are derived, not confirmed. Test channel 37 alone first. If 38 and 39 fail, try `0x40|ch`.
2. It is assumed that the engine sends the AA, the PDU and the CRC24 in BLE bit order (LSB first). The driver doc implies this, but it is untested.
3. Default TX timeout semantics of `bluetooth_le_set_tx`, and the RX meaning of `set_pdu_length`.
4. Frequency accuracy: BLE allows ±150 kHz, and the LR2021 TCXO/XTAL should be fine.
5. An SPI race if the main thread ever touches the radio. Keep all radio access in the beacon thread and the IRQ callback.
6. Use -17..+12 dBm only and stay at 2.4 GHz. Phones show it in nRF Connect as non-connectable "LR2021" with 0xFFFF manufacturer data.
