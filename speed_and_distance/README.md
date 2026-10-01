# Speed and distance demo (LR2021 LoRa Plus EVK)

Two boards alternate between two phases, forever:

- **SPEED** (5 s): the one-way FLRC stream from [`flrc_throughput/`](../flrc_throughput/README.md),
  with the same settings. The RX board shows goodput and packet success rate.
- **DISTANCE** (5 s): LoRa RTToF ranging. The RX board is the ranging **manager**. It
  computes the distance and shows it in metres. The TX board is the **subordinate**.

All transmissions stay in the 2.4 GHz ISM band. The sub-GHz path is never used.

## Phases and coordination

The TX board is the phase master. It switches phases on its own timer.

```
TX  |---- SPEED 5 s (FLRC TX) ----|---- DISTANCE 5 s (subordinate) ----|---- SPEED ...
                         flag ^^^^ (last 100 ms)
RX  |---- SPEED (FLRC RX) --------|---- DISTANCE (manager) ---------|-- SPEED ...
                                  ^ switches 2 ms before the TX      ^ 30 ms before the TX
```

- Bytes 4–7 of each FLRC payload are a header: flags, ms left in the SPEED phase, and
  cycle number. During the last 100 ms of SPEED, the TX sets the switch flag.
- On the first flagged packet, the RX board computes when the TX will switch. It leaves
  SPEED 2 ms before that time. It ends DISTANCE 30 ms before the TX does, so it is
  already in FLRC RX when the stream starts again.
- Fallback: if no FLRC packet arrives for 1 s during SPEED, the RX board goes to DISTANCE
  without a flag. That window is 4 s (5 s − 1 s), because the TX has at most that long
  left in its own DISTANCE phase.
- The manager sends a config packet and waits 50 ms for the ack. If no ack comes, it
  retries every 10 ms. This covers a few ms of skew between the boards.
- The manager starts a new run only if the run can finish before the window ends. A
  subordinate that misses 6 exchanges in a row stops the run and listens for a new config.
- If one board reboots, the boards re-sync within one cycle. A rebooted TX starts in
  SPEED, and the RX board returns to SPEED when its window ends. A rebooted RX starts in
  SPEED, then follows the next flag (or uses the fallback).

Phase lengths, flag lead time and fallback time are `CONFIG_APP_*` options in `Kconfig`.

## Radio settings

SPEED (unchanged from `flrc_throughput`):

- FLRC at 2440 MHz, 2.6 Mbps, CR 1/1, 511-byte payload, +5 dBm.
- Immediate radio access, pipelined TX, continuous RX.

DISTANCE (forked from the USP `ranging_demo`, `app_ranging_hopping.c`):

- LoRa SF5, BW 812 kHz (`RAL_LORA_BW_800_KHZ`), CR 4/5, preamble 12, +5 dBm.
- Config and ack packets at 2440 MHz.
- 29 RTToF exchanges per run, one on each channel of the stock "2G4" hop table
  (2402–2480 MHz). Each slot is 22 ms, so a run takes about 0.7 s. That gives about
  6 runs per DISTANCE phase.
- Calibration: the stock delay indicator for BW812 above 2 GHz at SF5 (18562).
- Distance of a run = median of the kept raw results × 150 / (812 kHz × 4096 / 10⁶),
  which is about 0.045 m per raw step, minus `CONFIG_APP_RANGING_OFFSET_CM` (default 0).
  A hop is kept when its exchange is valid and its RSSI is within
  `CONFIG_APP_RANGING_RSSI_WINDOW_DB` of the strongest hop in the run. The default is
  100 dB, which keeps every valid hop. A 10 dB window made results worse indoors (see
  Measured on the bench).
- The RX OLED shows the median of the last 5 run results (`CONFIG_APP_RANGING_SMOOTH_RUNS`)
  with one decimal, and their median absolute deviation as "+/-". The history is kept
  across phases and is cleared after 30 s with no result. The stock demo shows only
  whole metres from one run.

A build fails if any configured frequency (FLRC, ranging config or any hop channel) is
outside 2400–2483.5 MHz (`src/app_radio_limits.h`). Each frequency is also checked at run
time before it goes to the radio.

### Radio handover

- FLRC → ranging (`app_flrc_stop()`): standby, clear IRQs, clear both FIFOs, set the
  fallback mode back to STDBY_RC, then `smtc_rac_release_immediate_radio_access()`.
  After that, the app submits RAC ranging transactions.
- Ranging → FLRC (`app_ranging_stop()`, then `app_flrc_start()`): stop new submits, call
  `smtc_rac_abort_radio_submit()`, and wait for the aborted task's callback. Then take
  immediate access from the main thread and run `radio_configure()` again.
- Phase switches run in the main thread at a cooperative priority (-2), so the USP thread
  cannot preempt them. Radio callbacks never call LVGL or I2C. They write to the latest-value
  slots and wake the main thread.

## Build

From the west workspace root (`lr2021/`):

```bash
source .venv/bin/activate

# TX board (7110B0C1, OLED at 0x3D)
west build -p -b xiao_nrf54l15/nrf54l15/cpuapp \
  --shield semtech_loraplus_expansion_board --shield semtech_wio_lr2021 \
  speed_and_distance -d speed_and_distance/build_tx \
  -- -DEXTRA_DTC_OVERLAY_FILE=boards/oled_addr_0x3d.overlay

# RX board (3EF60302)
west build -p -b xiao_nrf54l15/nrf54l15/cpuapp \
  --shield semtech_loraplus_expansion_board --shield semtech_wio_lr2021 \
  speed_and_distance -d speed_and_distance/build_rx -- -DCONFIG_APP_ROLE_RX=y
```

## Flash

```bash
pyocd flash -t nrf54l -u 7110B0C1 speed_and_distance/build_tx/zephyr/zephyr.hex
pyocd flash -t nrf54l -u 3EF60302 speed_and_distance/build_rx/zephyr/zephyr.hex
```

## Screens

TX board (text layout):

```
SPEED  FLRC TX  #12          DIST  ranging SUB  #12
2440 MHz +5 dBm 511B         LoRa SF5 BW812 2G4 hop
Pkts/s: 528                  cfg ok: 7  runs: 6
TX: 2.15 Mbps                answered: 177
Seq: 21127  err: 0           missed: 0  cut: 0
-> DIST in 3 s               -> SPEED in 2 s
```

RX board (large font):

```
SPEED  FLRC RX 2440          DIST  ranging MGR  #12
   2.13 Mbps                       6.4 m          <- median of the last 5 runs
  98.6% pkts                   +/-0.4  n=5        <- their median absolute deviation
RSSI -33  CRC 3  lost 7      run 6.6 m 27/29 RSSI -50   <- last run
```

## Console

Each board prints a stats line every second and a line at each phase switch. For each
ranging run, the RX board prints the per-hop results (raw/RSSI, `x` = no valid exchange,
`*` = dropped by the RSSI filter) and a summary line. Measured lines (the RANGE line is from
the build before the per-hop log and smoothing were added):

```
=== SPEED (FLRC TX) cycle 9, t=80067 ms, switch #17
TX SPEED 1000 ms: 528 pkt (528 pkt/s), 2.15 Mbps payload, seq 22182, err 0, disp 1593
=== DISTANCE (ranging subordinate) cycle 9, t=85067 ms, 5000 ms window, reason: timer, switch #18
TX DIST: cfg rx windows 7, cfg ok 7, runs 6 (cut 0), answered 177, missed 0, 999 ms left

RX SPEED 1000 ms: ok 524 / exp 529 (99.0%), 2.14 Mbps goodput, crc 2, tmo 0, resync 0, fifo 0, seq 21113, rssi -33, disp 327
=== DISTANCE (ranging manager) cycle 9, t=27532 ms, 4972 ms window, reason: flag, switch #8
RANGE run 21: 4.3 m (raw median 95), 29/29 valid, rssi -30
RX DIST: cfg 5, ack 5, runs 4, exch ok 133 fail 0, 1967 ms left
```

## Measured on the bench

- SPEED: TX 527–529 pkt/s, 2.15 Mbps. RX 2.03–2.14 Mbps goodput, 96.5–99.4 % packets
  with boards close together, and 1.93–2.09 Mbps, 90–97 % at 1.9 m.
- DISTANCE: about 7 runs per 5 s phase, usually 27–29 of 29 exchanges valid.
- Accuracy (indoors): **expect a few metres of error.** At a true 1.9 m, readings were
  5.1–8.1 m (median 6.4 m, raw median about 142, RSSI about -50 dBm). Earlier placements
  closer together read 1–4 m. The per-hop logs show the cause: each channel has its own
  multipath bias, and it repeats from run to run. Hops at 2428–2444 MHz always read about
  +12–17 m. Hops at 2458–2466 MHz read negative and are also the strongest, so an RSSI
  filter keeps exactly the biased subset. A plain median over all 29 hops is the tightest
  estimate. The bias changes with placement, so `CONFIG_APP_RANGING_OFFSET_CM` stays at 0.
  A single-point offset would only be correct at that one spot. Expect better results
  outdoors or over longer distances; that is not tested yet.
- Phases alternated for more than 40 switches without a missed handover. After a reset of
  either board, the pair re-synced within one cycle.

The `reason` is `timer` on the TX board. On the RX board it is `flag` (switch flag
received) or `no FLRC` (fallback).
