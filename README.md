# LR2021 experiments

Firmware experiments for the Seeed/Semtech LR2021 LoRa Plus EVK: two XIAO nRF54L15
boards, each with a Wio-LR2021 radio on the LoRa Plus Expansion Board (SSD1306 OLED).

This repo is a [west](https://docs.zephyrproject.org/latest/develop/west/) workspace
root. Zephyr v4.4.0 and Semtech USP v1.2.1 are pulled in through
`flrc_throughput/west.yml` and are not committed.

## Projects

| Directory | What it does |
|---|---|
| [`flrc_throughput/`](flrc_throughput/README.md) | One-way FLRC throughput demo at 2.44 GHz. Measured result: about 2.1 Mbps goodput and 96–98 % packet success, shown on the RX board's OLED. |
| [`speed_and_distance/`](speed_and_distance/README.md) | Alternates 5 s of the FLRC throughput stream with 5 s of LoRa ranging (SF5, 812 kHz, 2402–2480 MHz hopping). The RX board shows Mbps and success rate, then the distance in metres. |

## Parked research

- [`docs/research/ble_beacon.md`](docs/research/ble_beacon.md): using the LR2021's own BLE engine as a beacon or scanner. Feasible, and the prototype compiles, but it has not been tested over the air.

## Setup (macOS)

```bash
brew install cmake ninja gperf ccache dtc libmagic
uv venv --python 3.13 .venv && source .venv/bin/activate
uv pip install west "pyocd>=0.38" pip

west init -l flrc_throughput
west update --narrow -o=--depth=1
west zephyr-export
west packages pip --install
(cd zephyr && west sdk install --version 1.0.1 -t arm-zephyr-eabi)
```

This takes about 8 GB of disk: about 6.4 GB for the workspace and 1.4 GB for the SDK in
`~/zephyr-sdk-1.0.1`.

Build and flash commands for each project are in its README.

## License

MIT (see [`LICENSE`](LICENSE)), except files derived from Semtech sources, which keep
the Clear BSD or Apache-2.0 license stated in their headers.
