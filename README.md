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
