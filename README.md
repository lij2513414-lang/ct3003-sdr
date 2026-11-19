# CT3003 SDR (MT7981B)

Real-time dual-band (2.4 GHz & 5 GHz) Software-Defined Radio for **CT3003 (MediaTek MT7981B + MT7976CN)** routers, running alongside stock `mt_wifi` APs without disrupting Wi-Fi.

## Features

- **Dual-band IQ capture (`mode=1`)**: `80 Msps` on 2.4 GHz (`160 Msps` on 5 GHz `>=80M`, `40 Msps` on RXFD Node 6), up to `16384` words/frame.
- **High-speed kernel async streaming (`wm_sdr_net.c`)**: Zero-copy `kernel_sendpage()` across parallel WebSocket lanes (`~821–881 Mbps` over 1 GbE).
- **Web SDR UI (`host/web/`)**: Live FFT, Waterfall, Time-Domain, I/Q Constellation, configurable UI sampling rate limit (`1–400 Hz` / `Unlimited`), and built-in **BLE 1M advertising decoder**.

## Tested Environment

- **Target Router**: CT3003 (`MediaTek MT7981B` dual-core `aarch64_cortex-a53`, `MT7976CN` RF, ~225 MiB usable RAM)
- **Router OS**: `ImmortalWrt 24.10-SNAPSHOT r33003-bc90209e19` (`mediatek/mt7981`)
- **Kernel**: `Linux 5.4.284 #0 SMP aarch64 GNU/Linux`
- **Network Link**: `1000 Mbps` Full-Duplex Ethernet (`lan3` to host PC)
- **Build Toolchain**: `OpenWrt GCC 13.3.0` (`aarch64-openwrt-linux-musl-gcc`, OpenWrt 24.10.0 Filogic toolchain) on WSL, `Python 3.14`
- **Client**: Windows + Chromium-based browser

## Directory Structure

- `firmware/` — Stock E1 ALPS patch, NDS32 IQ hook sources, and prebuilt `mt7981_patch_iq.bin`
- `kernel/wm_sdr/` — `/dev/wmsdr` driver (`wm_sdr.ko`) with WFDMA `mmap()` and async multi-stream TX
- `kernel/wmrom/` — WM ROM dump helper module (`wmrom.ko`)
- `host/` — `wmsdr` CLI/HTTP/WS server and `host/web/` browser UI
- [TECH_SHARE.md](TECH_SHARE.md) — Background, reverse-engineering overview, and design notes

## Quick Start (Router)

### 1. Load Firmware Patch & Driver

Bind-mount the patched firmware (non-persistent; rebooting restores stock) and load `wm_sdr.ko`:

```sh
mount --bind /root/wmtest/mt7981_patch_iq.bin /lib/firmware/mt7981_patch_e1_hdr.bin
insmod /root/wmtest/wm_sdr.ko
/root/wmtest/wmsdr info
```

### 2. Start Web SDR Server

```sh
/root/wmtest/wmsdr serve 8088 /root/wmtest/web
```

Open `http://<router-ip>:8088/` in a browser and click **Start Streaming**.

### 3. BLE Advertising Decoding

In the Web UI, click **BLE Preset (Node 6)** under **Bluetooth Advertising Decoder (BLE 2.4G)** (sets `2.4 GHz`, `Node 6`, `16384` words, `40 Msps`) and start streaming to decode nearby BLE advertising packets (e.g., Channel 38 at `2426 MHz` when the AP operates on `2437 MHz`).

## CLI Usage

```sh
wmsdr info
wmsdr iqstatus
wmsdr iqcapture out.bin 15 8192 0 0 0
wmsdr iqstream 15 16384 0 0 0 100 0
wmsdr serve 8088 /root/wmtest/web
```

See [host/web/README.md](host/web/README.md) for Web UI modes, CLI flags, and HTTP/WebSocket API details.

## Build from Source (WSL / Linux)

```bash
python3 firmware/rom_patch/build_iq_patch.py
bash kernel/wm_sdr/build.sh
bash kernel/wmrom/build.sh
bash host/build.sh
```

> **Note**: Keep `mode=1` when `mt_wifi` APs (`ra0`/`rax0`) are active. Do not write to Flash/MTD or `rmmod mtk_warp`.

## License

Licensed under the [MIT License](LICENSE).
