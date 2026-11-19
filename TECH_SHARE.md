# Capturing BLE Advertising Frames with a CT3003 (MT7981B) Router

## 1. Background

After coming across `h0m3us3r/w1700k-sdr` on GitHub, I wanted to see whether AI-assisted reverse engineering could achieve a similar result on the **CT3003 (MT7981B)** router. Ultimately, I got real-time IQ streaming and **Bluetooth Low Energy (BLE 1M) advertising packet capture and decoding** working without flashing MTD/Flash or disrupting normal dual-band Wi-Fi operation. This note shares the high-level approach, findings, and implementation details.

> Note: Because much of the analysis relied heavily on AI-assisted reverse engineering, some low-level hardware details are not yet fully understood, and a few inferred names or mechanisms may still contain inaccuracies. Other community projects targeting MT7981 (such as `MtkCSIdump` and `ekstra-csi`) were also helpful references along the way.

---

## 2. Hardware Architecture

The CT3003 router pairs an **MT7981B** SoC with an **MT7976CN** dual-band RF front-end IC:

- **MT7976CN**: Handles the 2.4 GHz / 5 GHz RF front end and ADC sampling, streaming digitized quadrature baseband I/Q data to the MT7981.
- **MT7981B**: The host CPU consists of dual ARM Cortex-A53 cores (~225 MiB usable RAM; tested on `ImmortalWrt 24.10-SNAPSHOT` with `Linux 5.4.284`). Inside the wireless subsystem sit coprocessors labeled **WM, WA, and WO** (inferred from firmware paths to correspond to Wireless Management, Acceleration, and Offload), connected to Linux host memory via the **WFDMA** DMA engine.

---

## 3. Analysis Process

Reference materials included `QATool_UIv2.78_DLLv6.83_ap_2021.11.05_RU_Customer`, the vendor `mt_wifi` driver source code, the on-device WM firmware, and the `w1700k-sdr` source code.

The WM coprocessor runs the **NDS32** instruction set. Using lightweight AI-built NDS32 disassembly/analysis helpers cross-checked against `mt_wifi`, the core mechanisms became clear:

1. **Driver Debug Path & ICAP Capture Nodes**
   Both `mt_wifi` and the `QATool`/ATE debug stack preserve control logic for the internal **ICAP** capture engine, which taps different stages along the RX chain: `ADC → AFIFO → RXFI → RXFD → DLPF → AGCDGC`. Each tap exposes either raw front-end samples or digitally filtered baseband data:
   - **ADC node (`Node 15`)**: Interleaved even/odd dual-antenna 12-bit raw ADC samples (`80 Msps` per chain on 2.4 GHz, up to `160 Msps` on wide 5 GHz bands). Great for wideband spectrum inspection, though unfiltered and relatively noisy out-of-band.
   - **RXFD node (`Node 6`)**: Digitally front-end-filtered and decimated 14-bit output (`40 Msps`), offering better out-of-band rejection and twice the capture duration per frame for narrowband demodulation.

2. **WM Firmware Structure & Load Flow**
   WM firmware operates across multiple stages: an immutable on-chip **Mask ROM** provides basic exception handling, communication, and the firmware downloader (FWDL); during driver initialization, WFDMA downloads the **ROM Patch** followed by the main **RAM firmware** (which also updates shared `ChInfo` fields with active 2.4 GHz / 5 GHz center LO frequencies and channel bandwidths).

3. **ROM Hardware Instruction Replacer (ROM Patch Mechanism)**
   Disassembling the dumped `_ROM.bin` alongside the stock patch revealed that the SoC integrates a **64-slot hardware instruction replacement engine** on the ROM instruction-fetch bus (control bitmap registers `0x89040700` / `0x89040704` and slot table `0x89040720`):
   - The start of Patch SRAM (`0x00900000`) holds a 64-bit enable bitmap plus 64 `(src_addr, dst_addr)` pairs (the stock patch enables slots `0..9`).
   - Whenever a slot bit is `1` and the CPU fetches `src[slot]`, **hardware intercepts the 32-bit bus read on the fly and synthesizes an unconditional NDS32 jump `0x48000000 | ((dst - src) >> 1)` (`j dst`)**, redirecting execution into Patch SRAM with zero software table lookup overhead.
   - Moreover, the stock loader finishes `ALPS` patch downloads with `ucCheckCrc = 0` (no full-section CRC check), making it possible to extend the section descriptor into unused upper Patch SRAM (`0x00903000`).

4. **Identifying the COS RTOS**
   Inspecting decompiled pseudo-C structures revealed familiar HAL/RTOS scheduling, mailbox, and messaging patterns. Searching ROM/firmware strings uncovered numerous original source path names (`cos_api_rom.c`, `cos_task_rom.c`, `wsys_mbox_rom.c`, etc.), confirming that WM runs on an RTOS named **COS**.

5. **Mapping the RTOS & ICAP Control Path**
   Further analysis located `IcapConfigRequest`, the node lookup table, and the shared bus L2 remap window—allowing the Linux host to safely save/restore vendor window state and read/write WM SRAM directly without contending with the Wi-Fi MCU mailbox queue.

---

## 4. Implementation

1. **Non-Intrusive IQ Capture via a Parasitic ROM Patch Hook**
   Extending the upper region of the ALPS patch at `0x00903000`, a 10-byte trampoline was placed at `delay_us` (`0x0090044E`, already redirected by stock `slot 9`) to run a compact **`WMIQ` state-machine mailbox**:
   - **Cooperating with COS RTOS**: After arming a capture, the hook polls briefly (`~1.5–2.5 ms` max) and immediately yields back to Wi-Fi tasks (`ARMED`) if a longer frame is still running, finishing on the next tick so stock `mt_wifi` AP traffic stays uninterrupted.

2. **Kernel-Space Async Multi-Stream Zero-Copy TX (Breaking the MT7981 Bottleneck)**
   Because the dual-core A53 CPU struggled with user-space copying and single-TCP bottlenecks at high rates, network streaming was moved directly into the kernel driver (`wm_sdr.ko`):
   - **WFDMA Direct to Compound Pages**: Hardware IQ frames land in pre-allocated 32/64 KiB compound pages paired with shared 4 KiB RFC 6455 WebSocket + `WSIQ` header pages.
   - **Work-Stealing Multi-Lane `kernel_sendpage()`**: A shared ready ring feeds multiple concurrent WebSocket TCP sockets via non-blocking workers using zero-copy `kernel_sendpage()` (yielding cleanly on `-EAGAIN` via `sk_write_space` callbacks so slow sockets never stall the pipeline).
   - Over 1 GbE, sustained streaming reaches **`~820–880 Mbps` (~90–100 MB/s class)** at **`~50%–80%`** dual-core MT7981 CPU utilization.

3. **Pure-JS Browser DSP & Live BLE 1M Decoding**
   The router only runs a tiny C static HTTP/WebSocket handshake server; all visualization (Spectrum FFT, Waterfall, Time Domain, Constellation) and **BLE 1M advertising decoding** happen in client-side JavaScript:
   - **Why `Node 6 (RXFD)` instead of `Node 15 (ADC)`**: Aside from ADC lacking digital filtering and picking up heavier out-of-band noise, the hardware per-frame limit of `16384` words (`8192` complex samples after single-antenna decimation) gives only **`102.4 μs`** per frame at `80 Msps`, frequently truncating BLE packets. Switching to **`RXFD (Node 6, 40 Msps, 14-bit)`** doubles the window to **`204.8 μs`**. With the 2.4 GHz AP operating on `2437 MHz` (Ch 6), **BLE advertising Channel 38 (`2426 MHz`)** falls cleanly at `-11 MHz`, letting nearby devices (e.g., a Bluetooth mouse) appear live in the browser table.
   - **Verified Polarity & CRC-24 Gate**: The decoder adapts to MT7981B Band 0's `I - jQ` conjugate polarity, shifts to baseband, runs a 17-tap Hamming low-pass filter decimated to `8 Msps`, demodulates GFSK via instantaneous frequency, syncs on the BLE advertising Access Address (`0x8E89BED6`), de-whitens payloads, and enforces **CRC-24** verification to reject noise false positives. A fixed-interval UI frame limiter (`1–400 Hz`) decouples high-rate background scanning from Canvas redraws.
   - Because each frame spans only ~200 microseconds, **network throughput and packet continuity directly impact the BLE capture success rate**.

---

## 5. Limitations

1. **Limited RF Parameter Tuning**: Capture currently piggybacks on the active Wi-Fi AP's channel and tuning state; standalone arbitrary frequency tuning and dynamic gain (LNA/VGA) controls were not implemented (AI usage quota ran out).
2. **Knowledge Gaps from Rapid AI Iteration**: Relying heavily on AI to synthesize reverse-engineering notes, assembly patches, and driver code means several deeper COS subsystems and RF calibration state machines remain only partially understood.
