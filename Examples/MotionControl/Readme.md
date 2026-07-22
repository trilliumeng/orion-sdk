# Motion Control Example

Step response testing and control loop analysis for Orion gimbals using the `OrionCmdExtended` packet.

## Firmware Requirement

This example uses the `OrionCmdExtended` packet format (packet type 0x01), which requires Clevis and Crown firmware **3.1 or newer**. Earlier firmware will not respond to the extended command packet. Contact Trillium Engineering if you need a compatible build.

## Theory of Operation

The application connects to the gimbal and sends position commands at 80 Hz using the `OrionCmdExtended` packet (type 0x01). The extended packet includes a sequence number that the gimbal echoes in each response, enabling precise correlation between every command and its corresponding response.

This provides:

- **Synchronized command/response pairs** for step response analysis
- **Accurate timing measurements** at each stage: TX send, gimbal execution, RX receive
- **Packet drop detection** for both uplink (command lost) and downlink (response lost)

A single-threaded loop in main() sends commands at precise intervals: pre-compute the next command, spin-wait using QPC (Windows) or clock_gettime (Linux), send immediately, then drain responses and accumulate statistics in the remaining period. No external timer libraries or file I/O was used during the test in order to keep the timing as deterministic as possible.

## Usage

Pass the gimbal address as the first argument — an IP address, a Windows COM port, or a Linux serial device — optionally followed by `--null-gyros`:

```
MotionControl.exe 169.254.87.45
MotionControl.exe 169.254.87.45 --null-gyros
MotionControl.exe \\.\COM4
./MotionControl /dev/ttyUSB0 --null-gyros
```

**`--null-gyros`** runs a 30-second gyro-null pre-step before the test. Default is to skip it, which keeps repeated iterations fast; enable it for qualifying runs or when stabilized-rate behavior is being measured.

**Windows COM port:** the `\\.\` Win32 device-namespace prefix is required. A bare `COM4` will not be recognized as a serial path and the example will fall through to ethernet broadcast.

**Enter the COM port form at a Windows command prompt (`cmd.exe`)**, where backslashes are literal. Shells that quote arguments before invoking the process (PowerShell, Git Bash, MSYS) need extra escaping to preserve the prefix and are not recommended for this argument form.

Run with `-h` or `--help` to print usage and a summary of the compile-time test parameters the binary was built with.

Test parameters (mode, axis, signal, amplitude, frequency, cycles, rate) are set as compile-time constants at the top of `MotionControl.c`. Edit those and rebuild to change the test. Each run writes a CSV named after the interface (e.g. `responses_169.254.87.45.csv`).

## Ethernet vs. Front Panel Serial

The gimbal exposes two command paths from the PC. The choice of interface affects latency and jitter, which matters for high-rate control loops.

**Ethernet:** PC → UDP → payload computer → crown → UART → clevis

**Front panel serial:** PC → serial → crown → UART → clevis

The serial path skips the payload computer entirely. The payload computer runs a general-purpose OS, which introduces scheduling variability on every packet it handles. This shows up as execute jitter — the variation in time between when the command is sent and when the gimbal executes it.

### Measured results (80 Hz, 5 deg step, pan axis)

| Metric | Ethernet | Front Panel Serial |
|--------|----------|--------------------|
| RTT mean | 46.3 msec | 32.6 msec |
| RTT max | 116 msec | 87 msec |
| Execute jitter (RMS) | 10.8 msec | 2.8 msec |
| Packet drops | 0 | 0 |

Execute jitter drops by 4x on the serial path. For control system integration, this is the most important metric — high execute jitter means the gimbal is executing commands at unpredictable times relative to your control loop, which degrades closed-loop performance. If you're integrating a controller that runs at 80 Hz or faster, front panel serial is the recommended interface.

## Building

Build prerequisites for both platforms are listed in [`Public/Readme.md`](../../Readme.md).

### Windows (MSVC)

```
MSBuild Public.sln -p:Configuration=Release -p:Platform=Win32
```

### Linux/macOS

Supported. The timing loop uses QPC on Windows and clock_gettime(CLOCK_MONOTONIC) on Linux/macOS. Build with gcc or clang and link -lrt on Linux.

## CSV Output

Each run writes `responses_<interface>.csv` with one row per command sent:

| Column | Description |
|--------|-------------|
| sequence_num | Command sequence number |
| time_sec | Time since test start |
| tx_time_us | Transmit timestamp (microsec) |
| rx_time_us | Receive timestamp (microsec) |
| rtt_us | Round-trip time (microsec) |
| received | 1 if response received, 0 if dropped |
| cmd_target_deg | Commanded position/rate (deg) |
| stab_gyro_rate_deg_s | Stabilization gyro rate (deg/s) |
| encoder_pos_deg | Encoder position (deg) |
| encoder_rate_deg_s | Encoder rate (deg/s) |
| clevis_time_ms | Gimbal timestamp when command executed |
