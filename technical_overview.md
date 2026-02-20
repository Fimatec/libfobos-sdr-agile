# libfobos_sdr (Agile) - Technical Overview

## 1. What Is This Project?

**libfobos_sdr** (agile) is a C shared library that provides a host-side API for the [RigExpert Fobos SDR](https://rigexpert.com/en/products/kits-en/fobos-sdr/) receiver. It lets applications enumerate devices, configure RF parameters, and stream I/Q sample data over USB.

This is the **"agile" variant** -- a separate, lighter-weight API with a different enumeration/control/streaming model and a **fast frequency scan mode** not available in the original [libfobos](https://github.com/rigexpert/libfobos). Both libraries can coexist on the same system, but this one requires **special agile firmware** (v3.1.0 or later) on the device.

- **License:** LGPL-2.1+
- **Language:** C99
- **Current version:** 3.1.1 (beta)
- **Sole dependency:** libusb-1.0 (v1.0.21+)
- **Platforms:** Linux (Ubuntu 18.04+, Raspbian), Windows (7-11, x86/x64), macOS

---

## 2. Directory Layout

```
libfobos-sdr-agile/
├── fobos/
│   ├── fobos_sdr.h              # Public API header (all exported functions)
│   └── fobos_sdr.c              # Complete library implementation (~1600 lines)
├── eval/
│   ├── fobos_sdr_devinfo_main.c # Example: enumerate devices, print board info
│   ├── fobos_sdr_recorder_main.c# Example: stream I/Q to WAV file
│   ├── fobos_sdr_scanner_main.c # Example: fast frequency scanning to WAV
│   └── fobos_sdr_fwloader_main.c# Tool: read/write firmware to device
├── wav/
│   ├── wav_file.h               # WAV file I/O interface
│   └── wav_file.c               # WAV file I/O implementation
├── cmake/
│   └── cmake_uninstall.cmake.in # CMake uninstall helper
├── CMakeLists.txt               # Build configuration
├── libfobos_sdr.pc.in           # pkg-config template
├── fobos-sdr.rules              # Linux udev rules for non-root access
├── versions.txt                 # Changelog
├── LICENSE                      # LGPL-2.1+
└── README.md
```

The entire library lives in **two files**: `fobos/fobos_sdr.h` (public API) and `fobos/fobos_sdr.c` (implementation). Everything else is examples, utilities, or build infrastructure.

---

## 3. Build System

**CMake** (minimum 3.7.2). The build produces:

| Target | Type | Description |
|---|---|---|
| `libfobos_sdr` | Shared library (.so / .dll) | The library itself |
| `fobos_sdr_devinfo` | Executable | Device enumeration tool |
| `fobos_sdr_recorder` | Executable | I/Q recording example |
| `fobos_sdr_scanner` | Executable | Frequency scanning example |
| `fobos_sdr_fwloader` | Executable | Firmware upload/download tool |

### Build on Linux

```bash
mkdir build && cd build
cmake ..
make
sudo make install
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`make install` places:
- `libfobos_sdr.so` in the system library directory
- `fobos_sdr.h` in `/usr/local/include/`
- `fobos-sdr.rules` in `/etc/udev/rules.d/` (grants access to `plugdev` group)
- `libfobos_sdr.pc` for pkg-config

### Build on Windows

```bash
mkdir build && cd build
cmake ..           # or: cmake .. -A Win32  (for 32-bit)
cmake --build . --config Release
```

Requires libusb binaries placed in a `libusb/` directory at the repo root. See README for details.

---

## 4. Hardware Communication Layer

### USB Identifiers

| Field | Value |
|---|---|
| Vendor ID | `0x16D0` |
| Product ID | `0x132E` |
| Device ID | `0x0101` |
| Bulk IN endpoint | `0x81` |
| Control timeout | 300 ms |

### USB Command Protocol

All device control goes through **libusb vendor-type control transfers**. The library defines these request codes:

| Request Code | Purpose |
|---|---|
| `0xE0` | Device reset (hardware reboot) |
| `0xE1` | Main command interface (frequency, sample rate, gains, start/stop, etc.) |
| `0xE2` | SPI passthrough (bidirectional) |
| `0xE7` | I2C passthrough (read/write) |
| `0xE8` | Board info queries (hw revision, fw version, fw build) |
| `0xEC` | Read firmware from device |
| `0xED` | Write firmware to device |

The main command interface (`0xE1`) uses the `wValue` field to select a sub-command:

| Sub-command | Code | Data | Description |
|---|---|---|---|
| `CMD_OPEN` | `0x00` | -- | Initialize device session |
| `CMD_CLOSE` | `0x01` | -- | End device session |
| `CMD_START` | `0x02` | packs_per_transfer (wIndex) | Start data streaming |
| `CMD_STOP` | `0x03` | -- | Stop data streaming |
| `CMD_SET_FREQ` | `0x10` | 8 bytes (uint64, Hz) | Set RX frequency |
| `CMD_SET_SR` | `0x11` | 8 bytes (uint64, Hz) | Set sample rate |
| `CMD_SET_BW` | `0x12` | 8 bytes (uint64, Hz) | Set exact bandwidth |
| `CMD_SET_AUTOBW` | `0x13` | 8 bytes (uint64, ratio*1024) | Set auto bandwidth |
| `CMD_SET_DIRECT` | `0x20` | wIndex: 0/1 | Enable/disable direct sampling |
| `CMD_SET_CLK_SOURCE` | `0x21` | wIndex: 0/1 | Internal/external clock |
| `CMD_SET_LNA` | `0x22` | wIndex: 0-3 | LNA gain |
| `CMD_SET_VGA` | `0x23` | wIndex: 0-31 | VGA gain |
| `CMD_SET_GPO` | `0x30` | wIndex: 0x00-0xFF | User GPIO output |
| `CMD_START_SCAN` | `0x40` | N*8 bytes (frequency array) | Start frequency scanning |
| `CMD_STOP_SCAN` | `0x41` | -- | Stop frequency scanning |

### Data Transfer

I/Q sample data arrives via **USB bulk transfers** on endpoint `0x81`. In async mode, the library submits multiple transfers (configurable 2-64, default 16) and processes them via libusb's event loop. In sync mode, it uses blocking `libusb_bulk_transfer()`.

On Linux, the library attempts **zero-copy** memory allocation (`libusb_dev_mem_alloc`) when libusb >= 1.0.5 is available, falling back to regular `malloc`.

---

## 5. Public API Reference

All functions return `int` error codes (0 = success) unless noted otherwise. The device handle is an opaque `struct fobos_sdr_dev_t *`.

### 5.1 Device Management

```c
// Get library version string and driver name. Can be called before opening a device.
int fobos_sdr_get_api_info(char *lib_version, char *drv_version);

// Return count of connected Fobos SDR (agile) devices.
int fobos_sdr_get_device_count(void);

// Get space-delimited serial numbers of all connected devices.
int fobos_sdr_list_devices(char *serials);

// Open device by index (0-based). Allocates handle, claims USB interface,
// reads board info, sets defaults: freq=100MHz, SR=10MHz, auto_bw=0.9.
int fobos_sdr_open(struct fobos_sdr_dev_t **out_dev, uint32_t index);

// Close device. Stops any active streaming, releases USB resources, frees handle.
int fobos_sdr_close(struct fobos_sdr_dev_t *dev);

// Close and trigger hardware reset (used after firmware update).
int fobos_sdr_reset(struct fobos_sdr_dev_t *dev);

// Read hardware revision, firmware version, manufacturer, product, serial strings.
int fobos_sdr_get_board_info(struct fobos_sdr_dev_t *dev,
    char *hw_revision, char *fw_version, char *manufacturer,
    char *product, char *serial);
```

### 5.2 Frequency Control

```c
// Set RX center frequency in Hz. Ignored while frequency scanning is active.
int fobos_sdr_set_frequency(struct fobos_sdr_dev_t *dev, double value);

// Start fast frequency scanning. frequencies = array of Hz values, count = 2..256.
int fobos_sdr_start_scan(struct fobos_sdr_dev_t *dev,
    double *frequencies, unsigned int count);

// Stop frequency scanning.
int fobos_sdr_stop_scan(struct fobos_sdr_dev_t *dev);

// Get current scan frequency index. Returns -1 while tuning or when not scanning.
int fobos_sdr_get_scan_index(struct fobos_sdr_dev_t *dev);

// Returns 1 if scanning is active, 0 if not, <0 on error.
int fobos_sdr_is_scanning(struct fobos_sdr_dev_t *dev);
```

### 5.3 Receiver Configuration

```c
// Enable (1) or disable (0) direct HF sampling, bypassing the RF frontend.
int fobos_sdr_set_direct_sampling(struct fobos_sdr_dev_t *dev, unsigned int enabled);

// Set LNA gain: 0..3 (clamped if out of range).
int fobos_sdr_set_lna_gain(struct fobos_sdr_dev_t *dev, unsigned int value);

// Set VGA gain: 0..31 (clamped if out of range).
int fobos_sdr_set_vga_gain(struct fobos_sdr_dev_t *dev, unsigned int value);

// Get array of supported sample rates and their count.
int fobos_sdr_get_samplerates(struct fobos_sdr_dev_t *dev,
    double *values, unsigned int *count);

// Set sample rate (Hz). Must be one of the supported values.
int fobos_sdr_set_samplerate(struct fobos_sdr_dev_t *dev, double value);

// Set exact RX filter bandwidth in Hz. Disables auto-bandwidth.
int fobos_sdr_set_bandwidth(struct fobos_sdr_dev_t *dev, double value);

// Set bandwidth as fraction of sample rate (0.0..1.0). Typical: 0.8-0.9.
// Set 0.0 or call fobos_sdr_set_bandwidth() to disable auto mode.
int fobos_sdr_set_auto_bandwidth(struct fobos_sdr_dev_t *dev, double value);

// Set clock source: 0 = internal (default), 1 = external reference.
int fobos_sdr_set_clk_source(struct fobos_sdr_dev_t *dev, int value);

// Set user general-purpose output bits (0x00..0xFF).
int fobos_sdr_set_user_gpo(struct fobos_sdr_dev_t *dev, uint8_t value);
```

### 5.4 Asynchronous Streaming (Callback Mode)

```c
// Callback signature. buf = interleaved float I/Q pairs, buf_length = complex sample count.
typedef void(*fobos_sdr_cb_t)(float *buf, uint32_t buf_length,
    struct fobos_sdr_dev_t *sender, void *user);

// Start async streaming. Blocks the calling thread in the libusb event loop.
// buf_count: USB transfer count (0=default 16, max 64).
// buf_length: complex samples per buffer (must be multiple of 8192, min 8192,
//             use >= 65536 for frequency scanning).
int fobos_sdr_read_async(struct fobos_sdr_dev_t *dev,
    fobos_sdr_cb_t cb, void *user, uint32_t buf_count, uint32_t buf_length);

// Request async streaming stop. Safe to call from within the callback.
int fobos_sdr_cancel_async(struct fobos_sdr_dev_t *dev);
```

### 5.5 Synchronous Streaming (Polling Mode)

```c
// Start sync mode. buf_length = complex samples per read (0 = default 65536).
int fobos_sdr_start_sync(struct fobos_sdr_dev_t *dev, uint32_t buf_length);

// Read one buffer of samples. Blocking. Caller provides float buffer.
int fobos_sdr_read_sync(struct fobos_sdr_dev_t *dev,
    float *buf, uint32_t *actual_buf_length);

// Stop sync mode.
int fobos_sdr_stop_sync(struct fobos_sdr_dev_t *dev);
```

Async and sync modes are **mutually exclusive**. Attempting to use both simultaneously returns an error.

### 5.6 Firmware Management

```c
// Read 160 KB firmware from device and save to file.
int fobos_sdr_read_firmware(struct fobos_sdr_dev_t *dev,
    const char *file_name, int verbose);

// Write firmware file to device (max ~260 KB). Must not be streaming.
// Call fobos_sdr_reset() after writing to reboot into new firmware.
int fobos_sdr_write_firmware(struct fobos_sdr_dev_t *dev,
    const char *file_name, int verbose);
```

### 5.7 Error Handling

```c
// Get human-readable error description string.
const char *fobos_sdr_error_name(int error);
```

| Code | Constant | Meaning |
|---|---|---|
| 0 | `FOBOS_ERR_OK` | Success |
| -1 | `FOBOS_ERR_NO_DEV` | dev == NULL or device not found |
| -2 | `FOBOS_ERR_NOT_OPEN` | Device handle not open |
| -3 | `FOBOS_ERR_NO_MEM` | Memory allocation failed |
| -4 | `FOBOS_ERR_CONTROL` | USB control transfer failed |
| -5 | `FOBOS_ERR_ASYNC_IN_SYNC` | Async read attempted while sync mode active |
| -6 | `FOBOS_ERR_SYNC_IN_ASYNC` | Sync start attempted while async reading |
| -7 | `FOBOS_ERR_SYNC_NOT_STARTED` | Sync read before start_sync |
| -8 | `FOBOS_ERR_UNSUPPORTED` | Unsupported parameter or mode |
| -9 | `FOBOS_ERR_LIBUSB` | Underlying libusb error |
| -10 | `FOBOS_ERR_BAD_PARAM` | Invalid parameter value |

---

## 6. Data Flow & Signal Processing Pipeline

```
  Fobos SDR Hardware
  (14-bit ADC, int16 I/Q pairs)
         │
         │  USB bulk transfer (endpoint 0x81)
         ▼
  libusb callback (_libusb_callback)
         │
         ▼
  fobos_sdr_convert_all()
  ┌──────────────────────────────────────────┐
  │  1. Read stream header flags             │
  │     - bit 15: IQ swap flag               │
  │     - bit 14: data interleave flag       │
  │                                          │
  │  2. Scan marker extraction (if scanning) │
  │     - Detect 0x2AAA/0x1555 signature     │
  │     - Extract current frequency index    │
  │                                          │
  │  3. I/Q balance calibration              │
  │     - Compute avg deviation per channel  │
  │     - Adaptive scale_im adjustment       │
  │     - Smoothed with alpha = 0.0001       │
  │     - Skipped in direct sampling mode    │
  └──────────────────┬───────────────────────┘
                     │
                     ▼
  fobos_sdr_convert_buff()  (per data chunk)
  ┌──────────────────────────────────────────┐
  │  For each complex sample:                │
  │    1. Mask to 14 bits (& 0x3FFF)         │
  │    2. Cast to float                      │
  │    3. DC offset removal (IIR high-pass): │
  │         dc += 0.0004 * (sample - dc)     │
  │         out = (sample - dc) * scale      │
  │    4. Apply I/Q gain correction          │
  │    5. Handle IQ swap if needed           │
  └──────────────────┬───────────────────────┘
                     │
                     ▼
  User callback / fobos_sdr_read_sync() return
  (float *buf: interleaved [I, Q, I, Q, ...])
```

### Raw data format

Each complex sample from the hardware is two `int16_t` values (4 bytes total). Only the lower 14 bits carry the ADC sample. The library converts these to `float` scaled by `1/32768.0` and applies DC offset correction.

### DC offset filter

An exponential moving average (first-order IIR) with coefficient `k = 0.0004`:

```
dc_re += k * (sample_re - dc_re)
output_re = (sample_re - dc_re) * scale_re
```

The DC state persists across buffers. Initial DC estimate is `8192.0` (midpoint of 14-bit range).

### I/Q balance calibration

The library continuously estimates the I/Q amplitude imbalance ratio (`avg_re / avg_im`) and adjusts `scale_im` so that the Q channel matches the I channel in amplitude. This is skipped in direct sampling mode.

---

## 7. Frequency Scanning

The "agile" feature. The hardware can rapidly hop between 2-256 frequencies.

### How it works

1. **Submit frequency list:** `fobos_sdr_start_scan(dev, freqs, count)` sends the array to the firmware via `CMD_START_SCAN`.

2. **Hardware cycles frequencies** automatically during streaming. Each buffer of samples corresponds to one frequency.

3. **Scan markers in the data stream:** The hardware embeds a signature pattern in the first samples of each buffer:
   - Two 14-bit words: `0x2AAA`, `0x1555`
   - Followed by the frequency index (AND of two copies for reliability)
   - These marker words are overwritten with neighboring sample data after extraction.

4. **Read the current index:** `fobos_sdr_get_scan_index(dev)` returns the current frequency index (0-based), or `-1` if the device is in the process of tuning between frequencies.

5. **Minimum buffer size:** Scanning requires `buf_length >= 65536` (which maps to `packs_per_transfer >= 16`). If the buffer is too small, scanning is silently disabled.

### Usage pattern

```c
double freqs[] = {100e6, 110e6, 120e6, 130e6};
fobos_sdr_start_scan(dev, freqs, 4);
fobos_sdr_read_async(dev, callback, ctx, 16, 65536);
// In callback:
//   int idx = fobos_sdr_get_scan_index(sender);  // 0..3 or -1
fobos_sdr_stop_scan(dev);
```

---

## 8. Concurrency Model

The library is **single-threaded by design**. There are no mutexes, atomic operations, or condition variables.

### Async mode threading

`fobos_sdr_read_async()` **blocks the calling thread** -- it runs the libusb event loop in a `while` loop until cancelled. The user's callback executes **within the libusb event-handling context** (same thread). This means:

- The callback should return quickly to avoid blocking USB processing.
- `fobos_sdr_cancel_async()` is safe to call from within the callback (it sets a flag and returns immediately).
- Do not call other library functions from a different thread while the event loop is running.

### State machine (async)

```
FOBOS_IDDLE → FOBOS_STARTING → FOBOS_RUNNING → FOBOS_CANCELING → FOBOS_IDDLE
```

State transitions are plain assignments (not atomic). This is safe because everything runs on one thread.

### Thread safety guidance

- One thread per device instance.
- If your callback writes to shared data structures, synchronize in your application code.
- Do not call library functions for the same device from multiple threads.

---

## 9. Key Data Structures

### Device handle (`struct fobos_sdr_dev_t`)

Defined in `fobos_sdr.c` (not exposed in the header). Key fields:

```c
struct fobos_sdr_dev_t {
    // USB
    libusb_context *libusb_ctx;
    struct libusb_device_handle *libusb_devh;
    struct libusb_transfer **transfer;     // async transfer array
    unsigned char **transfer_buf;          // raw data buffers
    uint32_t transfers_count;              // buffer count
    uint32_t transfer_buf_size;            // bytes per buffer
    int use_zerocopy;                      // Linux zero-copy optimization

    // Board info
    char hw_revision[32], fw_version[32], fw_build[32];
    char manufacturer[64], product[64], serial[64];

    // RX configuration
    double rx_frequency;                   // current frequency (Hz)
    double rx_samplerate;                  // current sample rate
    double rx_bandwidth;                   // current bandwidth
    double rx_auto_bandwidth;              // auto-BW ratio (0.0-1.0)
    uint32_t rx_lna_gain;                  // 0..3
    uint32_t rx_vga_gain;                  // 0..31
    uint32_t rx_direct_sampling;           // 0 or 1

    // Scanning state
    int rx_scan_active;                    // scanning enabled flag
    int rx_scan_freq_index;                // current frequency index (-1 if tuning)
    double rx_scan_freqs[256];             // frequency list

    // Signal processing state
    float rx_dc_re, rx_dc_im;             // DC offset estimates
    float rx_avg_re, rx_avg_im;           // running averages for I/Q balance
    float rx_scale_re, rx_scale_im;       // output scaling factors
    int rx_swap_iq;                        // IQ swap flag from hardware

    // Callback / streaming
    fobos_sdr_cb_t rx_cb;                 // user callback
    void *rx_cb_usrer;                    // user context
    enum fobos_async_status rx_async_status;
    float *rx_buff;                        // converted float output buffer

    // Sync mode
    int rx_sync_started;
    unsigned char *rx_sync_buf;            // raw sync-mode buffer
};
```

---

## 10. Supported Sample Rates

The library has a fixed list (defined in `fobos_sdr.c`):

| Sample Rate | Bandwidth at auto_bw=0.9 |
|---|---|
| 50 MHz | 45 MHz |
| 40 MHz | 36 MHz |
| 32 MHz | 28.8 MHz |
| 25 MHz | 22.5 MHz |
| 20 MHz | 18 MHz |
| 16 MHz | 14.4 MHz |
| 12.5 MHz | 11.25 MHz |
| 10 MHz | 9 MHz |
| 8 MHz | 7.2 MHz |

---

## 11. Example Programs

### `fobos_sdr_devinfo` (`eval/fobos_sdr_devinfo_main.c`)

Enumerates all connected devices and prints board info (HW revision, FW version, manufacturer, product, serial). Good starting point to verify the device is detected.

### `fobos_sdr_recorder` (`eval/fobos_sdr_recorder_main.c`)

Configures the device (100 MHz, 25 MHz sample rate) and records I/Q data to `rx.iq.wav` using async mode. Demonstrates the full lifecycle: enumerate, open, configure, stream with callback, close.

### `fobos_sdr_scanner` (`eval/fobos_sdr_scanner_main.c`)

Demonstrates fast frequency scanning across 4 frequencies (100-130 MHz). Shows how to use `fobos_sdr_start_scan()`, read scan indices in the callback, and optionally split data per channel into separate files.

### `fobos_sdr_fwloader` (`eval/fobos_sdr_fwloader_main.c`)

Command-line firmware tool:
```
fobos_sdr_fwloader -r firmware_backup.bin   # read from device
fobos_sdr_fwloader -w new_firmware.bin      # write to device (triggers reset)
```

---

## 12. Typical Usage Flow

```c
#include <fobos_sdr.h>

// 1. Enumerate
int count = fobos_sdr_get_device_count();

// 2. Open
struct fobos_sdr_dev_t *dev = NULL;
fobos_sdr_open(&dev, 0);

// 3. (Optional) Read board info
char hw[64], fw[64], mfg[64], prod[64], sn[64];
fobos_sdr_get_board_info(dev, hw, fw, mfg, prod, sn);

// 4. Configure
fobos_sdr_set_frequency(dev, 146e6);         // 146 MHz
fobos_sdr_set_samplerate(dev, 25e6);         // 25 Msps
fobos_sdr_set_auto_bandwidth(dev, 0.9);      // 22.5 MHz BW
fobos_sdr_set_lna_gain(dev, 2);
fobos_sdr_set_vga_gain(dev, 15);

// 5. Stream (async) -- blocks until cancelled
fobos_sdr_read_async(dev, my_callback, my_ctx, 16, 65536);

// 6. Close
fobos_sdr_close(dev);
```

The callback receives interleaved float I/Q samples:

```c
void my_callback(float *buf, uint32_t buf_length,
                 struct fobos_sdr_dev_t *sender, void *user) {
    for (uint32_t i = 0; i < buf_length; i++) {
        float I = buf[i * 2];
        float Q = buf[i * 2 + 1];
        // process sample...
    }
    // To stop: fobos_sdr_cancel_async(sender);
}
```

---

## 13. Constraints & Limits

| Parameter | Range / Limit |
|---|---|
| Sample rates | 8, 10, 12.5, 16, 20, 25, 32, 40, 50 MHz (fixed list) |
| LNA gain | 0..3 |
| VGA gain | 0..31 |
| Auto bandwidth ratio | 0.0..1.0 |
| Scan frequency count | 2..256 |
| Min buffer size (buf_length) | 8192 complex samples |
| Min buffer for scanning | 65536 complex samples |
| Max async buffer count | 64 |
| Firmware file size | max 0x3FFE0 bytes (~260 KB) |
| ADC resolution | 14-bit |
| Direct sampling | 0 = RF frontend, 1 = HF1+HF2 direct |
| Clock source | 0 = internal, 1 = external |

---

## 14. WAV File Utility

The `wav/` directory provides a standalone WAV file reader/writer used by the example programs. It supports:

- IEEE float32 format (`audio_format = 3`) for direct I/Q recording
- 2-channel (stereo) layout mapping to I and Q
- Header re-writing to update sample counts after streaming

This is a utility for the examples, not part of the core library.
