# gr-rigexpert Technical Overview

A GNU Radio out-of-tree (OOT) module that provides a complex IQ signal source block for the **Fobos SDR** receiver by Rig Expert Ukraine Ltd. It wraps the [libfobos](https://github.com/rigexpert/libfobos) C library to stream samples over USB into GNU Radio flowgraphs.

Product page: https://rigexpert.com/en/products/kits-en/fobos-sdr/

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [Architecture](#architecture)
3. [Data Flow](#data-flow)
4. [Operating Modes](#operating-modes)
5. [Warmup Sequence](#warmup-sequence)
6. [Message Ports & Protocol](#message-ports--protocol)
7. [Phase Exclusion](#phase-exclusion)
8. [Frequency Cycle (gr-rigexpert-extra)](#frequency-cycle-gr-rigexpert-extra)
9. [Buffer Architecture](#buffer-architecture)
10. [Threading Model](#threading-model)
11. [Hardware Interface (libfobos)](#hardware-interface-libfobos)
12. [GRC Block Parameters](#grc-block-parameters)
13. [Build System & Dependencies](#build-system--dependencies)
14. [Key Files Reference](#key-files-reference)

---

## Project Structure

```
gr-rigexpert/
├── CMakeLists.txt                          # Top-level build config
├── README.md                               # Basic usage instructions
├── LICENSE                                 # GPL 3.0
├── include/gnuradio/RigExpert/
│   ├── api.h                               # DLL export macros
│   └── fobos_sdr.h                         # Public block interface
├── lib/
│   ├── CMakeLists.txt                      # Library build + linking
│   ├── fobos_sdr_impl.h                    # Private implementation header
│   └── fobos_sdr_impl.cc                   # Core implementation (~470 lines)
├── grc/
│   └── RigExpert_fobos_sdr.block.yml       # GRC block definition
├── python/RigExpert/
│   ├── __init__.py                         # Python module init
│   ├── qa_fobos_sdr.py                     # Unit tests
│   └── bindings/
│       ├── fobos_sdr_python.cc             # pybind11 class bindings
│       └── python_bindings.cc              # pybind11 module entry
├── cmake/Modules/                          # CMake helpers
└── docs/                                   # Doxygen generation
```

---

## Architecture

The module provides a single GNU Radio block: **`RigExpert::fobos_sdr`**.

- **Block type**: `gr::sync_block` (synchronous source — 0 inputs, 1 complex output)
- **Namespace**: `gr::RigExpert`
- **Public class**: `include/gnuradio/RigExpert/fobos_sdr.h`
- **Implementation**: `lib/fobos_sdr_impl.h` / `lib/fobos_sdr_impl.cc`

The block uses the factory pattern — users call `fobos_sdr::make(...)` which returns a `shared_ptr<fobos_sdr>`. The actual constructor lives in `fobos_sdr_impl` (PIMPL-style, standard for GNU Radio OOT modules).

---

## Data Flow

```
┌──────────────┐     USB      ┌──────────────────┐
│  Fobos SDR   │─────────────→│  libfobos async   │
│  (hardware)  │              │  read callback     │
└──────────────┘              └────────┬───────────┘
                                       │
                    ┌──────────────────┼──────────────────┐
                    ▼                                     ▼
          ┌─────────────────┐                   ┌─────────────────┐
          │  Circular Buffer │                   │  Vector Queue    │
          │  (32 slots)      │                   │  (freq, samples) │
          └────────┬────────┘                   └────────┬────────┘
                   │                                     │
                   ▼                                     ▼
          ┌─────────────────┐                   ┌─────────────────┐
          │  work() function │                   │  vector_loop()   │
          │  (GNU Radio      │                   │  thread           │
          │   scheduler)     │                   └────────┬────────┘
          └────────┬────────┘                            │
                   │                                     │
                   ▼                                     ▼
          Stream output (out0)                  Message output ("vector")
          Complex IQ samples                    {channel: freq, samples: [...]}
```

All received USB buffers are written to the **circular buffer** and forwarded to `work()` as the complex IQ stream output. During scanning mode, valid channel buffers are also copied to the **vector queue** and published on the `"vector"` message port with their frequency tag.

---

## Operating Modes

### 1. Single-Frequency Mode

When `frequency` is set to a positive value (e.g. `1.0e9`), the receiver tunes to that frequency and streams continuous IQ samples from `out0`.

```
set_frequency(1.0e9)
  → fobos_sdr_stop_scan()   // stop any active scan
  → fobos_sdr_set_frequency(1.0e9)
```

### 2. Scanning Mode

When `frequency = -1`, the block enters scanning mode. It sweeps through a list of frequencies loaded from `frequency_cycle` (see [Frequency Cycle](#frequency-cycle-gr-rigexpert-extra)).

```
set_frequency(-1)
  → start_scan()
    → fobos_sdr_start_scan(dev, frequencies.data(), frequencies.size())
```

In scanning mode:
- The hardware cycles through the frequency list automatically
- Each buffer in the USB callback is tagged with a channel index (`fobos_sdr_get_scan_index()`)
- Channel index `-1` means the hardware is transitioning between frequencies — the buffer is zeroed
- Valid channel buffers are pushed to the vector queue, and the `vector_loop` thread publishes them as PMT messages on the `"vector"` port
- After sweeping the last frequency, `_update_list` is set to `true`, triggering a frequency list refresh on the next vector loop iteration

---

## Warmup Sequence

The receiver uses a warmup period for 1-PPS (pulse-per-second) detection before entering its configured operating mode.

```
Constructor
  │
  ├─ _warmup = true
  ├─ set_frequency(warmup_frequency)   // default: 100 MHz
  ├─ set_lna_gain(0)                   // gains OFF during warmup
  └─ set_vga_gain(0)

  ... receiver streams at warmup frequency, external logic detects 1-PPS ...

Phase message arrives on "phase" port
  │
  ├─ _warmup = false
  ├─ _frequencies = _cycle.pull()      // load frequency list from CSV
  ├─ set_lna_gain(_lna_gain)           // apply configured gains
  ├─ set_vga_gain(_vga_gain)
  └─ set_frequency(_frequency)         // switch to target (-1 = scan, or fixed freq)
```

Key points:
- During warmup, control messages are stored but **not applied** to hardware
- Gains are intentionally kept at zero to avoid transient overload during calibration
- The warmup frequency is separate from the operational frequency to isolate 1-PPS detection

---

## Message Ports & Protocol

The block has 2 input message ports and 1 output message port, all using GNU Radio's PMT (Polymorphic Message Transport).

### Input: `"phase"` (integer)

Signals the detected phase from an external 1-PPS synchronization source.

| Field | Type | Description |
|-------|------|-------------|
| value | PMT integer (uint64) | Sample number within 1-second cycle (0 to sample_rate-1) |

**Effect**: Ends warmup mode, loads frequency list, applies stored gains and frequency.

### Input: `"control"` (dictionary)

Runtime parameter control. Only applied when `_warmup == false`.

| Key | PMT Type | Description |
|-----|----------|-------------|
| `"freq"` | number (double) | Frequency in Hz. -1 starts scanning |
| `"lna"` | number (long) | LNA gain level (0-3) |
| `"vga"` | number (double) | VGA gain level (0-15) |

All keys are optional — only present keys are applied.

### Output: `"vector"` (dictionary)

Published during scanning mode for each valid frequency buffer.

| Key | PMT Type | Description |
|-----|----------|-------------|
| `"channel"` | double | Frequency in Hz |
| `"samples"` | c32vector | Complex IQ samples for that frequency |

---

## Phase Exclusion

The phase exclusion mechanism filters out samples that fall near a phase marker (e.g., 1-PPS pulse edge), avoiding transient artifacts.

### Configuration

Two constructor parameters define the exclusion window:
- `exclude_before_phase` — start of exclusion zone (in samples)
- `exclude_after_phase` — end of exclusion zone (in samples)

These are converted to a center-offset representation:
```cpp
_exclude_half_width = (exclude_after_phase - exclude_before_phase) / 2;
_exclude_offset     = exclude_before_phase + _exclude_half_width;
```

### How It Works

In the USB callback, for each scanning buffer:

1. Track `_sample_count` modulo `_sample_rate` to know the position within the current second
2. Calculate distance `dist` from the current buffer to the phase marker
3. Only enqueue the buffer for vector output if `dist > _exclude_half_width`

This ensures samples contaminated by the 1-PPS pulse transition are discarded from scanning results, while still forwarding them to the stream output (out0).

---

## Frequency Cycle (gr-rigexpert-extra)

The block depends on `gr-rigexpert-extra`, a separate library providing the `extra::frequency_cycle` class.

```cpp
extra::frequency_cycle _cycle;  // initialized with (table_path, pattern)
```

### Purpose

Manages a dynamic, prioritized list of frequencies loaded from a CSV file (`table_path`). The `pattern` parameter selects which subset/ordering of frequencies to use.

### Usage

- At construction: `_cycle(table_path, pattern)` initializes the cycle
- On phase message: `_frequencies = _cycle.pull()` loads the current frequency list
- During scanning: after sweeping all frequencies, `update_list()` calls `_cycle.pull()` again to get an updated list (priorities may change dynamically)

### Scan Update Flow

```
Last channel scanned → _update_list = true
  → vector_loop detects flag
    → update_list()
      → fobos_sdr_stop_scan()
      → _frequencies = _cycle.pull()    // fresh list
      → fobos_sdr_start_scan(dev, _frequencies.data(), size)
```

---

## Buffer Architecture

### Circular Buffer (RX)

The module uses a ring buffer of 32 pre-allocated float arrays for streaming IQ data from USB to the `work()` function.

```
Buffer count:  32 (fixed)
Buffer size:   8192 * buf_len samples (each sample = 2 floats: I + Q)
Memory:        32 * 8192 * buf_len * 2 * sizeof(float) bytes
```

**Write side** (USB callback thread):
- Writes to `_rx_bufs[_rx_idx_w]`
- Advances `_rx_idx_w = (_rx_idx_w + 1) % 32` when buffer is full
- Increments `_rx_filled`

**Read side** (`work()` function):
- Reads from `_rx_bufs[_rx_idx_r]` at position `_rx_pos_r`
- May consume partial buffers (GNU Radio requests variable `noutput_items`)
- Decrements `_rx_filled` when a full buffer is consumed

**Overrun**: If `_rx_filled == 32`, the callback prints `"OVERRUN!!!"` and drops the buffer.

### Vector Queue (Scanning)

A `std::queue<std::pair<double, std::vector<gr_complex>>>` holds (frequency, sample_vector) pairs for the vector output thread.

---

## Threading Model

The block runs **3 threads**:

### 1. USB Read Thread (`thread_proc`)

```
Started in constructor → runs fobos_sdr_read_async() (blocking)
  → invokes read_samples_callback() per USB buffer
  → on exit: sets _running = false, notifies all waiters
```

- Calls `fobos_sdr_read_async(dev, callback, this, 16, buf_len)` — 16 USB URBs queued
- The callback fills the circular buffer and optionally the vector queue
- Terminated via `fobos_sdr_cancel_async()` in the destructor

### 2. Vector Output Thread (`vector_loop`)

```
While _vec_running:
  wait on _vec_cond (until queue non-empty or shutdown)
  if _update_list → stop scan, pull new frequencies, restart scan
  else → pop (freq, samples), publish PMT dict on "vector" port
```

### 3. GNU Radio Scheduler (calls `work()`)

```
wait on _rx_cond (until circular buffer has data or shutdown)
copy samples from circular buffer → output_items[0]
return number of produced samples
```

### Synchronization

| Resource | Mutex | Condition Variable |
|----------|-------|--------------------|
| Circular buffer (`_rx_bufs`, `_rx_filled`, indices) | `_rx_mutex` | `_rx_cond` |
| Vector queue (`_vec_queue`, `_update_list`) | `_vec_mutex` | `_vec_cond` |

### Shutdown Sequence (destructor)

1. `fobos_sdr_cancel_async()` — unblocks the USB read thread
2. Lock `_rx_mutex`, set `_running = false`, notify `_rx_cond`
3. Join USB thread
4. Lock `_vec_mutex`, set `_vec_running = false`, notify `_vec_cond`
5. Join vector thread
6. `stop_scan()` + `fobos_sdr_close()`
7. Free all buffer memory

---

## Hardware Interface (libfobos)

Communication with the Fobos SDR goes through the `libfobos` C library over USB (via libusb-1.0).

### Device Lifecycle

```cpp
// Initialization (constructor)
fobos_sdr_get_api_info(lib_version, drv_version);
int count = fobos_sdr_get_device_count();
fobos_sdr_open(&_dev, index);
fobos_sdr_get_board_info(_dev, hw_rev, fw_ver, manufacturer, product, serial);

// Configuration
fobos_sdr_set_frequency(_dev, frequency);
fobos_sdr_set_samplerate(_dev, samplerate);
fobos_sdr_set_lna_gain(_dev, lna_gain);
fobos_sdr_set_vga_gain(_dev, vga_gain);
fobos_sdr_set_direct_sampling(_dev, mode);
fobos_sdr_set_clk_source(_dev, source);

// Streaming
fobos_sdr_read_async(_dev, callback, user_ptr, 16, buf_len);  // start
fobos_sdr_cancel_async(_dev);                                   // stop

// Scanning
fobos_sdr_start_scan(_dev, freq_array, freq_count);
fobos_sdr_stop_scan(_dev);
fobos_sdr_is_scanning(dev);         // query state in callback
fobos_sdr_get_scan_index(dev);      // current channel (-1 = transitioning)

// Cleanup
fobos_sdr_close(_dev);
```

### USB Callback Signature

```cpp
static void read_samples_callback(
    float *buf,                        // interleaved I/Q float pairs
    uint32_t buf_length,               // number of complex samples
    struct fobos_sdr_dev_t* sender,    // device handle
    void *user                         // user pointer (this)
);
```

---

## GRC Block Parameters

Defined in `grc/RigExpert_fobos_sdr.block.yml`. The block appears in GRC under the **[RigExpert]** category as "Fobos SDR source".

| Parameter | ID | Type | Default | Options / Notes |
|-----------|----|------|---------|-----------------|
| Device # | `index` | int | 0 | Select device when multiple are connected |
| Frequency priority table | `table_path` | string | `"./table.csv"` | CSV file for frequency_cycle |
| Prioritized list pattern | `pattern` | string | `"0"` | Pattern selector for frequency subset |
| Initial frequency (Hz) | `Frequency` | real | -1 | -1 = scanning mode, positive = single freq |
| Frequency during 1-PPS detection (Hz) | `warmup_frequency` | real | 100e6 | Used during warmup only |
| Sample rate (Hz) | `samplerate` | real | 10e6 | 50M, 40M, 32M, 25M, 20M, 16M, 12.5M, 10M, 8M |
| LNA gain | `lna_gain` | int | 0 | 0=none, 1=0dB, 2=15dB, 3=30dB |
| VGA gain | `vga_gain` | int | 0 | 0-15 → 0dB to 30dB in 2dB steps |
| Sampling | `direct_sampling` | int | 0 | 0=RF input, 1=HF1/2 direct |
| Clock source | `clock_source` | int | 0 | 0=internal, 1=external 10MHz |
| Buffer length | `buf_len` | int | 100 | Multiplier: actual buffer = 8192 * buf_len samples |
| Exclude before phase | `exclude_before_phase` | int | 0 | Phase exclusion window start (samples) |
| Exclude after phase | `exclude_after_phase` | int | 0 | Phase exclusion window end (samples) |

### Runtime Callbacks (GRC)

These parameters can be changed while the flowgraph is running:
- `set_samplerate(samplerate)`
- `set_lna_gain(lna_gain)`
- `set_vga_gain(vga_gain)`
- `set_direct_sampling(direct_sampling)`
- `set_clock_source(clock_source)`

### I/O Ports

| Port | Direction | Domain | Type | Description |
|------|-----------|--------|------|-------------|
| `phase` | input | message | PMT integer | Phase synchronization from 1-PPS detector |
| `control` | input | message | PMT dict | Runtime freq/gain control |
| `out0` | output | stream | complex | Continuous IQ sample stream |
| `vector` | output | message | PMT dict | Per-frequency sample vectors (scanning mode) |

All ports are optional in the GRC definition.

---

## Build System & Dependencies

### Requirements

| Dependency | Version | Purpose |
|-----------|---------|---------|
| GNU Radio | >= 3.10 | Block framework, PMT, threading, pybind11 |
| libfobos | (latest) | Fobos SDR hardware API |
| libusb | >= 1.0.25 | USB communication backend |
| gr-rigexpert-extra | (internal) | `frequency_cycle` class for dynamic frequency lists |
| CMake | >= 3.8 | Build system |
| pybind11 | (via GNU Radio) | Python bindings |

### Build

```bash
mkdir build && cd build
cmake ..
make
sudo make install
```

### Library Linking

The shared library `gnuradio-RigExpert` links against:
- `libusb-1.0`
- `libfobos_sdr`
- `gnuradio::gnuradio-runtime`
- `gr-rigexpert-extra`

### Updating Bindings Hash

When the public header `fobos_sdr.h` changes, the pybind11 binding hash must be updated:

```bash
# In the build directory:
md5sum ../include/gnuradio/RigExpert/fobos_sdr.h
# Copy the hash into python/RigExpert/bindings/fobos_sdr_python.cc
# (BINDTOOL_HEADER_FILE_HASH line)
```

Or use: `gr_modtool bind -u <module_name>`

---

## Key Files Reference

| File | Lines | Description |
|------|-------|-------------|
| `lib/fobos_sdr_impl.cc` | ~470 | Core implementation: constructor, work(), callbacks, scanning, all setters |
| `lib/fobos_sdr_impl.h` | ~115 | Private class: all member variables, thread functions, message handlers |
| `include/gnuradio/RigExpert/fobos_sdr.h` | ~77 | Public interface: factory method, virtual setters |
| `grc/RigExpert_fobos_sdr.block.yml` | ~113 | GRC block definition: parameters, ports, callbacks |
| `python/RigExpert/bindings/fobos_sdr_python.cc` | ~94 | pybind11 bindings: exposes class + methods to Python |
| `CMakeLists.txt` | ~154 | Top-level: GNU Radio 3.10 requirement, gr-rigexpert-extra dependency |
| `lib/CMakeLists.txt` | ~102 | Library build: links libusb, libfobos, gnuradio-runtime, gr-rigexpert-extra |
