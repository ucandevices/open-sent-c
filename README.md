# open-sent-c

[![tests](https://github.com/ucandevices/open-sent-c/actions/workflows/test.yml/badge.svg)](https://github.com/ucandevices/open-sent-c/actions/workflows/test.yml)

A portable C implementation of the SAE J2716 SENT (Single Edge Nibble Transmission)
protocol for embedded systems and host-side tooling.

## Layout

```
.
├── *.h                 public API headers
└── implementation/     library sources (.c) and platform ports
```

The protocol layer (`sent_protocol`, `sent_encoder`, `sent_decoder`, `sent_crc`)
is MCU-agnostic and uses the `hal.h` function-pointer interface to talk to
hardware. Platform-specific ports live alongside the protocol code:

- `hal_stm32f042.{h,c}` — STM32F042 (TIM input-capture RX, software TX) port
- `hal_host.{h,c}` — host-side stub for unit tests

## Architecture

### Layers

```
┌─────────────────────────────────────────────────────┐
│                  Application / Bridge               │
│   (configures, drives mode_manager, reads frames)   │
├─────────────────────────────────────────────────────┤
│              Protocol layer (MCU-agnostic)          │
│  sent_protocol  sent_encoder  sent_decoder  sent_crc│
├─────────────────────────────────────────────────────┤
│           HAL interface  (hal.h / hal_config.h)     │
│        sent_rx_hal_t          sent_tx_hal_t         │
│     (function-pointer vtables, no OS dependency)    │
├──────────────────────┬──────────────────────────────┤
│  STM32F042 port      │  Host (x86/Linux) port       │
│  hal_stm32f042.{h,c} │  hal_host.{h,c}              │
│  TIM input-capture   │  pthread ring-buffer + store  │
│  + software TX ISR   │  (unit tests / simulation)    │
└──────────────────────┴──────────────────────────────┘
```

### Protocol layer

All encoding and decoding operates on **ticks**, not microseconds, keeping the
logic independent of the physical tick period:

- **`sent_crc`** — SAE J2716 CRC-4 (polynomial 0x0D). Supports both
  `DATA_ONLY` and `STATUS_AND_DATA` modes and configurable init seeds
  (0x05 legacy / 0x03 APR2016).
- **`sent_encoder`** — `sent_frame_t` → tick-interval array →
  absolute µs timestamps. Used by both the HAL TX path and test fixtures.
- **`sent_decoder`** — absolute µs timestamps → `sent_frame_t`. Tolerates
  ±35% per-interval jitter and scans all candidate sync positions in the
  window, so a leading extra edge from sync detection never blocks decoding.
- **`sent_protocol`** — shared types (`sent_frame_t`, `sent_config_t`),
  J2716 constants, and nibble pack/unpack helpers.

### HAL interface

`hal.h` defines two vtable structs — `sent_rx_hal_t` and `sent_tx_hal_t` —
each holding a `void* context` and a set of function pointers. The protocol
layer calls through these pointers without knowing the underlying hardware.
A port fills the struct once at startup (via `sent_xxx_make_rx/tx_hal()`)
and passes it to the application.

Optional slots (`set_data_nibbles`, `set_sync_min_us`, `set_tick_x10_us`)
allow the application to reconfigure a running HAL without restarting it;
backends that do not support a capability leave the pointer NULL.

### RX data flow

```
Falling edge (hardware)
   │
   ▼ ISR
sent_stm32f042_rx_on_capture_edge_isr()
   │  convert raw counter → µs (Q12 fixed-point, no division)
   │  sync detection: long interval resets batch, seeds ts[0]
   │  accumulate active_timestamps_us[]
   │  when batch full → push to ready ring buffer
   ▼ main loop
sent_rx_hal_t.poll_timestamps_us()   ← dequeue one batch
   │
   ▼
sent_decode_from_timestamps_us()     ← protocol layer
   │
   ▼
sent_frame_t                         ← application
```

### TX data flow

```
Application
   │
   ▼
sent_tx_hal_t.submit_frame()
   │  encode frame → tick intervals (sent_build_intervals_ticks)
   │  expand each interval into [LOW, HIGH] toggle pair
   │  publish flat intervals[] array; write count last (store barrier)
   ▼ ISR (TIM14 compare)
sent_stm32f042_tx_pop_next_interval_ticks_from_isr()
   │  pop one toggle duration per compare event
   │  toggle pin; reload timer with next duration
   ▼
SENT output pin
```

### Design constraints

| Constraint | How it is met |
|---|---|
| No dynamic allocation | all state is in caller-supplied structs |
| ISR-safe on Cortex-M0 | `volatile` on shared counters; `count` written last as a publish barrier; `uint8_t`/`uint32_t` reads/writes are atomic on M0 |
| No FPU required | tick→µs conversion uses Q12 integer fixed-point |
| Portable C99 | protocol layer compiles on any C99 toolchain; platform code is isolated behind `hal_config.h` guards |
| 100% test coverage | host HAL + test suite exercise every reachable line and branch |

## Public API

- `sent_protocol.h` — frame/config types, nibble packing
- `sent_encoder.h` — frame → tick intervals → microsecond timestamps
- `sent_decoder.h` — microsecond timestamps → decoded frame
- `sent_crc.h` — SAE J2716 4-bit CRC
- `mode_manager.h` — RX/TX/STOPPED state machine + statistics
- `hal.h` / `hal_config.h` — RX/TX HAL interfaces a port must implement

## Usage

The library has no build system of its own — drop the headers on your
include path and compile the relevant `.c` files in `implementation/`
into your project. Pick the HAL port that matches your target (or write
your own against `hal.h`).

## Testing

Unit tests live in `implementation/Tests/` and are built with plain GCC
(`-std=c99`). They use a single-header framework (`test.h`) with no external
dependencies.

```sh
cd implementation/Tests
make run       # build and run all tests
make coverage  # run tests + enforce 100% line and branch coverage
```

Coverage is measured with **gcov/gcovr**. The `make coverage` target requires
both tools to be on `PATH` (`apt-get install gcc gcovr` on Debian/Ubuntu) and
fails if either metric drops below 100%.

### CI

A GitHub Actions workflow (`.github/workflows/test.yml`) runs on every push
and pull request to `master`/`main`:

1. Builds the test binary with `-O0 --coverage`.
2. Runs all 126 tests — the job fails on any failing test.
3. Generates a gcovr XML + text report and uploads it as a build artifact.

Genuinely unreachable branches (dead code protected by configuration
invariants) are annotated with `/* GCOV_EXCL_BR_LINE */` or suppressed via
gcovr pattern filters so they do not inflate the denominator.

### Coverage artifacts

After each CI run the coverage report is uploaded as a GitHub Actions artifact
named **`coverage`** and retained for 90 days. To download it:

1. Open the [Actions tab](https://github.com/ucandevices/open-sent-c/actions).
2. Click any completed workflow run.
3. Scroll to the **Artifacts** section at the bottom of the page and download `coverage.zip`.

The zip contains:
- `coverage.txt` — human-readable summary table
- `coverage.xml` — Cobertura XML (compatible with SonarQube, Codecov, VS Code extensions)

## Logic 2 signal analysis

`logic_plugin/` contains two tools for decoding SENT frames from a
[Saleae Logic 2](https://www.saleae.com/) capture.

**Standalone** — decode a `.sal` file directly, no GUI needed:
```sh
python logic_plugin/decode_sent_sal.py Session0.sal
```

**Logic 2 HLA plugin** (`logic_plugin/SENT/`) — load via Extensions →
Load Existing Extension → select `extension.json`. Add **Async Serial**
(10 Mbaud, 8 bits/frame, non-inverted) on the SENT channel first, then stack
**SENT Protocol (SAE J2716)** on top with `tick_us=3.0` and `num_nibbles=6`
(adjust for your sensor). Capture at ≥ 10 MHz for reliable CRC decoding.

## Reference integration

[**SENTToUSB**](https://github.com/ucandevices/SENTToUSB) is a reference
firmware project that integrates this library on an STM32F042 to expose a
SENT sensor over USB CDC. It demonstrates the full stack:
HAL port (`hal_stm32f042`) wired to real hardware timers and USB transport.

## License

MIT — see [LICENSE](LICENSE).
