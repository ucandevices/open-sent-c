# open-sent-c

[![tests](https://github.com/ucandevices/open-sent-c/actions/workflows/test.yml/badge.svg)](https://github.com/ucandevices/open-sent-c/actions/workflows/test.yml)

Portable C implementation of the SAE J2716 SENT (Single Edge Nibble
Transmission) protocol. MCU-agnostic protocol layer + a small HAL vtable
(`hal.h`) implemented per target; ports for STM32F042 and a host stub
ship in `implementation/`.

## Layout

```
*.h                  public API headers
implementation/      sources, HAL ports, tests
logic_plugin/        Saleae Logic 2 decoder
```

## Public API

- `sent_protocol.h` — frame/config types, nibble packing
- `sent_encoder.h` — frame → tick intervals → µs timestamps
- `sent_decoder.h` — µs timestamps → decoded frame
- `sent_crc.h` — J2716 4-bit CRC (fast frame) and 6-bit CRC (Enhanced Serial)
- `sent_slow_channel.h` — slow (serial) channel reassembler
- `mode_manager.h` — RX/TX/STOPPED state machine + statistics
- `hal.h` / `hal_config.h` — HAL vtable a port must implement

All encoding/decoding works in **ticks**, not µs, so the logic is
independent of the physical tick period.

## Fast channel

A J2716 fast frame is a sequence of falling-edge intervals:

```
sync (56 ticks) │ status (4b) │ data×N (1–6 nibbles) │ CRC (4b) │ pause
```

Each nibble is encoded as `ticks = value + 12`, so a nibble pulse is
12–27 ticks. CRC is 4-bit J2716 (`sent_crc.h`); `sent_config_t` selects
`DATA_ONLY` vs `STATUS_AND_DATA` and seed `0x03` (APR2016) vs `0x05`
(legacy).

**Encode** (`sent_encoder.h`) — `sent_frame_t` →
`sent_build_intervals_ticks()` → tick array →
`sent_intervals_to_timestamps_us()` → absolute µs edge timestamps. The
HAL TX path consumes the tick array; tests use the timestamp array to
feed the decoder.

**Decode** (`sent_decoder.h`) — absolute falling-edge timestamps →
`sent_decode_from_timestamps_us()` → `sent_frame_t`. Tolerates ±35%
per-interval jitter and scans every candidate sync position in the
window, so a stray leading edge from sync detection does not block
decoding. Returns a `sent_decode_status_t` (`OK` / `SYNC_ERROR` /
`CRC_ERROR` / `SHAPE_ERROR`) even on failure.

```c
sent_frame_t frame;
sent_decode_status_t st;
if (sent_decode_from_timestamps_us(&cfg, ts, n, &frame, &st)) {
    /* frame.status, frame.data_nibbles[0..count-1], frame.crc valid */
}
```

## Slow channel

Bits 2 and 3 of every fast-frame status nibble carry the J2716 slow
channel. Feed each fast frame's status nibble in; the call returns
`true` and fills `out_message` once a complete CRC-valid message arrives.

```c
sent_slow_channel_t slow;
sent_slow_channel_init(&slow);

sent_slow_message_t msg;
if (sent_slow_channel_process_status(&slow, frame.status_nibble, &msg)) {
    /* msg.format / msg.message_id / msg.data are valid */
}
```

Decoded formats (see `sent_slow_channel.h` for the full enum/struct):

| `format`                         | ID bits | Data bits | CRC                       |
|----------------------------------|---------|-----------|---------------------------|
| `SENT_SLOW_FORMAT_SHORT`         | 4       | 8         | 4-bit J2716 over id+data  |
| `SENT_SLOW_FORMAT_ENHANCED_12_8` | 8       | 12        | 6-bit J2716 over 24-bit body |
| `SENT_SLOW_FORMAT_ENHANCED_16_4` | 4       | 16        | 6-bit J2716 over 24-bit body |

The Enhanced sync pattern (`0x7E` on bit 3) takes priority and resets
any in-flight Short Serial state, so a sensor switching modes recovers
without an explicit `reset()`.

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

## Build / use

No build system of its own — put the headers on your include path,
compile the relevant `.c` files in `implementation/`, and pick (or
write) a HAL port against `hal.h`.

## Tests

```sh
cd implementation/Tests
make run        # build + run
make coverage   # run + enforce 100% line/branch coverage (needs gcov + gcovr)
```

CI (`.github/workflows/test.yml`) runs the suite on every push/PR and
uploads a `coverage` artifact (txt + Cobertura XML, 90-day retention).

## Logic 2 decoder

`logic_plugin/decode_sent_sal.py` decodes a `.sal` capture standalone.
`logic_plugin/SENT/` is a Logic 2 HLA — stack it on an Async Serial
analyzer (10 Mbaud, 8 bits, non-inverted) on the SENT channel, capture
at ≥ 10 MHz.

## License

MIT — see [LICENSE](LICENSE).
