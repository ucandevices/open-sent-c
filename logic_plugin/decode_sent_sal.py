"""
Standalone SENT (SAE J2716) decoder for Saleae Logic 2 .sal captures.

Reads the .sal file directly – no Logic 2 GUI, no UART/Async Serial plugin.

Usage
-----
    python decode_sent_sal.py [options] [Session0.sal]

    Session0.sal is looked for next to this script if not specified.

Options
    --channel N      Digital channel index (default: 2)
    --tick-us F      Nominal tick period µs (default: 3.0)
    --nibbles N      Data nibbles per frame (default: 6)
    --crc-mode M     DATA_ONLY | STATUS_AND_DATA (default: DATA_ONLY)
    --seed N         CRC seed: 3 or 5 (default: 3)
    --csv PATH       Write decoded frames to a CSV file

.sal binary format (Logic 2 version 3, type 100 digital)
---------------------------------------------------------
Offset  0- 7 : magic  "<SALEAE>"
Offset  8-11 : version uint32 = 3
Offset 12-15 : type    uint32 = 100  (digital)
Offset 16-19 : initial_state uint32  (0 or 1)
Offset 20-76 : header fields (timestamps, channel metadata – unused here)
Offset 77+   : run-length encoded transitions

Run-length encoding
  byte < 0x40          : 1-byte run, length = byte + 1         (1..64 samples)
  byte >= 0x40         : 2-byte run, length = (byte-0x40)*128
                         + next_byte + 1                 (65..24576 samples)
Each run alternates the signal state starting from initial_state.
"""

import argparse
import csv
import json
import os
import struct
import sys
import zipfile

# ── CRC-4 SAE J2716 ──────────────────────────────────────────────────────────
_CRC_LUT = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]

def _crc4(data_nibbles, status_nibble, mode, seed):
    crc = seed
    if mode == "STATUS_AND_DATA":
        crc = _CRC_LUT[crc ^ (status_nibble & 0xF)]
    for n in data_nibbles:
        crc = _CRC_LUT[crc ^ (n & 0xF)]
    return crc


# ── Binary .sal reader ───────────────────────────────────────────────────────
_MAGIC       = b"<SALEAE>"
_DATA_OFFSET = 77   # fixed header size for version 3, type 100

def _read_sal_channel(sal_path, channel):
    """Return (falling_edge_times_us, sample_rate_hz) from a .sal capture."""
    with zipfile.ZipFile(sal_path) as zf:
        # --- sample rate from meta.json ---
        meta       = json.loads(zf.read("meta.json"))
        sr_section = (meta.get("data", {})
                          .get("legacySettings", {})
                          .get("sampleRate", {}))
        sample_rate = int(sr_section.get("digital", 500_000))

        # --- raw binary for the requested channel ---
        bin_name = f"digital-{channel}.bin"
        if bin_name not in zf.namelist():
            raise FileNotFoundError(
                f"Channel {channel} not found in {sal_path}. "
                f"Available: {zf.namelist()}"
            )
        raw = zf.read(bin_name)

    # sanity check
    if raw[:8] != _MAGIC:
        raise ValueError(f"Unexpected magic in {bin_name}")
    version     = struct.unpack_from("<I", raw, 8)[0]
    type_val    = struct.unpack_from("<I", raw, 12)[0]
    init_state  = struct.unpack_from("<I", raw, 16)[0]

    if version != 3 or type_val != 100:
        raise ValueError(
            f"{bin_name}: unexpected version={version} type={type_val} "
            f"(expected v3 type 100)"
        )

    # --- decode run-length data ---
    pos          = _DATA_OFFSET
    current_samp = 0
    state        = init_state & 1
    falling_us   = []             # falling-edge timestamps in µs

    us_per_samp = 1_000_000.0 / sample_rate

    while pos < len(raw):
        b = raw[pos]
        if b < 0x40:
            run = b + 1
            pos += 1
        else:
            if pos + 1 >= len(raw):
                break
            n   = raw[pos + 1]
            run = (b - 0x40) * 128 + n + 1
            pos += 2

        current_samp += run
        next_state    = 1 - state

        if state == 1 and next_state == 0:          # HIGH → LOW (falling edge)
            falling_us.append(current_samp * us_per_samp)

        state = next_state

    return falling_us, sample_rate


# ── SENT decoder ─────────────────────────────────────────────────────────────
_SYNC_TICKS       = 56
_SYNC_MIN_TICKS   = int(_SYNC_TICKS * 0.65)   # 36
_SYNC_MAX_TICKS   = int(_SYNC_TICKS * 1.35)   # 75
_NIBBLE_MIN       = 12
_NIBBLE_MAX       = 27
_GAP_MIN_US       = 30.0   # intervals shorter than this are noise / artefacts


def decode_sent(falling_us, nom_tick_us, num_nibbles, crc_mode, seed):
    """Decode SENT frames from a list of falling-edge timestamps (µs).

    Returns a list of frame dicts with keys:
        frame_idx, sync_us, tick_us, status, data (list), crc_recv, crc_calc, ok
    """
    if len(falling_us) < 2:
        return []

    intervals_us = [falling_us[i + 1] - falling_us[i]
                    for i in range(len(falling_us) - 1)]

    frames        = []
    state         = "WAIT_SYNC"
    tick          = nom_tick_us
    status        = None
    data          = []
    sync_us       = None
    reset_count   = 0

    i = 0
    while i < len(intervals_us):
        iv    = intervals_us[i]
        i    += 1

        if iv < _GAP_MIN_US:
            continue

        ticks = round(iv / tick)

        # ── WAIT_SYNC ──────────────────────────────────────────────────────
        if state == "WAIT_SYNC":
            if _SYNC_MIN_TICKS <= ticks <= _SYNC_MAX_TICKS:
                tick   = iv / _SYNC_TICKS
                sync_us = iv
                state  = "STATUS"
                status = None
                data   = []
            continue

        # ── STATUS ─────────────────────────────────────────────────────────
        if state == "STATUS":
            if _SYNC_MIN_TICKS <= ticks <= _SYNC_MAX_TICKS:
                # back-to-back sync (pause+sync collapsed into one interval)
                tick   = iv / _SYNC_TICKS
                sync_us = iv
                data   = []
                status = None
                continue
            if ticks > _SYNC_MAX_TICKS:
                state = "WAIT_SYNC"
                tick  = nom_tick_us
                continue
            if not (_NIBBLE_MIN <= ticks <= _NIBBLE_MAX):
                reset_count += 1
                state = "WAIT_SYNC"
                tick  = nom_tick_us
                continue
            status = ticks - 12
            state  = "DATA"
            continue

        # ── DATA ───────────────────────────────────────────────────────────
        if state == "DATA":
            if _SYNC_MIN_TICKS <= ticks <= _SYNC_MAX_TICKS:
                tick   = iv / _SYNC_TICKS
                sync_us = iv
                state  = "STATUS"
                data   = []
                status = None
                continue
            if not (_NIBBLE_MIN <= ticks <= _NIBBLE_MAX):
                reset_count += 1
                state = "WAIT_SYNC"
                tick  = nom_tick_us
                continue
            data.append(ticks - 12)
            if len(data) >= num_nibbles:
                state = "CRC"
            continue

        # ── CRC ────────────────────────────────────────────────────────────
        if state == "CRC":
            if not (_NIBBLE_MIN <= ticks <= _NIBBLE_MAX):
                reset_count += 1
                state = "WAIT_SYNC"
                tick  = nom_tick_us
                continue
            crc_recv = ticks - 12
            crc_calc = _crc4(data,
                             status if status is not None else 0,
                             crc_mode, seed)
            frames.append({
                "frame_idx": len(frames),
                "sync_us":   sync_us,
                "tick_us":   tick,
                "status":    status,
                "data":      list(data),
                "crc_recv":  crc_recv,
                "crc_calc":  crc_calc,
                "ok":        crc_recv == crc_calc,
            })
            state  = "STATUS"
            status = None
            data   = []
            continue

    return frames, reset_count


# ── Presentation ─────────────────────────────────────────────────────────────

def print_results(frames, reset_count):
    if not frames:
        print("No SENT frames decoded.")
        return

    hdr = f"{'#':>5}  {'sync_us':>8}  {'tick_us':>8}  {'ST':>3}  {'Data':>12}  {'CRC':>3}  Result"
    print(hdr)
    print("-" * len(hdr))

    ok_168 = err_168 = ok_170 = err_170 = ok_other = err_other = 0
    for f in frames:
        data_str = "".join(f"{d:X}" for d in f["data"])
        result   = "OK" if f["ok"] else f"FAIL(calc={f['crc_calc']:X})"
        print(f"{f['frame_idx']:>5}  {f['sync_us']:>8.1f}  {f['tick_us']:>8.4f}  "
              f"{f['status']:>3X}  {data_str:>12}  {f['crc_recv']:>3X}  {result}")
        s = round(f["sync_us"])
        if s == 168:
            (ok_168 if f["ok"] else err_168).__class__  # trick; use below
            if f["ok"]: ok_168 += 1
            else:       err_168 += 1
        elif s == 170:
            if f["ok"]: ok_170 += 1
            else:       err_170 += 1
        else:
            if f["ok"]: ok_other += 1
            else:       err_other += 1

    total = len(frames)
    ok    = ok_168 + ok_170 + ok_other
    err   = err_168 + err_170 + err_other

    print()
    print(f"Total frames : {total}   OK: {ok}   CRC errors: {err}   "
          f"Pass rate: {100*ok/total:.1f}%")
    print(f"  sync=168µs : {ok_168+err_168:4d}  OK={ok_168}  "
          f"({100*ok_168/(ok_168+err_168) if ok_168+err_168 else 0:.1f}%)")
    print(f"  sync=170µs : {ok_170+err_170:4d}  OK={ok_170}  "
          f"({100*ok_170/(ok_170+err_170) if ok_170+err_170 else 0:.1f}%)")
    if ok_other + err_other:
        print(f"  sync=other : {ok_other+err_other:4d}  OK={ok_other}")
    print(f"  framing resets: {reset_count}")
    if ok_170 + err_170 > 0:
        print()
        print("Note: sync≈170µs is a 500 kHz quantisation artefact (true value ≈168µs).")
        print("      Capture at ≥10 MHz for reliable (>99%) CRC decoding.")


def write_csv(frames, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "sync_us", "tick_us", "status",
                    "data_hex", "crc_recv", "crc_calc", "ok"])
        for fr in frames:
            data_hex = "".join(f"{d:X}" for d in fr["data"])
            w.writerow([fr["frame_idx"], f"{fr['sync_us']:.3f}",
                        f"{fr['tick_us']:.4f}", f"{fr['status']:X}",
                        data_hex, f"{fr['crc_recv']:X}",
                        f"{fr['crc_calc']:X}", "OK" if fr["ok"] else "FAIL"])
    print(f"Frames written to {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Decode SENT frames from a Saleae Logic 2 .sal file."
    )
    p.add_argument("sal_file", nargs="?",
                   default=os.path.join(os.path.dirname(__file__), "Session0.sal"),
                   help="Path to .sal capture (default: Session0.sal next to this script)")
    p.add_argument("--channel",  type=int,   default=2,
                   help="Digital channel index (default 2)")
    p.add_argument("--tick-us",  type=float, default=3.0,
                   help="Nominal tick period µs (default 3.0)")
    p.add_argument("--nibbles",  type=int,   default=6,
                   help="Data nibbles per frame (default 6)")
    p.add_argument("--crc-mode", default="DATA_ONLY",
                   choices=["DATA_ONLY", "STATUS_AND_DATA"])
    p.add_argument("--seed",     type=int,   default=3,
                   choices=[3, 5])
    p.add_argument("--csv",      default=None,
                   help="Export decoded frames to this CSV file")
    args = p.parse_args()

    sal = args.sal_file
    if not os.path.exists(sal):
        print(f"File not found: {sal}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading : {sal}")
    print(f"Channel : {args.channel}")
    print(f"Settings: {args.nibbles} data nibbles  CRC={args.crc_mode}  "
          f"seed=0x{args.seed:02X}  nom_tick={args.tick_us} µs")
    print()

    try:
        falling_us, sample_rate = _read_sal_channel(sal, args.channel)
    except Exception as e:
        print(f"Error reading .sal: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Sample rate  : {sample_rate:,} Hz  ({1e6/sample_rate:.2f} µs/sample)")
    print(f"Falling edges: {len(falling_us)}")
    if falling_us:
        print(f"Capture span : {falling_us[-1]/1e6:.3f} s  "
              f"(first edge at {falling_us[0]:.1f} µs)")
    print()

    frames, reset_count = decode_sent(
        falling_us,
        nom_tick_us = args.tick_us,
        num_nibbles = args.nibbles,
        crc_mode    = args.crc_mode,
        seed        = args.seed,
    )

    print_results(frames, reset_count)

    if args.csv:
        write_csv(frames, args.csv)


if __name__ == "__main__":
    main()
