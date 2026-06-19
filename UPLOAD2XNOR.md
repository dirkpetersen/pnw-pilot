# UPLOAD2XNOR — upload on home WiFi regardless of onroad/offroad

**Branch:** `upload2xnor` (off `integration2xnor`) · **Status:** code done, validated; deploy = restart
the uploader process (no reboot).

## Problem

The connect2xnor uploader is two-pass: pass 1 = small `qlog`/`qcamera`, pass 2 = large HD
`rlog`/`fcamera`/`ecamera`. Pass 2 only ran **once pass 1 was fully drained**
(`if success is None and pass2_allowed(...)`). A car that is **parked but still "awake"** (ignition
on — EVs stay awake a while after parking, or a driver sitting in it) is treated as **onroad**
(onroad/offroad is decided by the ignition signal, not motion — `hardwared.py:230`), so it keeps
recording a stationary route. That route continuously produces fresh pass-1 files, so pass 1 never
empties and **pass 2 (the HD backlog) starves indefinitely** — even while sitting on home WiFi.

This is why the June 18 drive's HD stuck at ~131/270: each time the device was awake on WiFi it was
also "onroad," so pass 1 kept winning.

## Principle (per the user)

**On-road / off-road must NOT gate uploading. Being on home WiFi is the only criterion.** This is
safe *because* of where home WiFi is: you can only be associated to home WiFi while parked at home —
drive away and you drop to cellular, which the uploader already skips (`networkMetered` → skip;
`pass2_allowed` = `networkType == wifi` only, never hotspot/LTE). So "on real WiFi" already means
"parked at home, safe to upload everything," and the ignition state is irrelevant.

## Change

`system/loggerd/uploader.py` main loop: pass 2 is now **interleaved** with pass 1 on real WiFi
instead of waiting for pass 1 to empty:

```python
p1 = uploader.step(network_type_raw, metered)                                   # pass 1 (priority)
p2 = uploader.step(network_type_raw, metered, pass2=True) if pass2_allowed(network_type_raw) else None
success = None if (p1 is None and p2 is None) else (bool(p1) or bool(p2))
```

Each loop now moves **one small file AND one HD file** when on WiFi, so HD drains while parked at
home regardless of ignition/onroad. Pass 1 still goes first each iteration (the small keep-safe
files stay current). WiFi-gated, so it never touches cellular. Backoff/idle logic unchanged (idles
only when *both* passes have nothing left).

## Safety / scope

- WiFi-only (unmetered) — never burns cellular.
- Not safety-critical: the uploader only moves files; it does not touch control or logging.
- Pass 1 retains priority within each iteration (small files first).
- No new params, no schema/build changes — pure Python, loaded at runtime.

## Deploy

`uploader.py` is pure Python. Deploy = md5-guarded file copy + clear pyc + **restart just the
uploader process** (`pkill -f system.loggerd.uploader`; manager respawns it with the new code) — no
reboot, no interruption to driving/logging. Rollback = restore the backup + restart the process.

## Files
- `system/loggerd/uploader.py` — the interleave change (only functional change)
