# visionback — the bounded vision give-back measurement (2026-09-19)

Evidence behind `pnw/pnw-pilot` branch `visionback2pnw` and `docs/pnw/VISIONBACK2PNW.md`. Everything
here is offline analysis; **nothing here runs on the car and no control-path file was touched.**

## The question

> If vision had been allowed to give back at most **N mph** of a map-commanded ICBM slowdown between
> **50 m and 150 m** from the candidate, what lateral acceleration would the truck ACTUALLY have
> pulled on every real curve in the corpus?

## Reproduce

```bash
cd /home/dp/gh/comma/_scratch/visionback
PY=/home/dp/gh/comma/pnw/pnw-pilot/.venv/bin/python
PP=/home/dp/gh/comma/pnw/wt-icbmslow:/home/dp/gh/comma/pnw/wt-icbmslow/opendbc_repo

python3 scan.py                      # 98 files (drives/** + /tmp/arch) -> inventory.json, ticks.jsonl
python3 a1_capability.py             # per-corpus field availability          -> out_a1_capability.txt
python3 a2_sizing.py                 # how many ticks can carry a decision    -> out_a2_sizing.txt
PYTHONPATH=$PP $PY a3_giveback.py    # THE SWEEP                              -> out_a3_giveback.txt
python3 a4_vision_accuracy.py        # vision accuracy vs distance            -> out_a4_vision_accuracy.txt
PYTHONPATH=$PP $PY a5_worst.py       # window sweep + named worst sites       -> out_a5_worst.txt
PYTHONPATH=$PP $PY a6_worstsite_dump.py   # tick-by-tick of the max episodes  -> out_a6_worstsite.txt
```

`a3_giveback.py` imports the SHIPPED functions from `/home/dp/gh/comma/pnw/wt-icbmslow`
(`icbmslow2pnw` @ `d0a6b08abc` == `origin/3devpnw`), so no pipeline constant is transcribed by hand,
and `model_target()` is copied **verbatim** from `_scratch/icbmslow/a5_replay.py` — the forward model
already validated to a median +0.03…+0.10 m/s residual against logged `icbmT`.

## The four things that would have gone wrong silently

1. **`/tmp/arch` is 96 % PARKED.** The six "continuous, unbiased" generations hold 42,918 records of
   which 41,239 are below 5 m/s (ignition on, charging — `CLAUDE.md` Rule 3) and **zero** carry an
   `icbmT`. Folding them into a per-tick rate would have diluted every number with stationary ticks.
   `scan.py` reports per-file record/moving/ICBM counts so the corpus appears as an explicit zero row.
2. **The stock ACC was OFF on 61 % of all ICBM ticks** (`stockOn` = `cruiseState.enabled`), and on
   86 % of the weekend corpus that dominates the sample. With cruise off ICBM's SET− taps reach
   nothing, so its published target is advisory. The first run's worst case (5.80 m/s² at 50 mph) came
   from an episode where the truck was doing 30–34 mph with cruise off and peaked at 1.71 m/s²
   measured. `a3_giveback.py` now reports both populations separately and never merges them.
3. **"At most N mph" has to be clamped at the EPISODE level, not per tick.** The episode's binding
   target can come from a tick outside the 50–150 m window; without `min(..., T + N mph)` the rule
   handed back up to **+27 mph** while every per-tick arithmetic step obeyed its N. The first sweep
   printed that as a 27 mph "give-back" at N = 2 and it looked like a result.
4. **Both terms of the published vision-accuracy table are biased the same way.** The truth is a max
   over a stretch (its own stated caveat), and `visLat` is the lateral accel at the model's *planned*
   speed, so `|visLat|/v_ego²` under-states the curvature — which `icbm_vision_curvature`'s docstring
   warns about in as many words. `a4_vision_accuracy.py` varies both and reports all six combinations
   rather than picking one.

## Derived data

* `inventory.json` — per-file bytes/lines/records/parse-errors/PT span/field-presence/car mix.
* `ticks.jsonl` — 71,136 deduped Ford moving ticks (vEgo > 4 m/s) with the 60 fields the analysis
  needs. Tesla is excluded at the scan: ICBM does not exist there and `slKActl` is 0 % live.
* `episodes.json` — the analysed ICBM curve episodes (cruise-ON population) with their apex witness.
