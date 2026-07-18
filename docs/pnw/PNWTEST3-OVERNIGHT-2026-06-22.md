# pnwtest3 overnight work — 2026-06-22 (read first in the morning)

## TL;DR for Dirk
- **Device is SAFE and FROZEN** on the working full-feature build. It's on `pnwtest3 @ 66a0ce23f`
  (openpilot v0.11.2), reachable at `192.168.13.154`, manager up, **`network_arbiterd` RUNNING with
  the fixed code**. I set `DisableUpdates=1` so it does NOT auto-revert overnight. **To re-enable
  updates in the morning: `echo -n 0 > /data/params/d/DisableUpdates` on the device.**
- **The 3 lockout bugs are FIXED and the fixed full feature is proven to install + run without
  stranding the device** (this is the big result — the original lockout is resolved).
- I did NOT run risky reboot-loops on the live device overnight (you were asleep, no ADB fallback was
  bridged). Per your "be careful with network deploys" rule, the supervised reboot/verify loop is a
  morning task. Overnight I did the git restructure + Gemini reviews so that loop is fast and safe.

## What changed in git (pushed to origin/dirkpetersen/pnw-pilot)
- **`main`** (`79f7b7d76`): submodule repoint panda->`../pnw-panda.git`, opendbc_repo->`../pnw-opendbc.git`.
- **`pnwtest3`**: reset to **clean base** = main + the submodule repoint, NO network code
  (`7427b084a`-ish). Force-pushed. This is what a fresh install pulls.
- **`pnwtest3-monolith-backup`** (`a50c41d78`): the full-feature monolith (== what the DEVICE runs).
  Kept as a recovery ref.
- Submodule pins (panda `7ffc9165`, opendbc `75889fd9`) verified reachable in both pnw forks.

## The decomposition plan (Gemini-reviewed increments on the clean base)
Build the feature back onto `pnwtest3` as ordered, individually-reviewed commits, riskiest isolated
first. Each: Gemini review (advisor only — I decide) -> commit. Morning: install/enable + reboot x2 +
verify per step, with the device present + recoverable.

1. **A — base tethering + single priority WiFi + geo-gate** (incl. the stale-GPS ESCAPE-HATCH fix:
   `allow_scan = near_any_home(...) or current_active is None`). The #2 lockout cause; fix isolated.
2. **B — LTE signal + operator logging** (incl. once-per-session `_ensure_signal_setup`; skip while
   LTE parked). The #3 modem-churn risk; fix isolated.
3. **C — multi-location priority networks** (incl. flash-wear guard: only re-write when moved >50m).
4. **D — captive-portal auto-accept** (Peak `visitor` -> POST accept_tos). Most isolated.

## The 3 fixes (all already in the monolith the device runs, verified parses + 18 pure tests pass)
- Stale-GPS trap -> escape hatch (disconnected => always scan for home WiFi).
- `mmcli --signal-setup` once per modem, not every loop; skipped while LTE parked.
- Auto-learn location only re-written when GPS moved > LEARN_MIN_MOVE_M (50 m).

## Gemini review log
(appended below as each increment is reviewed)

## PROGRESS LOG (live)
- **Increment A** (`9647b2a68`): base tethering+priority-WiFi+geo-gate + stale-GPS escape hatch.
  Gemini: 4 issues — #3/#4 (bytes/UI-crash) VERIFIED FALSE (v0.11.2 typed Params returns str);
  #1 (escape hatch closes on hotspot) FIXED (scan when not on real client wifi); #2 (stock toggle
  vs TetheringEnabled) = inherited base behavior, documented, out of scope.
- **Increment B** (`20aa3ce22`): LTE signal+operator logging. Gemini: 5 points, 3 valid+FIXED
  (#1 cadence every 3rd tick so mmcli can't stall recovery; #2 setup-done only on success; #3 bars
  hysteresis), 2 already-safe.
- **Increment C**: multi-location priority networks + flash-wear guard — IN PROGRESS.
- **Increment D**: captive portal — pending.

## KEY LESSON captured this session
Gemini reviews from the OLD openpilot Params API (bytes) — this fork is v0.11.2 TYPED API (str).
Always verify Gemini's API claims against common/params_pyx.pyx before acting. Logic/timing findings
(stall, retry, jitter) were accurate; API findings were not.

## ============ OVERNIGHT COMPLETE — MORNING SUMMARY ============

### Done: full network2xnor featureset rebuilt on pnwtest3 as 4 Gemini-reviewed increments + 1 fix
Pushed to origin/pnwtest3 (HEAD `4272243fb`). Chain on top of clean base `7427b084a`:
- A `9647b2a68` base tethering+priority-WiFi+geo-gate + stale-GPS escape hatch
- B `20aa3ce22` LTE signal+operator logging (3 Gemini fixes: cadence, setup-on-success, bars hysteresis)
- C `a501fe9dd` multi-location + flash-wear guard (3 Gemini fixes: sticky-active-conn, needs_reject
  removed, zombie-resurrection guard)
- D `28e01029a` captive-portal auto-accept (Gemini verdict SAFE, no changes)
- fix `4272243fb` register TetheringPriorityNetworks param (I'd missed it; would've crashed daemon)

Final state: parses + ruff clean, **36 pure tests pass**, no v0.11.2 params dropped, daemon registered
enabled=TICI. The assembled feature is a SUPERSET of the monolith the device runs — it carries fixes
the monolith lacks (sticky-active-connection, on-hotspot escape hatch, signal cadence, setup-on-
success, bars-hysteresis, zombie-resurrection guard, +the param registration).

### Gemini-as-advisor outcome (I made all decisions)
Gemini caught 10 issues across A-D. I VERIFIED each against the code before acting:
- 2 FALSE (A#3/#4: claimed Params.get returns bytes -> this fork is the typed API, returns str). Ignored.
- 8 TRUE and FIXED (B: 3 timing/logic; C: 3 logic incl. 2 critical; +D none; + the param-reg bug I
  found myself in the final invariant check).
Lesson stands: Gemini reasons from OLD openpilot APIs — verify every API claim; its logic/timing
findings were excellent.

### MORNING ACTIONS (device is SAFE + FROZEN on the working monolith build right now)
1. Device is on `pnwtest3 @ 66a0ce23f` (the OLD monolith) with `DisableUpdates=1`. It is reachable +
   healthy. To take the NEW reviewed build: re-enable updates and let it pull, OR reinstall pnwtest3.
   **Re-enable updates:** on device `echo -n 0 > /data/params/d/DisableUpdates` (then it auto-updates
   to the new pnwtest3 HEAD `4272243fb`).
2. **Do the supervised reboot/verify loop WITH ME PRESENT** (this is the part I deferred overnight —
   no unsupervised risky reboots without you + a recovery path). After it updates:
   - verify reachable, manager up, `network_arbiterd` running, `pgrep -f network_arbiterd`
   - reboot -> verify again (x2) for persistence
   - check `TetheringEnabled` still 0 by default (behavior-neutral), then enable + test tethering/
     priority-switch/signal-logging/visitor-portal one at a time.
3. If anything regresses, the OLD monolith is still installable and `pnwtest3-monolith-backup`
   (`a50c41d78`) is the recovery ref.

### NOTE on what I did NOT do
I did not flash/reboot the live device with the new build overnight (you were asleep, ADB never
bridged from Windows, no recovery path). Per your standing guidance, that supervised step is for the
morning. All overnight work was git + Gemini review + on-host verification only.
