#!/usr/bin/env python3
"""
ces2core2pnw — CES v1 vs CES2 replay harness (the acceptance gate, CES2-STUDY.md §6 phase 0).

Feeds a recorded ces_events.jsonl tick stream through BOTH decision cores (the live v1
ConditionalExperimentalSwitching and the CES2 Ces2Core) and reports the divergence timeline +
summary. This is how the DEC-anchor stop-urgency table gets validated / re-anchored before the
Ces2Core flag may ever go live (study §4 retirement condition).

Usage:
  PYTHONPATH=. python3 tools/ces2_replay.py <ces_events.jsonl> [--start HH:MM:SS] [--end HH:MM:SS]
                                            [--max-rows N] [--quiet]
  (times are UTC, matched against the record's wall-clock `t`)

SIGNAL RECONSTRUCTION (per-tick record -> the decide_active signals dict) and its documented
approximations — read before trusting a divergence:
  vEgo/vSet/spdLim/aEgo/gas   exact (logged).
  lead/dRel/vLead             exact (logged; `lead` bool is authoritative, dRel 0.0 is ambiguous).
  mapV/mapDist                exact (raw pfeiferj target + distance; 0/0 = none -> (0, inf)).
  curve_lat_accel_vision      APPROXIMATE: only curvePct+curveSrc are logged. When curveSrc ==
                              "vision", |lat| ~= curvePct/100 * CURVE_LAT_ACCEL_ENTER (saturates at
                              100% == exactly-at-threshold, so a tripping vision curve is replayed
                              at threshold+0.02); when curveSrc == "map" or "", the vision half is
                              UNKNOWN (curve_closeness logs only the max half) -> replayed as 0.
  time_to_curve               APPROXIMATE: 2.0 s when a vision curve is present (inside the 3.5 s
                              window — it was counted), else 10.0 (outside).
  model_should_stop           APPROXIMATE: reconstructed as reason in ("stop", "stopIntent") — the
                              tick stream does not log shouldStop directly. A shouldStop tick that
                              lost the ladder to a higher condition (or was gated by has_lead) is
                              invisible; stop timing is ~1 s quantized (tick cadence).
  mdl_end_x                   exact WHERE PRESENT (mdlEndX ships from 2e4bcec457). Older logs have
                              none -> 0.0 -> CES2's endpoint urgency is NEUTRALIZED (only the
                              reconstructed shouldStop feeds StopEvidence). Stated per-file below.
  blinker / lane_change_intent  NOT LOGGED -> False (the TURN condition cannot fire in replay).
  standstill                  vEgo < 0.3 (carState.standstill is not logged).
  toggles                     assumed ALL ON (+ turns OFF), CESMode Standard. A Light-mode drive
                              replays with curves enabled — check the DRIVE_REPORT for the config.
  cadence                     records are ~1 Hz (vs 100 Hz live): both cores step with the REAL dt
                              between records, so filters/dwells see true seconds, but entries can
                              appear up to 1 tick (~1 s) later than live. Both cores share the
                              distortion, so relative divergence remains meaningful.
  gaps                        dt > 5 s (log gap / drive break) resets both cores + trackers.
"""
import argparse
import datetime
import json
import sys

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (ConditionalExperimentalSwitching,
                                                              PullAwayTracker)
from openpilot.selfdrive.controls.lib.ces_pnw.ces2_core import Ces2Core, DivergenceCounter

ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True, "turns": False}
GAP_RESET_S = 5.0


def sig_from_record(r: dict) -> dict:
  v = float(r.get("vEgo") or 0.0)
  lead = bool(r.get("lead")) if r.get("lead") is not None else float(r.get("dRel") or 0.0) > 0.0
  vis, ttc = 0.0, 10.0
  if r.get("curveSrc") == "vision":
    pct = float(r.get("curvePct") or 0.0)
    vis = pct / 100.0 * C.CURVE_LAT_ACCEL_ENTER
    if pct >= 100.0:
      vis = C.CURVE_LAT_ACCEL_ENTER + 0.02      # saturated: was at/over threshold live
    ttc = 2.0
  map_v = float(r.get("mapV") or 0.0)
  map_dist = float(r.get("mapDist") or 0.0)
  reason = r.get("reason") or ""
  return {
    "v_ego": v,
    "has_lead": lead,
    "lead_vlead": float(r.get("vLead") or 0.0),
    "lead_drel": float(r.get("dRel") or 0.0),
    "blinker": False,
    "map_target_v": map_v,
    "map_target_dist": map_dist if map_dist > 0.0 else float("inf"),
    "curve_lat_accel_vision": vis,
    "time_to_curve": ttc,
    "model_should_stop": reason in ("stop", "stopIntent"),
    "mdl_end_x": float(r.get("mdlEndX") or 0.0),
    "v_set": float(r.get("vSet") or 0.0),
    "spd_lim": float(r.get("spdLim") or 0.0),
    "standstill": v < 0.3,
    "lane_change_intent": False,
    "toggles": ALL_ON,
  }


def hms(t: float) -> str:
  return datetime.datetime.fromtimestamp(t, datetime.UTC).strftime("%H:%M:%S")


def parse_hms(day_t: float, s: str) -> float:
  """HH:MM:SS (UTC) on the same day as day_t -> epoch."""
  d = datetime.datetime.fromtimestamp(day_t, datetime.UTC)
  hh, mm, ss = (int(x) for x in s.split(":"))
  return d.replace(hour=hh, minute=mm, second=ss, microsecond=0).timestamp()


def replay(path: str, t_start=None, t_end=None, max_rows=200, quiet=False):
  v1 = ConditionalExperimentalSwitching()
  v2 = Ces2Core()
  trk1, trk2 = PullAwayTracker(), PullAwayTracker()
  div = DivergenceCounter()

  n = n_div = 0
  n_mdl = 0
  last_t = None
  match_logged = total_logged = 0
  rows = []
  transitions = {}          # (v1_reason -> v2_reason) while diverged
  div_episodes = []
  cur_ep = None

  with open(path) as f:
    for line in f:
      try:
        r = json.loads(line)
      except json.JSONDecodeError:
        continue
      if r.get("ev") not in ("tick", "adopt"):
        continue
      t = float(r.get("t") or 0.0)
      if (t_start and t < t_start) or (t_end and t > t_end):
        continue
      dt = (t - last_t) if last_t is not None else 1.0
      last_t = t
      if dt <= 0.0:
        continue
      if dt > GAP_RESET_S:
        v1.reset()
        v2.reset()
        trk1, trk2 = PullAwayTracker(), PullAwayTracker()
        dt = 1.0

      s = sig_from_record(r)
      if s["mdl_end_x"] > 0.0:
        n_mdl += 1
      s1 = dict(s)
      s2 = dict(s)
      s1["lead_opening"] = trk1.update(t, s["has_lead"], s["lead_drel"], s["model_should_stop"])
      s2["lead_opening"] = trk2.update(t, s["has_lead"], s["lead_drel"], s["model_should_stop"])
      m1 = v1.update_decision(s1, dt)
      m2 = v2.update_decision(s2, dt)
      n += 1

      # reconstruction sanity: replayed v1 vs the LOGGED live mode
      logged = r.get("mode")
      if logged in ("chill", "experimental"):
        total_logged += 1
        if logged == m1:
          match_logged += 1

      div.update(m1, m2)
      if m1 != m2:
        n_div += 1
        key = f"{m1}/{v1.status()} -> {m2}/{v2.status()}"
        transitions[key] = transitions.get(key, 0) + 1
        if cur_ep is None:
          cur_ep = {"t0": t, "key": key, "v": s["v_ego"]}
        if len(rows) < max_rows:
          rows.append((t, m1, v1.status(), m2, v2.status(), s, round(v2.urgency, 2), logged))
      else:
        if cur_ep is not None:
          cur_ep["t1"] = t
          div_episodes.append(cur_ep)
          cur_ep = None
  if cur_ep is not None:
    cur_ep["t1"] = last_t
    div_episodes.append(cur_ep)

  print(f"\n=== {path}")
  if t_start or t_end:
    print(f"    window {hms(t_start) if t_start else '...'} - {hms(t_end) if t_end else '...'} UTC")
  mdl_note = "urgency LIVE" if n_mdl else "urgency NEUTRALIZED — pre-mdlEndX log"
  print(f"    ticks replayed: {n}   with mdlEndX: {n_mdl} ({mdl_note})")
  if total_logged:
    pct = 100.0 * match_logged / total_logged
    print(f"    reconstruction sanity: replayed-v1 matches the LOGGED live mode on {match_logged}/{total_logged} ticks ({pct:.1f}%)")
  print(f"    divergent ticks: {n_div} ({100.0 * n_div / max(n, 1):.1f}%)   episodes: {len(div_episodes)}")
  for key, cnt in sorted(transitions.items(), key=lambda kv: -kv[1]):
    print(f"      {cnt:5d}  {key}")
  if not quiet and rows:
    print("    --- divergence timeline (v1 | ces2) ---")
    for t, m1, r1, m2, r2, s, urg, logged in rows:
      left = f"{hms(t)}  v={s['v_ego']:5.1f} set={s['v_set']:5.1f} lead={int(s['has_lead'])} dRel={s['lead_drel']:5.1f} vLead={s['lead_vlead']:5.1f}"
      right = f"stop={int(s['model_should_stop'])} urg={urg:4.2f} | v1={m1[:4]}/{r1:<10} ces2={m2[:4]}/{r2:<10} logged={logged}"
      print("      " + left + " " + right)
  return {"ticks": n, "div_ticks": n_div, "episodes": len(div_episodes), "transitions": transitions}


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("log", help="ces_events.jsonl (tick stream)")
  ap.add_argument("--start", help="HH:MM:SS UTC window start")
  ap.add_argument("--end", help="HH:MM:SS UTC window end")
  ap.add_argument("--max-rows", type=int, default=200)
  ap.add_argument("--quiet", action="store_true", help="summary only")
  args = ap.parse_args()

  t0 = None
  with open(args.log) as f:
    for line in f:
      try:
        t0 = float(json.loads(line).get("t"))
        break
      except Exception:
        continue
  if t0 is None:
    print("no records", file=sys.stderr)
    return 1
  t_start = parse_hms(t0, args.start) if args.start else None
  t_end = parse_hms(t0, args.end) if args.end else None
  replay(args.log, t_start, t_end, args.max_rows, args.quiet)
  return 0


if __name__ == "__main__":
  sys.exit(main())
