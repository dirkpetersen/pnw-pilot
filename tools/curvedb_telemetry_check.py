#!/usr/bin/env python3
"""curvedbtel2pnw — the CURVEDB2PNW.md section 3.7 verification gate for the Phase-1 telemetry.

**"Non-zero on a real drive" is necessary and NOT sufficient**, and this file exists because that
lesson was paid for twice inside 24 hours: `visK` has been in ces_events the whole time and is
always zero, and `icbmKVis` was in the key list and never populated. Both misled the curvedb
analysis, and one produced a wrong conclusion that reached the owner. Non-zero also would not have
caught `units2pnw` (km/h logged as mph) or the capnp `str()` enum trap -- both were *non-zero and
wrong*. So, per section 3.7, every new field gets:

  1. its WRITER NAMED -- the check locates the emit site in the live source, so a field that stops
     being written is traceable, not merely detectable (`--writers`; same discipline as the swaglog
     grep trap: identify the writer, never count the string);
  2. a CROSS-CONSISTENCY INVARIANT, not a presence test;
  3. a NULL RATE per drive -- which is also section 3.2's "was the sensor alive on this drive at
     all" test, since P1-B makes aliveness a count of non-nulls;
  4. a NEGATIVE CONTROL -- fields that are Ford-only must read null on a Tesla drive, and the
     Tesla's own achieved curvature (the thing P1-A fixes) must NOT be all-null.

A field that is absent from every record is reported as ABSENT and FAILS. It is not a pass: a
pre-feature corpus and a broken writer look identical from the data alone, and treating the missing
column as "nothing to check" is precisely how a dead field survives. Pass --allow-absent to run the
checker against an OLD corpus (e.g. to exercise the checker itself) without that failing the run.

Usage:
  PYTHONPATH=. python3 tools/curvedb_telemetry_check.py <ces_events.jsonl[.gz]> [more...] \
      [--allow-absent] [--min-speed 5.0] [--quiet]

Exit status: 0 = every check passed, 1 = at least one FAIL.
"""
import argparse
import datetime
import gzip
import json
import math
import statistics
import sys
import zoneinfo

PT = zoneinfo.ZoneInfo("America/Los_Angeles")   # the device runs UTC; the driver lives in Pacific

# The fields this feature added, each with the function that writes it. Verified against the live
# source by --writers, so deleting an emit line is reported here rather than discovered on a drive.
WRITERS = {
  "kPeak":      ("_curve_tele", "CurvePeak.take via _curve_peak_step (100 Hz)"),
  "kPeakN":     ("_curve_tele", "CurvePeak.n -- ticks in the record's window"),
  "kPoseP":     ("_curve_tele", "CurvePeak.k_pose_peak <- _pose_curvature (livePose)"),
  "kPose":      ("_curve_tele", "CESController._pose_k <- _pose_curvature (livePose)"),
  "achLatPose": ("_curve_tele", "_ach_lat(_pose_k, vEgo)"),
  "dq":         ("_curve_tele", "CurvePeak.dq_bits <- _curve_peak_step disqualifier OR"),
  "dqWhy":      ("_curve_tele", "_dq_names(CurvePeak.dq_bits)"),
  "strTq":      ("_curve_tele", "carState.steeringTorque, sampled in _curve_peak_step"),
  "mapLat":     ("_curve_tele", "map_candidate_point(_map_targets, truck, mapDist)"),
  "mapLon":     ("_curve_tele", "map_candidate_point(_map_targets, truck, mapDist)"),
}

# Section 3.2: these must NEVER contain an exact 0.0. A zero here is indistinguishable from a dead
# sensor, which is the defect (D1) the whole of Phase 1 is built around.
NO_ZERO_FIELDS = ("kPeak", "kPoseP", "kPose", "achLatPose", "strTq", "slKActl", "achLat")

# Section 3.7 item 4. car_gps is Ford-only by construction (no Ford carstate -> nothing publishes
# CarGps); icbmT/icbmSrc only exist on the Lightning's stock-ACC shadow path.
FORD_ONLY_FIELDS = ("car_gps", "icbmT", "icbmSrc")

R_EARTH = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * R_EARTH * math.asin(min(1.0, a ** 0.5))


def load(path):
  """Every parsable JSON object in a (possibly gzipped, possibly torn) ces_events file."""
  op = gzip.open if str(path).endswith(".gz") else open
  out, bad = [], 0
  with op(path, "rt", errors="replace") as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        r = json.loads(line)
      except Exception:
        bad += 1        # a torn last line from a brown-out reboot is expected; a flood of them is not
        continue
      if isinstance(r, dict):
        out.append(r)
  return out, bad


class Report:
  def __init__(self, quiet=False):
    self.rows, self.failed, self.quiet = [], False, quiet

  def line(self, text=""):
    if not self.quiet:
      print(text)

  def check(self, status, name, detail):
    """status: PASS / FAIL / ABSENT / INFO / SKIP.

    ABSENT counts as a failure, deliberately: a field missing from every record cannot be
    distinguished from a dead writer by looking at the data, so "there was nothing to check" must
    not exit 0. --allow-absent downgrades it to SKIP at the call site, which is the only way to run
    this against a pre-feature corpus."""
    if status in ("FAIL", "ABSENT"):
      self.failed = True
    self.line(f"  [{status:6}] {name:38} {detail}")


def present(records, field):
  """How many records carry the key at all, and how many carry a non-null value."""
  have = sum(1 for r in records if field in r)
  nonnull = sum(1 for r in records if r.get(field) is not None)
  return have, nonnull


def check_writers(rep):
  """Section 3.7 item 1: name the writer, and prove the emit line still exists in the live source."""
  rep.line("WRITER PATHS (section 3.7 item 1) -- the emit site must exist in the shipped source")
  try:
    import inspect

    from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  except Exception as e:
    rep.check("FAIL", "import ces_pnw", f"{type(e).__name__}: {e} -- writers cannot be verified")
    return
  for field, (func, how) in sorted(WRITERS.items()):
    fn = getattr(m, func, None)
    if fn is None:
      rep.check("FAIL", field, f"no {func}() in ces_pnw -- the writer is GONE")
      continue
    try:
      src = inspect.getsource(fn)
    except OSError as e:
      rep.check("FAIL", field, f"cannot read {func}() source ({type(e).__name__})")
      continue
    ok = f'"{field}"' in src
    rep.check("PASS" if ok else "FAIL", field,
              f"ces_pnw.{func}() <- {how}" if ok else f"NOT EMITTED by ces_pnw.{func}() -- dead field")
  # The consumers of the fragment, so a builder that exists but is never spliced in is caught too.
  for meth in ("_event_record", "_steer_log_step"):
    try:
      src = inspect.getsource(getattr(m.CESController, meth))
      ok = "_curve_tele(" in src
    except Exception:
      ok = False
    rep.check("PASS" if ok else "FAIL", f"{meth} splices _curve_tele",
              "wired" if ok else "the fragment is built but never reaches the record")
  ok = "archive_rotated_generation(" in inspect.getsource(m.rotate_event_log)
  rep.check("PASS" if ok else "FAIL", "rotate_event_log archives (3.8)",
            f"-> {m.CES_ARCHIVE_DIR}" if ok else "rotation DESTROYS the oldest generation; the " +
                                                 "Phase-1 corpus cannot accumulate")


# Two fields are null BY DESIGN on most records, so "always null" only means "dead" within the
# population where a value is actually expected. Stating that population explicitly is the point:
# folding them in unconditionally would either raise a false alarm on a clean drive (dqWhy) or hide
# a real one (mapLat on the map ticks, drowned by the far larger no-candidate population).
CONDITIONAL = {
  "dqWhy":  (lambda r: r.get("dq") is True, "records whose second was disqualified"),
  "mapLat": (lambda r: bool(r.get("mapDist")), "records with a map candidate"),
  "mapLon": (lambda r: bool(r.get("mapDist")), "records with a map candidate"),
}


def check_presence(rep, recs, allow_absent):
  """Section 3.7 item 3: the per-drive null rate, and ABSENT as a FAIL rather than a shrug."""
  rep.line("FIELD PRESENCE + NULL RATE (section 3.7 item 3)")
  n = len(recs)
  absent = []
  for field in WRITERS:
    have, nonnull = present(recs, field)
    if have == 0:
      absent.append(field)
      rep.check("SKIP" if allow_absent else "ABSENT", field,
                "not in ANY record -- pre-feature corpus, or the writer is dead")
      continue
    null_pct = 100.0 * (have - nonnull) / have
    detail = f"{have}/{n} records carry it, {nonnull} non-null ({null_pct:.1f}% null)"
    if field in CONDITIONAL:
      pred, what = CONDITIONAL[field]
      pop = [r for r in recs if field in r and pred(r)]
      live = sum(1 for r in pop if r.get(field) is not None)
      if not pop:
        rep.check("INFO", field, detail + f"  (no {what} on this drive -- nothing to expect)")
        continue
      status = "PASS" if live else "FAIL"
      rep.check(status, field, detail + f"; {live}/{len(pop)} non-null among {what}"
                + ("" if live else "  <-- ALWAYS NULL where a value was due: the visK failure"))
      continue
    status = "PASS" if nonnull else "FAIL"
    rep.check(status, field, detail + ("" if nonnull else "  <-- ALWAYS NULL: this is the visK failure"))
  return absent


def check_no_zeros(rep, recs):
  """Section 3.2: an exact 0.0 in this family is a dead sensor wearing a measurement's costume."""
  rep.line("EXACT ZERO -> NULL (section 3.2 / P1-B)")
  for field in NO_ZERO_FIELDS:
    have, _ = present(recs, field)
    if have == 0:
      rep.check("SKIP", field, "not present in this corpus")
      continue
    zeros = sum(1 for r in recs if r.get(field) == 0.0)
    rep.check("PASS" if zeros == 0 else "FAIL", field,
              "no exact zeros" if zeros == 0 else f"{zeros} records log exactly 0.0 -- " +
                                                  "indistinguishable from a dead sensor")


def _ratios(recs, min_speed):
  """kPeak * vEgo^2 vs |achLat| on records where both are real (section 3.7 item 2)."""
  out = []
  for r in recs:
    kp, al, v = r.get("kPeak"), r.get("achLat"), r.get("vEgo")
    if kp is None or al is None or v is None or v < min_speed:
      continue
    if abs(al) < 1.0:            # below ~0.1 g the instantaneous sample is mostly road noise
      continue
    out.append((kp * v * v) / abs(al))
  return out


def check_invariants(rep, recs, min_speed):
  rep.line("CROSS-CONSISTENCY INVARIANTS (section 3.7 item 2)")

  # I1 -- the peak may exceed the instantaneous sample (it is a max, and it carries the commanded
  # half), but it must never UNDER-read it: that would mean the accumulator is not accumulating.
  ratios = _ratios(recs, min_speed)
  if len(ratios) < 20:
    rep.check("SKIP", "I1 kPeak*vEgo^2 vs achLat", f"only {len(ratios)} eligible records (need 20)")
  else:
    bad = sum(1 for x in ratios if x < 0.8)
    pct = 100.0 * bad / len(ratios)
    rep.check("PASS" if pct <= 5.0 else "FAIL", "I1 kPeak*vEgo^2 vs achLat",
              f"n={len(ratios)} p50={statistics.median(ratios):.2f} " +
              f"min={min(ratios):.2f} max={max(ratios):.2f}; {bad} ({pct:.1f}%) under 0.8x")

  # I2 -- P1-A's own section 3.1 cross-check: the localizer-derived achieved curvature against the
  # CAN-derived one, on the car where BOTH are alive (the Lightning). This is what quantifies the
  # device-frame-vs-calibrated-frame approximation _pose_curvature documents.
  pairs = [(r["kPose"], r["slKActl"]) for r in recs
           if r.get("kPose") is not None and r.get("slKActl") is not None
           and abs(r["slKActl"]) > 1e-4 and (r.get("vEgo") or 0.0) >= min_speed]
  if len(pairs) < 20:
    rep.check("SKIP", "I2 kPose vs slKActl (Ford)", f"only {len(pairs)} records with both alive")
  else:
    # SIGNED, not |ratio|: both sources are documented positive=left (controlsd's steer_limit_status
    # comment for CS.yawRate; the ISO device frame for angularVelocityDevice.z), so a median near
    # -1.0 means the pose source needs negating -- non-zero and WRONG, the failure mode "is it
    # non-zero" cannot see. Measuring this is section 3.1's stated cross-check.
    rel = statistics.median([a / b for a, b in pairs])
    rep.check("PASS" if abs(rel - 1.0) <= 0.25 else "FAIL", "I2 kPose vs slKActl (Ford)",
              f"n={len(pairs)} median ratio {rel:+.3f} (expect ~+1.0; sign flip or |ratio-1|>0.25 " +
              "means the device frame needs calibrating)")

  # I3 -- section 3.4: mapLat/mapLon must be mapDist away from the truck, or they are not the
  # candidate's coordinates at all.
  geo = [r for r in recs if r.get("mapLat") is not None and r.get("lat") is not None
         and r.get("mapDist")]
  if not geo:
    rep.check("SKIP", "I3 mapLat/mapLon vs mapDist", "no record carries both a candidate and a fix")
  else:
    errs = [abs(haversine_m(r["lat"], r["lon"], r["mapLat"], r["mapLon"]) - r["mapDist"]) for r in geo]
    bad = sum(1 for e in errs if e > 30.0)
    rep.check("PASS" if bad == 0 else "FAIL", "I3 mapLat/mapLon vs mapDist",
              f"n={len(geo)} p50={statistics.median(errs):.1f} m max={max(errs):.1f} m; " +
              f"{bad} outside mapDist +-30 m")
    at_truck = sum(1 for r in geo if haversine_m(r["lat"], r["lon"], r["mapLat"], r["mapLon"]) < 1.0)
    rep.check("PASS" if at_truck == 0 else "FAIL", "I3b candidate is not the truck",
              "distinct" if at_truck == 0 else f"{at_truck} records put the candidate ON the truck")

  # I4 -- section 3.5: the roll-up must cover every sampled flag. dq is a superset (it ORs over the
  # whole window at 100 Hz), so a sampled flag true with dq false means the OR is not running.
  dqr = [r for r in recs if r.get("dq") is not None]
  if not dqr:
    rep.check("SKIP", "I4 dq covers the sampled flags", "dq not present")
  else:
    def flagged(r):
      return bool(r.get("strPrs") or r.get("blnk") or r.get("slAngSat") or r.get("slSat")
                  or r.get("slCurvLim") or r.get("lcGate") == "lanechange")
    missed = [r for r in dqr if flagged(r) and not r["dq"]]
    n_flag = sum(1 for r in dqr if flagged(r))
    rep.check("PASS" if not missed else "FAIL", "I4 dq covers the sampled flags",
              f"{n_flag}/{len(dqr)} records carry a sampled disqualifier; {len(missed)} of those " +
              "have dq false")
    rep.check("INFO", "I4b dq rate",
              f"{sum(1 for r in dqr if r['dq'])}/{len(dqr)} seconds disqualified")

  # I5 -- the peak must never be below the instantaneous sample it is the peak OF.
  both = [r for r in recs if r.get("kPoseP") is not None and r.get("kPose") is not None]
  if not both:
    rep.check("SKIP", "I5 kPoseP >= |kPose|", "not present")
  else:
    bad = [r for r in both if r["kPoseP"] < abs(r["kPose"]) - 1e-9]
    rep.check("PASS" if not bad else "FAIL", "I5 kPoseP >= |kPose|",
              f"n={len(both)}, {len(bad)} records where the peak is below its own sample")


def check_negative_control(rep, recs, cars):
  """Section 3.7 item 4. On a Tesla drive the Ford-only columns must be null -- and, as the positive
  half of the same control, the Tesla's livePose-derived curvature must NOT be."""
  rep.line("NEGATIVE CONTROL (section 3.7 item 4)")
  tesla = [r for r in recs if str(r.get("car") or "").startswith("TESLA")]
  ford = [r for r in recs if str(r.get("car") or "").startswith("FORD")]
  if not tesla:
    rep.check("SKIP", "Ford-only fields on a Tesla", f"no Tesla records here (cars: {sorted(cars)})")
  else:
    for field in FORD_ONLY_FIELDS:
      leaked = sum(1 for r in tesla if r.get(field) is not None)
      rep.check("PASS" if leaked == 0 else "FAIL", f"{field} null on Tesla",
                f"{len(tesla)} Tesla records, {leaked} non-null")
    # The whole point of P1-A: this is the car where the CAN-derived achieved curvature is dead.
    # Counted as "null OR exactly 0.0" so the number means the same thing on a pre-P1-B corpus (where
    # a dead reading logs as 0.0) and a post-P1-B one (where it logs as null).
    dead = sum(1 for r in tesla if r.get("slKActl") is None or r.get("slKActl") == 0.0)
    have_pose, alive_pose = present(tesla, "kPose")
    rep.check("INFO", "Tesla slKActl (CAN) dead rate",
              f"{dead}/{len(tesla)} records null-or-zero -- defect D1, the reason P1-A exists")
    if have_pose == 0:
      rep.check("ABSENT", "Tesla kPose alive (P1-A)", "kPose not present in this corpus")
    else:
      rep.check("PASS" if alive_pose else "FAIL", "Tesla kPose alive (P1-A)",
                f"{alive_pose}/{have_pose} non-null"
                + ("" if alive_pose else "  <-- P1-A did NOT fix D1; Phase 1 is void on this car"))
  if ford:
    have, alive = present(ford, "car_gps")
    rep.check("INFO", "car_gps on Ford (positive control)",
              f"{alive}/{have} non-null" if have else "not present")


def span_pt(recs):
  ts = [r["t"] for r in recs if isinstance(r.get("t"), int | float) and r["t"] > 1577836800.0]
  if not ts:
    return "no valid wall clock (dead-RTC boot?)"
  f = "%Y-%m-%d %H:%M:%S %Z"
  return (f"{datetime.datetime.fromtimestamp(min(ts), PT):{f}} -> " +
          f"{datetime.datetime.fromtimestamp(max(ts), PT):{f}}  ({(max(ts) - min(ts)) / 3600:.2f} h)")


def check_file(path, rep, min_speed, allow_absent):
  recs, bad = load(path)
  ticks = [r for r in recs if r.get("ev") in ("tick", "adopt", "steer")]
  moving = [r for r in ticks if (r.get("vEgo") or 0.0) >= min_speed]
  cars = {r.get("car") for r in recs if r.get("car")}
  rep.line("")
  rep.line("=" * 100)
  rep.line(f"{path}")
  rep.line(f"  {len(recs)} records ({bad} unparsable), {len(ticks)} tick/adopt/steer, " +
           f"{len(moving)} above {min_speed} m/s")
  rep.line(f"  cars: {sorted(c for c in cars if c)}")
  rep.line(f"  span (PT): {span_pt(recs)}")
  rep.line("=" * 100)
  if not moving:
    rep.check("FAIL", "corpus", "no moving tick/adopt/steer records -- nothing can be checked here")
    return
  check_presence(rep, moving, allow_absent)
  check_no_zeros(rep, moving)
  check_invariants(rep, moving, min_speed)
  check_negative_control(rep, moving, cars)


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("paths", nargs="+", help="ces_events.jsonl (or .gz) files")
  ap.add_argument("--min-speed", type=float, default=5.0,
                  help="m/s below which a record is parked/crawling and proves nothing (default 5)")
  ap.add_argument("--allow-absent", action="store_true",
                  help="a field missing from every record is SKIP, not a failure (old corpora only)")
  ap.add_argument("--quiet", action="store_true")
  args = ap.parse_args(argv)

  rep = Report(quiet=args.quiet)
  rep.line("curvedbtel2pnw -- CURVEDB2PNW.md section 3.7 verification gate")
  rep.line("")
  check_writers(rep)
  for p in args.paths:
    check_file(p, rep, args.min_speed, args.allow_absent)
  rep.line("")
  rep.line("RESULT: " + ("FAIL -- see the [FAIL]/[ABSENT] rows above" if rep.failed else "PASS"))
  return 1 if rep.failed else 0


if __name__ == "__main__":
  sys.exit(main())
