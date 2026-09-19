#!/usr/bin/env python3
"""A8: would the bounded give-back have refuted the 2026-09-08 ~20:28:51 PT phantom -- the event the
whole line of work exists for (docs/CURVEDB2PNW.md Sec 1)? Dumps every in-window map ICBM tick of
that drive with what vision said."""
import json, sys, datetime, zoneinfo
sys.path.insert(0, "/home/dp/gh/comma/_scratch/visionback")
import a3_giveback as A3                                                         # noqa: E402

PT = zoneinfo.ZoneInfo("America/Los_Angeles")
MPH, fnum = A3.MPH, A3.fnum
V = A3.veh()
rows = sorted((json.loads(x) for x in open(A3.TICKS)), key=lambda r: r["t"])
sel = [r for r in rows if r["_src"].startswith("2026-09-08")]
print(f"2026-09-08 ford moving ticks: {len(sel)}  span "
      f"{datetime.datetime.fromtimestamp(sel[0]['t'], PT):%H:%M:%S}.."
      f"{datetime.datetime.fromtimestamp(sel[-1]['t'], PT):%H:%M:%S} PT")
icb = [r for r in sel if r.get("icbmT") is not None]
print(f"ICBM ticks on 09-08: {len(icb)}")
for r in icb:
  d, ve = fnum(r.get("mapDist")), fnum(r.get("vEgo"))
  ref = fnum(r.get("icbmC")) or fnum(r.get("stockSet"))
  mv = fnum(r.get("mapV"))
  if r.get("icbmSrc") != "map" or not d or not (50 <= d <= 150) or not (mv and ve and ref):
    continue
  p = A3.model_target(V, mv, d, ref, ve, None, False, floor_raw=True)
  vv, why = A3.vision_speed(r, "vislat_bold", V.icbm_lead_lat_accel)
  vs = "inf" if vv == float("inf") else (f"{vv / MPH:.0f}" if vv else "refuse")
  print(f"  {datetime.datetime.fromtimestamp(r['t'], PT):%H:%M:%S} stockOn={r.get('stockOn')} "
        f"v={ve / MPH:.0f} set={ref / MPH:.0f} mapV={mv / MPH:.0f} d={d:.0f} "
        f"modelT={p / MPH if p else float('nan'):.1f} visLat={fnum(r.get('visLat')):.2f} "
        f"-> v_vis={vs} why={why}")
