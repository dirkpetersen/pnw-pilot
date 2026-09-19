#!/usr/bin/env python3
"""A6: tick-by-tick dump of the episodes that produce the sweep's maximum, plus posted-limit
availability. The maximum decides the verdict, so it must be shown, not asserted."""
import datetime
import json
import sys
import zoneinfo
from collections import defaultdict

sys.path.insert(0, "/home/dp/gh/comma/_scratch/visionback")
import a3_giveback as A3                                                         # noqa: E402

PT = zoneinfo.ZoneInfo("America/Los_Angeles")
MPH, fnum, CP = A3.MPH, A3.fnum, A3.CP
SITES = [1789674001.0, 1789588735.0, 1789595300.0]   # filled from argv/epoch below


def main():
  V = A3.veh()
  a_lat_vis = V.icbm_lead_lat_accel
  recs = sorted((json.loads(x) for x in open(A3.TICKS)), key=lambda r: r["t"])
  eps = A3.build_episodes(recs)

  # ---- posted-limit availability on the ticks a give-back would act on ----------------------
  post = defaultdict(int)
  for e in eps:
    for r in e["ticks"]:
      if r.get("icbmSrc") != "map":
        continue
      d = fnum(r.get("mapDist"))
      if d is None or not (50.0 <= d <= 150.0):
        continue
      p = fnum(r.get("spdLim")) or 0.0
      post["in-window map ticks"] += 1
      post["posted known" if p > 0 else "posted UNKNOWN (spdLim 0)"] += 1
  print("POSTED-LIMIT AVAILABILITY on in-window map ticks:", dict(post))
  print("  -> 'never above the posted limit' is unenforceable wherever spdLim is 0; on those ticks")
  print("     the only remaining cap is the driver's own set speed.\n")

  # ---- dump every episode whose counterfactual clears 4.0 at N=5 or N=10 --------------------
  want = []
  for e in eps:
    ticks = e["ticks"]
    lbl = f"{datetime.datetime.fromtimestamp(e['t0'], PT):%m-%d %H:%M:%S}"
    if lbl in ("09-12 14:59:36", "09-12 14:56:30", "09-17 13:23:10", "09-13 13:56:52"):
      want.append((lbl, e))
  for lbl, e in want:
    ticks = e["ticks"]
    print("=" * 118)
    print(f"EPISODE {lbl} PT   {A3.drive_of(ticks[0]['_src'])}   {len(ticks)} ticks")
    print(f"{'PT':<10}{'src':>5}{'vEgo':>7}{'set':>7}{'icbmT':>7}{'modelT':>8}{'mapV':>7}"
          f"{'mapD':>7}{'spdLim':>8}{'visLat':>8}{'visTtc':>7}{'v_vis':>8}{'mdlEndX':>9}"
          f"{'icbmKVis':>10}{'slKActl':>9}{'kPeak':>8}{'a_lat':>7}")
    for r in ticks:
      ve = fnum(r.get("vEgo")) or 0.0
      mv, md = fnum(r.get("mapV")), fnum(r.get("mapDist"))
      ref = fnum(r.get("icbmC")) or fnum(r.get("stockSet")) or fnum(r.get("vSet"))
      mt = None
      if r.get("icbmSrc") == "map" and mv and md and md > 0 and ref:
        mt = A3.model_target(V, mv, md, ref, ve, None, False, floor_raw=True)
      vv, _ = A3.vision_speed(r, "vislat_bold", a_lat_vis)
      ka = fnum(r.get("slKActl"))
      kp = fnum(r.get("kPeak"))
      k_now = max(abs(ka or 0.0), abs(kp or 0.0))
      print(f"{datetime.datetime.fromtimestamp(r['t'], PT):%H:%M:%S}  "
            f"{str(r.get('icbmSrc')):>5}{ve / MPH:>7.1f}"
            f"{(fnum(r.get('stockSet')) or 0) / MPH:>7.1f}"
            f"{(fnum(r.get('icbmT')) or 0) / MPH:>7.1f}"
            f"{(mt / MPH if mt is not None else float('nan')):>8.1f}"
            f"{(mv or 0) / MPH:>7.1f}{(md or 0):>7.0f}"
            f"{(fnum(r.get('spdLim')) or 0) / MPH:>8.1f}"
            f"{(fnum(r.get('visLat')) or 0):>8.2f}{(fnum(r.get('visTtc')) or 0):>7.1f}"
            f"{(vv / MPH if vv not in (None, float('inf')) else float('inf')):>8.1f}"
            f"{(fnum(r.get('mdlEndX')) or 0):>9.0f}"
            f"{(fnum(r.get('icbmKVis')) if r.get('icbmKVis') is not None else float('nan')):>10.5f}"
            f"{abs(ka) if ka is not None else float('nan'):>9.5f}"
            f"{abs(kp) if kp is not None else float('nan'):>8.5f}"
            f"{k_now * ve * ve:>7.2f}")
    # the 30 s after the episode -- the curve itself
    t_end = e["t_end"]
    tail = [r for r in recs if t_end < r["t"] <= t_end + 25.0]
    print(f"  --- the {len(tail)} s after the last decision tick (the curve itself) ---")
    for r in tail:
      ve = fnum(r.get("vEgo")) or 0.0
      ka, kp = fnum(r.get("slKActl")), fnum(r.get("kPeak"))
      k_now = max(abs(ka or 0.0), abs(kp or 0.0))
      if k_now <= 0:
        continue
      print(f"    {datetime.datetime.fromtimestamp(r['t'], PT):%H:%M:%S}  v={ve / MPH:5.1f} mph  "
            f"k={k_now:.5f} (R={1.0 / k_now:6.0f} m)  a_lat={k_now * ve * ve:5.2f}  "
            f"strPrs={r.get('strPrs')} slSat={r.get('slSat')} blnk={r.get('blnk')}")
    print()


if __name__ == "__main__":
  main()
