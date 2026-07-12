# PNW Pilot — openpilot for the Pacific Northwest

**PNW Pilot** is a personal production distribution of [openpilot](https://github.com/commaai/openpilot),
built on the [xnor-tech](https://github.com/xnor-tech) fork for its legacy Tesla (Raven) support and
tuned for one job: driving the PNW, especially the **I-5 corridor between Seattle, WA and
central-western Oregon**. It serves exactly two cars — a **2021 Tesla Model S (Raven, HW3)** and a
**2025 Ford F-150 Lightning** — with one comma 3X that is physically moved between them.

> ### Standing on the shoulders of the openpilot community
> Nearly every mechanism in this fork was invented by someone else first — comma.ai, xnor-tech,
> sunnypilot, BluePilot, FrogPilot, pfeiferj/mapd, and more. **See [CREDITS.md](CREDITS.md) for the
> people behind each feature.** If you like something here, they built the idea; we ported and tuned it.

<div align="center"> <img width="512" height="432" alt="image" src="https://github.com/user-attachments/assets/5dca7817-02cc-431b-83e3-d7fec3733ada" /> </div>


```
commaai/openpilot          upstream
  └─ xnor-tech/openpilot   adds full legacy Tesla HW1/HW2/HW3 (Raven) support
       └─ dirkpetersen/pnw-pilot   ← this distribution (Pacific Northwest)
```

### Focus

- **Region:** map data ships for **Washington, Oregon, and Idaho** by default — the first map
  download auto-arms on a fresh device, no settings page required. (British Columbia is optional
  and can be added to the state list.)
- **Drives:** predominantly **Seattle ↔ central-western Oregon** on I-5; curve and
  longitudinal behavior is calibrated against real drive logs from that corridor (plus I-90
  Snoqualmie Pass and US-12).
- **Vehicles:** shaped entirely around two cars — a **2021 Tesla Model S Long Range Plus** (Raven
  class, HW3; the primary I-5 car) and a **2025 Ford F-150 Lightning**. Car-specific code is
  fingerprint-gated, so it stays inert on the other car.
- **Development:** all code is written by **Claude Code** and validated by the **Gemini MCP server**
  running inside Claude Code.
- **Hardware:** only the **comma 3X** is tested. The **comma four is completely untested and will
  likely not work.**

## What PNW Pilot adds on top of xnor / upstream

### Speed control

- **CES — Conditional Experimental Switching.** openpilot's relaxed "chill" longitudinal mode stays
  the default, and the car automatically switches into end-to-end **Experimental mode** only when
  the situation calls for it: upcoming curves (map + vision), low speeds, stop lights, and slow
  leads. A three-state selector (Off / Light / Standard) and an on-screen status button control it.
  Works on both cars whenever openpilot is doing the longitudinal control. Concept from FrogPilot's
  Conditional Experimental Mode.
- **VTSC + map curves.** Vision Turn Speed Control caps cruise speed through curves from the
  model's predicted path curvature, folded together with OSM map-curve data — so it also slows for
  sharp curves the camera can't see yet (500 m lookahead, apex-timed so the curve entrance is the
  slowest point). Reduce-only, with a gentle regen-style decel envelope; tuned against I-5
  Terwilliger and I-90 Snoqualmie Pass drive logs.
- **ICBM — stock-ACC curve slow-downs (Lightning).** When the truck is on its *stock* adaptive
  cruise (openpilot longitudinal off), the same VTSC curve math taps the cruise **SET− button** to
  step the set speed down ahead of curves — decrease-only, no panda change needed. The Alpha
  Longitudinal toggle acts as the A/B switch between openpilot longitudinal and ICBM. Idea invented
  by sunnypilot, Ford port by BluePilot.
- **Red-light guard.** The acceleration-zone logic that keeps CES calm during on-ramp merges is
  hard-gated so it can never suppress the stop-for-red-light behavior on an intersection approach
  (root-caused from a live incident, pinned by regression tests).

### Ford F-150 Lightning — lateral (steering)

- **4-signal lateral control** — the BlueCruise-grade curvature command set (ported from
  BluePilot), giving noticeably stronger, smoother steering than the stock 2-signal path, with a
  **matching panda safety ruleset** (latch-hardened, written in C, capability-gated to the
  Lightning so the Tesla is untouched).
- **Turn-exit predicted-curvature blend** — blends the model's predicted curvature on turn exit so
  the truck unwinds the wheel like a human instead of overshooting.
- **Human-turn reset** — when the driver turns the wheel themselves, the controller's state resets
  cleanly instead of fighting the correction; lane changes get their own command scaling.

### Ford F-150 Lightning — longitudinal

- **LongitudinalExt highway follow control** — BluePilot's lead-follow shaping (classifies the lead
  as gaining / pacing / trailing and shapes gas and brake per state, with highway speed deadband).
  Ships in the tree but is **dormant until the Alpha Longitudinal toggle is on**; with it off the
  truck keeps stock behavior.

### Cluster & HUD

- **BlueCruise cluster display** — the Ford instrument cluster shows the blue BlueCruise engaged
  state when openpilot is steering, so engagement is readable at a glance in the factory display.
- **Rich cluster messaging** — openpilot status messages rendered in the Ford cluster (Cancelled,
  lane departure, and friends). Display-only; no reflash needed.
- **Confidence ball** — a comma-four-style indicator (ported via sunnypilot) on the comma 3X
  screen: a small gradient ball on the right edge showing the driving model's own confidence in its
  current plan (green high, amber mid, red low).

### Tesla Model S (Raven)

- **Native Raven support** — full legacy Tesla HW1/HW2/HW3 support inherited from the xnor-tech
  base: `tesla_legacy` panda safety, the legacy CAN stack, radar, and ignition detection. This is
  the reason PNW forks xnor rather than commaai directly.
- **Blind-spot monitoring** — the car's own blind-spot state read from the Autopilot status CAN
  message, used to make lane changes safer (and to gate nudgeless lane changes away from an
  occupied lane).

### Lane changes & engagement (both cars)

- **Nudgeless lane change** — signal, and after a short hold the lane change starts without a
  steering nudge. **No-disengage-on-brake** — a brake tap doesn't kick you out of engagement. Both
  default off.

### Maps & location

- **OSM speed limits + curve speeds** — [pfeiferj/mapd](https://github.com/pfeiferj/mapd) provides
  posted speed limits (with lower-limit warnings on screen) and the map-curvature feed used by the
  curve slow-down features. Map data for **Washington, Oregon, and Idaho** downloads automatically
  on first launch.
- **"Happening Ahead" overlay** — on freeways, a lower-left panel shows the nearest **police
  report** (Waze), **rest area**, and **EV fast charger** ahead along your route (nearest-anything
  within 3 mi on surface streets). Display-only — it never affects steering or speed.

### Networking & uploads

- **Networking** — perpetual tethering with a NAT fix, priority-WiFi switching, GPS-gated WiFi
  scanning with a Set Home Location button, captive-portal auto-login for visitor hotspots, and an
  LTE throttle guard.
- **Smarter drive upload** — two-pass upload (small log files automatically on any network, HD
  video only on real non-metered WiFi), an optional **Defer HD Video Upload** toggle, and a deleter
  that never removes anything not yet uploaded.

### Robustness

- **Capability view (`pnw_vehicle`)** — feature code never checks car fingerprints; each car
  declares its capabilities once, and features consume those. The Tesla can't be impaired by truck
  work, and vice versa.
- **Crash logger + auto-restart** — the car-interface process logs fatal crashes before dying and
  is automatically restarted by the process manager (same policy as the UI).
- **Stock-fallback armor** — the ported Ford lateral/longitudinal extensions are wrapped so any
  fault in the extension falls back to stock openpilot behavior instead of crashing the drive.
- **Engaged-path smoke tests** — the code paths that run while engaged are exercised by tests
  before anything ships.

### Deployment

- **Auto-update channels** — the device tracks a git branch per channel (dev / test / prod) and
  installs updates itself at ignition-off; shipping a change is a push.
- **Pre-drive sync discipline** — the device is verified in sync with its channel before a drive,
  never mid-drive.

### Safety posture

Every feature above **defaults to stock openpilot behavior**: new toggles default **off**, panda
safety rules are never weakened, and car-specific code is capability-gated so it is inert on the
other car. Priority order: safety > stability > quality > features.

### Installing PNW Pilot

PNW Pilot installs the same way as any custom openpilot fork — by entering its installer URL on the
comma device's setup screen. **Only the comma 3X is tested.**

1. On the comma 3X, do a factory reset / start the **Setup** flow (Settings → Device → Reset, or a
   fresh device boot).
2. Connect the device to Wi-Fi.
3. When asked for the software to install, choose **Custom Software** and enter one of:
   - **Production (stable):** `installer.comma.ai/dirkpetersen/3pnw`
   - **Test / staging:** `installer.comma.ai/dirkpetersen/3testpnw`
   - **Development (not recommended):** `installer.comma.ai/dirkpetersen/3devpnw`
4. Confirm; the device downloads and installs PNW Pilot, then reboots into it.

These URLs resolve through GitHub's `dirkpetersen/openpilot` → `dirkpetersen/pnw-pilot` redirect, so
the comma installer (which clones `<user>/openpilot`) finds the PNW fork automatically. Use
`3testpnw` to validate a build, then `3pnw` for the stable install; `3devpnw` is the active
development channel and can change (or break) daily.

---

<div align="center" style="text-align: center;">

<h1>openpilot</h1>

<p>
  <b>openpilot is an operating system for robotics.</b>
  <br>
  Currently, it upgrades the driver assistance system in 300+ supported cars.
</p>

<h3>
  <a href="https://docs.comma.ai">Docs</a>
  <span> · </span>
  <a href="https://docs.comma.ai/contributing/roadmap/">Roadmap</a>
  <span> · </span>
  <a href="https://github.com/commaai/openpilot/blob/master/docs/CONTRIBUTING.md">Contribute</a>
  <span> · </span>
  <a href="https://discord.comma.ai">Community</a>
  <span> · </span>
  <a href="https://comma.ai/shop">Try it on a comma 3X</a>
</h3>

Quick start: `bash <(curl -fsSL openpilot.comma.ai)`

[![openpilot tests](https://github.com/commaai/openpilot/actions/workflows/tests.yaml/badge.svg)](https://github.com/commaai/openpilot/actions/workflows/tests.yaml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![X Follow](https://img.shields.io/twitter/follow/comma_ai)](https://x.com/comma_ai)
[![Discord](https://img.shields.io/discord/469524606043160576)](https://discord.comma.ai)

</div>

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://github.com/commaai/openpilot/assets/8762862/2f7112ae-f748-4f39-b617-fabd689c3772"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://github.com/commaai/openpilot/assets/8762862/92351544-2833-40d7-9e0b-7ef7ae37ec4c"></a></td>
    <td><a href="https://youtu.be/SUIZYzxtMQs" title="A drive to Taco Bell"><img src="https://github.com/commaai/openpilot/assets/8762862/05ceefc5-2628-439c-a9b2-89ce77dc6f63"></a></td>
  </tr>
</table>


Using openpilot in a car
------

To use openpilot in a car, you need four things:
1. **Supported Device:** a comma 3X, available at [comma.ai/shop](https://comma.ai/shop/comma-3x).
2. **Software:** The setup procedure for the comma 3X allows users to enter a URL for custom software. Use the URL `openpilot.comma.ai` to install the release version.
3. **Supported Car:** Ensure that you have one of [the 275+ supported cars](docs/CARS.md).
4. **Car Harness:** You will also need a [car harness](https://comma.ai/shop/car-harness) to connect your comma 3X to your car.

We have detailed instructions for [how to install the harness and device in a car](https://comma.ai/setup). Note that it's possible to run openpilot on [other hardware](https://blog.comma.ai/self-driving-car-for-free/), although it's not plug-and-play.


### Branches

Running `master` and other branches directly is supported, but it's recommended to run one of the following prebuilt branches:

| comma four branch      | comma 3X branch        | URL                                    | description                                                                         |
|------------------------|------------------------|----------------------------------------|-------------------------------------------------------------------------------------|
| `release-mici`         | `release-tizi`         | openpilot.comma.ai                     | This is openpilot's release branch.                                                 |
| `release-mici-staging` | `release-tizi-staging` | openpilot-test.comma.ai                | This is the staging branch for releases. Use it to get new releases slightly early. |
| `nightly`              | `nightly`              | openpilot-nightly.comma.ai             | This is the bleeding edge development branch. Do not expect this to be stable.      |
| `nightly-dev`          | `nightly-dev`          | installer.comma.ai/commaai/nightly-dev | Same as nightly, but includes experimental development features for some cars.      |

To start developing openpilot
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot).

* Join the [community Discord](https://discord.comma.ai)
* Check out [the contributing docs](docs/CONTRIBUTING.md)
* Check out the [openpilot tools](tools/)
* Code documentation lives at https://docs.comma.ai
* Information about running openpilot lives on the [community wiki](https://github.com/commaai/openpilot/wiki)

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs#open-positions) and offers lots of [bounties](https://comma.ai/bounties) for external contributors.

Safety and Testing
----

* openpilot observes [ISO26262](https://en.wikipedia.org/wiki/ISO_26262) guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software-in-the-loop [tests](.github/workflows/tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software-in-the-loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware-in-the-loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware-in-the-loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

<details>
<summary>MIT Licensed</summary>

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**
</details>

<details>
<summary>User Data and comma Account</summary>

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.
</details>
