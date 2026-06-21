# PNW Pilot — openpilot for the Pacific Northwest

**PNW Pilot** is a fork of openpilot tuned for one job: driving in the PNW, especially the 
**I-5 corridor between Seattle, WA and central Oregon**.


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
- **Drives:** predominantly **Seattle ↔ Corvallis** on I-5; curve and longitudinal behavior is
  calibrated against real drive logs from that corridor.
- **Vehicles:** shaped entirely around two cars — a **2021 Tesla Model S Long Range Plus** (Raven
  class, HW3; the primary I-5 car) and a **2025 Ford F-150 Lightning**. Car-specific code is
  fingerprint-gated, so it stays inert on the other car.
- **Development:** all code is written by **Claude Code** and validated by the **Gemini MCP server**
  running inside Claude Code.
- **Hardware:** only the **comma 3X** is tested. The **comma four is completely untested and will
  likely not work.**

### Enhancements over upstream / xnor

- **Tesla Model S Raven (HW1/HW2/HW3) support** — inherited from the xnor base; the reason PNW
  forks xnor rather than commaai directly.
- **Ford F-150 Lightning (2025)** — fingerprint and SecOC support for the Flash truck.
- **Vision Turn Speed Control (VTSC)** — actively caps cruise speed through curves from the model's
  predicted path curvature, smooth by construction (gentle decel envelope, only ever reduces speed).
  Tuned to the I-5 Terwilliger curve.
- **Conditional Experimental Switching (CES)** — chill by default, automatically switches to
  Experimental mode for curves, low-speed, stop-lights, and slow leads; with a per-car gentle
  profile and an Off / Light / Standard selector.
- **OSM speed-limit display** — shows the current posted limit and warns on lower limits, sourced
  from the bundled PNW map data.
- **Nudgeless lane change + no-disengage-on-brake** — hands-light lane changes and braking that
  doesn't kick you out of engagement.
- **Blind-spot monitoring** and a **last-known-car indicator** on the offroad screen.
- **Networking** — tethering/hotspot NAT fix, perpetual tethering, priority-WiFi switching,
  GPS-gated WiFi scanning with a Set Home Location button, and an LTE throttle guard.
- **Smarter drive upload** — two-pass upload (small files automatically, video on real WiFi) with a
  deleter that preserves anything not yet uploaded.

### Installing PNW Pilot

PNW Pilot installs the same way as any custom openpilot fork — by entering its installer URL on the
comma device's setup screen. **Only the comma 3X is tested.**

1. On the comma 3X, do a factory reset / start the **Setup** flow (Settings → Device → Reset, or a
   fresh device boot).
2. Connect the device to Wi-Fi.
3. When asked for the software to install, choose **Custom Software** and enter one of:
   - **Production (stable):** `installer.comma.ai/dirkpetersen/pnwprod`
   - **Test / staging:** `installer.comma.ai/dirkpetersen/pnwtest`
4. Confirm; the device downloads and installs PNW Pilot, then reboots into it.

These URLs resolve through GitHub's `dirkpetersen/openpilot` → `dirkpetersen/pnw-pilot` redirect, so
the comma installer (which clones `<user>/openpilot`) finds the PNW fork automatically. Use
`pnwtest` to validate a build, then `pnwprod` for the stable install.

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
  <a href="https://comma.ai/shop">Try it on a comma four</a>
</h3>

Quick start: during the comma device setup, enter the custom software URL `installer.comma.ai/xnor-tech/xnor`

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
1. **Supported Device:** a comma four, available at [comma.ai/shop/comma-four](https://www.comma.ai/shop/comma-four).
2. **Software:** The setup procedure for the comma 3X allows users to enter a URL for custom software. Use the URL `installer.comma.ai/xnor-tech/xnor` to install the recommended precompiled version.
3. **Supported Car:** Ensure that you have one of [the 300+ supported cars](docs/CARS.md).
4. **Car Harness:** You will also need a [car harness](https://comma.ai/shop/car-harness) to connect your comma four to your car.

We have detailed instructions for [how to install the harness and device in a car](https://comma.ai/setup). Note that it's possible to run openpilot on [other hardware](https://blog.comma.ai/self-driving-car-for-free/), although it's not plug-and-play.


### Available Versions

#### PNW (this distribution)

The two PNW install branches for the comma device setup screen. Both resolve through the
`dirkpetersen/openpilot` → `dirkpetersen/pnw-pilot` redirect.

| Version | Device | Car | Installation URL |
|---------|--------|-----|------------------|
| `pnwprod` | comma 3X | Tesla Model S (Raven, HW3), Ford F-150 Lightning 2025 | https://installer.comma.ai/dirkpetersen/pnwprod |
| `pnwtest` | comma 3X | Tesla Model S (Raven, HW3), Ford F-150 Lightning 2025 | https://installer.comma.ai/dirkpetersen/pnwtest |

`pnwprod` is the stable production build; `pnwtest` is the test/staging build validated before
promotion to `pnwprod`.

#### Precompiled (recommended)

These branches come precompiled and install significantly faster — no on-device build step required.

| Version | Device | Car | Installation URL |
|---------|--------|-----|------------------|
| `xnor` | comma four, comma 3X | Tesla Model S/3/X/Y (HW1–HW4), MG 5 EV, MG ZS EV | https://installer.comma.ai/xnor-tech/xnor |
| `xnor-c3` | comma three (deprecated) | Tesla Model S/3/X/Y (HW1–HW4), MG 5 EV, MG ZS EV | https://installer.comma.ai/xnor-tech/xnor-c3 |
| `tesla-unity` | comma three, comma 3X | Tesla preAP Model S | https://installer.comma.ai/xnor-tech/tesla-unity |

#### Development (source)

These are the development branches. They compile on-device after installation, which takes longer.

| Version | Device | Car | Installation URL |
|---------|--------|-----|------------------|
| `xnor-dev` | comma four, comma 3X | Tesla Model S/3/X/Y (HW1–HW4), MG 5 EV, MG ZS EV | https://installer.comma.ai/xnor-tech/xnor-dev |
| `xnor-c3-dev` | comma three (deprecated) | Tesla Model S/3/X/Y (HW1–HW4), MG 5 EV, MG ZS EV | https://installer.comma.ai/xnor-tech/xnor-c3-dev |

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

By default, openpilot uploads driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software, and users can disable data collection if they wish.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.
</details>
