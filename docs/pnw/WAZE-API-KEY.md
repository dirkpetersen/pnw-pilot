# pnw-pilot — Waze police-alert API key

pnw-pilot shows **police reports ahead** (Waze data, via OpenWebNinja) on the "Happening Ahead"
overlay. **By default you configure nothing** — the device uses a shared **central AWS proxy** that
holds one API key server-side and enforces spend caps. No per-device key, nothing to set up.

---

## Default (recommended): the central AWS proxy

Out of the box the device polls a **keyless** proxy
(`https://jh69za4byd.execute-api.us-west-2.amazonaws.com/alerts`). It:

- needs **no API key on the device**,
- is spend-capped — **750 calls/day per device** and **$25/month** across the whole fleet — so you
  can never run up a surprise bill (at the cap it just stops fetching until the month rolls over),
- only polls while you're actually driving (a speed gate; currently **20 mph** for testing, normally
  45 mph), and dedups the fleet so the same road is one upstream call.

If you never create the override file below, this is what you get. **Recommended for almost everyone.**

---

## Optional: use YOUR OWN OpenWebNinja key (direct mode)

Prefer to run your own OpenWebNinja subscription and bypass the shared proxy? Drop your key in one
small file on the device.

> ⚠️ **Tradeoff:** direct mode has **NO budget or per-device tracking** — you own your own OpenWebNinja
> PAYG spend, with no $25 cap protecting you. Only do this if you want to manage your own key/billing.

1. **Get a key** at <https://app.openwebninja.com/api/waze> — it looks like `ak_xxxxxxxx...`.

2. **SSH into the comma** (user is `comma`; find the device's IP on your network — e.g. your router's
   client list, or the comma's WiFi settings screen):
   ```bash
   ssh comma@<device-ip>
   ```

3. **Write the override file:**
   ```bash
   mkdir -p /data/pnw/location
   cat > /data/pnw/location/police_proxy.json <<'EOF'
   {"source": "direct", "key": "ak_YOUR_OPENWEBNINJA_KEY"}
   EOF
   ```

4. **Done — no reboot needed.** The location daemon re-reads this file every poll cycle (~once/min),
   so it switches to your key within a minute. The file lives **outside** the git tree
   (`/data/pnw/...`), so it survives software updates and reinstalls and never enters the repo.

### Go back to the central proxy
Just delete the file:
```bash
rm /data/pnw/location/police_proxy.json
```
Within a minute you're back on the keyless central proxy (caps enforced again).

---

## Reference — `police_proxy.json`

| Contents | Behavior |
|---|---|
| *(no file)* | **Default.** Keyless central AWS proxy (caps enforced). |
| `{"source":"direct","key":"ak_..."}` | Direct to OpenWebNinja with **your** key — no caps, you own the spend. |
| `{"source":"proxy"}` | Force the central proxy even if a key is present. |

- Direct mode calls `https://api.openwebninja.com/waze/alerts-and-jams` with header `x-api-key`.
- Police data is **display-only**. The "N mi ahead" alert is freeway-only; below the speed gate the
  overlay reads a neutral grey **"speed <Nmph"** — that's normal (it just isn't polling while slow).
- The key file is the one piece of device state that must **never** be committed to the repo.
