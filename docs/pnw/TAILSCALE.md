---
updated: 2026-09-30
status: current
---

# Remote SSH over Tailscale (`tailscale2pnw`)

Reach the device's own OpenSSH (port 22, **your existing SSH keys**) from anywhere through your own
[Tailscale](https://tailscale.com) network. The device only ever makes **outbound** connections (Tailscale's control
plane and relays on 443; direct UDP is opportunistic), so it works through carrier CGNAT, guest WiFi and the phone
hotspot. No Tailscale SSH, no inbound port, no LAN reachability needed.

Limit nothing fixes: the device must be **powered**. It shuts itself down about 5 min after the truck is off on low 12 V.

## What the feature is, and is not

- Toggle **Settings > Toggles > Remote SSH (Tailscale)**, **default OFF**. The status shows next to the toggle and in
  its description: `off`, `installing`, `install deferred until parked`, `needs auth key`, `connecting`,
  `connected <tailscale ip>`, or `error <reason>`.
- **Unconfigured is a safe state.** Toggle OFF (fresh install): the `tailscale_pnw` process is never started by the
  manager; nothing is downloaded, written or run. Toggle ON without an auth key: status `needs auth key`; still no
  download, no `tailscaled`, one log line; it polls the key file locally every 30 s.
- Only while ON: the pinned Tailscale `1.102.4` arm64 static release is downloaded (about 36 MB, sha256-verified,
  refused loudly on mismatch) into `/data/pnw/tailscale/` (outside `/data/openpilot`, so the updater's `git clean` cannot
  delete it), and `tailscaled` is run with `--state=/data/pnw/tailscale/tailscaled.state`.
- **The download never happens while driving** (`IsOnroad` and not in Park). **Turn it on for the first time at home on
  WiFi.** A failed download is retried no sooner than 10 min, doubling to 1 h.
- It never touches iptables, nftables, `ip_forward`, NetworkManager or DNS (`--accept-dns=false`): hotspot tethering NAT
  and the LTE metering ladder are unaffected. If `/dev/net/tun` and passwordless `sudo` exist it runs in kernel mode with
  `--netfilter-mode=off`; otherwise in userspace networking as the `comma` user (decided at runtime, logged).
- `tailscaled` is started with `--no-logs-no-support` (no log upload to Tailscale).
- Toggle OFF (any time): `tailscale down`, then `tailscaled` is stopped. Node state is kept, so turning it back ON
  reconnects as the same device without a new key.
- No periodic network use of ours. Tailscale's own keepalive/netmap traffic: **idle MB/day is UNMEASURED** (estimate
  2-10 MB/day; measure `/proc/net/dev` over a day before relying on it on LTE).

## Owner setup (once)

1. Create a tailnet at tailscale.com with your own SSO + MFA. Install Tailscale on the laptop and phone you will SSH from.
2. **Access controls** (policy file). Add the tag owner and replace the default allow-all rule, so your devices reach the
   comma on port 22 **only** and the comma can reach **nothing**:

   ```json
   {
     "tagOwners": { "tag:comma": ["autogroup:admin"] },
     "acls": [
       { "action": "accept", "src": ["autogroup:member"], "dst": ["tag:comma:22"] }
     ]
   }
   ```

   There is deliberately no rule with `tag:comma` as a source. **This ACL step is load-bearing: do NOT keep the default
   `*:*` allow-all rule.** In kernel mode (the device has `/dev/net/tun`) the node accepts tailnet traffic to every
   port bound on 0.0.0.0, so allow-all would expose them all to every device on your tailnet, not just SSH.
3. On the device, **Settings > Developer > Enable SSH** (`SshEnabled`) must be ON, or sshd is not running and nothing answers on port 22.
4. **Settings > Keys > Generate auth key**: tick **Pre-approved** if device approval is on, **Tags: `tag:comma`**, leave
   **Reusable** and **Ephemeral OFF** (ephemeral nodes are deleted while the device is off, which is most of the time).
   Tagged nodes have key expiry disabled, so the device will not silently drop off after 180 days.
5. Put the key on the device **once, over the LAN**, then lock it down (the key never goes in git or chat):

   ```bash
   ssh comma@<lan-ip> 'umask 077; mkdir -p /data/pnw/secrets; cat > /data/pnw/secrets/tailscale.authkey'   # paste the key, Ctrl-D
   ssh comma@<lan-ip> 'chmod 600 /data/pnw/secrets/tailscale.authkey; wc -c /data/pnw/secrets/tailscale.authkey'
   ```

6. Turn the toggle ON (on WiFi, parked). Watch the status: `installing` -> `connecting` -> `connected 100.x.y.z`. The node
   appears in the admin console as `comma-<dongle id>`.
7. Once it is `connected`, the key has done its job (the node key now lives in `/data/pnw/tailscale/`); you may expire the
   auth key in the admin console.

## Using it

```bash
ssh comma@<tailscale-ip>            # or: ssh comma@comma-<dongle-id>   (MagicDNS name, resolved on YOUR side)
```

Authentication is the device's normal SSH key list; Tailscale only provides the network path. In **userspace** mode
tailscaled forwards connections to its own address to `localhost`; that path is expected to reach sshd but has **not been
verified on this device**. If `ssh` times out while the status says `connected`, report it (do not improvise `tailscale serve`).

## If something is wrong

| Status | Meaning / action |
|---|---|
| `needs auth key` | `/data/pnw/secrets/tailscale.authkey` is missing, empty or unreadable. Step 4. (Also shown if the node was logged out and the key is gone.) |
| `install deferred until parked` | You are driving. It installs when you are in Park or the car is off. |
| `error sha256 mismatch ...` | The download did not match the pinned digest; it was **not** installed and will not be retried until the next restart. Report it. |
| `error download failed ...` | No route / DNS / server error; retried every 10 min to 1 h. |
| `error tailscale up failed ...` | Usually a rejected, expired or already-used key (single-use key consumed by an earlier failed enrolment): make a new one. |
| `error not connected for over 3 min: ...` | Can't reach Tailscale (offline, captive portal not yet passed). Text is Tailscale's own health message. |
| `error device needs approval ...` | Approve the device in the admin console (or generate a pre-approved key). |
| `error tailscaled exited rc=...` | Last line of `/data/pnw/tailscale/tailscaled.log` is shown; it is restarted with backoff. |

Every state change is also in the device log (`cloudlog`, lines starting `tailscale:`).

## Revoking / stolen device

- Admin console: **delete** the machine `comma-<dongle id>` (its node key is invalid immediately) and delete/expire the auth
  key. Removing the `tag:comma` rule is optional: with no outbound rules a compromised comma can reach nothing on the tailnet.
- On the device: turn the toggle OFF; to forget the node entirely `rm -rf /data/pnw/tailscale` (this deletes the node
  identity, the binaries and the log) and delete `/data/pnw/secrets/tailscale.authkey`.

## Maintenance

- The release is **pinned** (`system/tailscale/installer.py`: `TAILSCALE_VERSION`, `TAILSCALE_SHA256`). There is no
  auto-update; a version bump is a reviewed commit. The digest is published at
  `https://pkgs.tailscale.com/stable/tailscale_<version>_arm64.tgz.sha256` and must also be recomputed locally.
- Params: `TailscaleEnabled` (BOOL, default 0, persistent), `TailscaleStatus` (STRING, cleared at manager start).
- Code: `system/tailscale/{installer,tailscale_pnw,status}.py`, manager entry `tailscale_pnw` in
  `system/manager/process_config.py`, toggle in `selfdrive/ui/layouts/settings/toggles.py`.
- Tests: `system/tailscale/tests/`, `selfdrive/ui/tests/test_tailscale_toggle.py`.
