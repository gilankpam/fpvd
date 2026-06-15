# Drone deployment

Build and deploy `fpvd` to an OpenIPC drone over SSH/LAN. `fpvd` replaces the
stock `S95waybeam` + `S98wifibroadcast` + `S99dynamic-link-applier` init scripts
and supervises `wfb_*` + `waybeam` + `msposd` (and the adaptive-link loop)
in-process.

## Prerequisites

- **Dev machine:** `nix` (the repo `shell.nix` provides the armv7l/musl cross
  toolchain), `ssh`/`scp`. The cross compiler is not needed on `PATH` — the
  script invokes it via `nix-shell`.
- **Drone:** reachable over SSH as root with key auth. Uses busybox/dropbear
  (no sftp), which is why the script uses `scp -O`.

## Usage

```sh
# First install OR update — auto-detected (presence of /etc/init.d/S99fpvd):
./deploy/drone/deploy.sh --host 192.168.10.152

# Deploy an already-built binary (skip the cross-build):
./deploy/drone/deploy.sh --host 192.168.10.152 --skip-build

# Roll back to the original OpenIPC stack:
./deploy/drone/rollback.sh --host 192.168.10.152 --reboot
```

Host/user can also come from `DRONE_HOST` / `DRONE_USER` env vars (defaults:
`192.168.10.152`, `root`).

## What `deploy.sh` does

1. Cross-builds `fpvd` (`build/ssc338q/fpvd`, Release/static) and strips it (~1.6 MB).
2. Pushes: `/usr/bin/fpvd` (staged as `fpvd.new`, then atomically `mv`'d to dodge
   `ETXTBSY` on a running binary), `radio-up.sh`/`radio-tune.sh` →
   `/usr/libexec/fpvd/`, and `/etc/init.d/S99fpvd`.
3. **First install:** backs up the old init scripts + `waybeam.json`/`wfb.yaml`
   to `/root/fpvd-rollback/`, stops & removes the old stack, starts fpvd.
   **Update:** restarts fpvd with the new binary.
4. Seeds `/etc/fpvd/config.json` from the code defaults (`fpvd --dump-config`)
   if the file is absent — preserves any existing operator config.
5. Verifies (process list, radio channel/txpower, dynamicLink state).

The daemon uses **code-baked defaults** (serialised `Config{}`) plus a single
`/etc/fpvd/config.json` that is deep-merged over them at startup. There is no
`defaults.json` file. `/etc/fpvd/config.json` is never clobbered on update
deploys; only a fresh install (or an absent file) triggers the seed step.

## Notes / gotchas

- The deploy uses fpvd's baseline **channel 132** — set the GS to match.
- To materialise the full default config for inspection or manual editing:
  `fpvd --dump-config > /etc/fpvd/config.json` (safe to run while fpvd is stopped).
- Enabling adaptive link: the in-process controller listens on UDP **:5800**
  (the old `dl-applier` used `:9999`); point the GS dynamic-link config at :5800.
  `curl -X PATCH http://<drone>:8080/config -d '{"dynamicLink":{"enabled":true}}' && curl -X POST http://<drone>:8080/apply`
- HTTP API: `http://<drone>:8080` (LAN and the wfb tunnel `10.5.0.10`).
