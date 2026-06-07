# Phase 3b — Wire Shrink + HELLO Removal + GS Teardown Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish Phase 3 by shrinking the GS→drone Decision wire to `{mcs}`-only (v3), removing HELLO/HelloAck/PING/PONG + the sync-gate, and deleting the now-dead GS bitrate/FEC/predictor/drone-config machinery (gutting `Policy` to selector→`{mcs}`).

**Architecture:** Pure subtraction around a single wire-version bump. The drone already computes its own bitrate/k/n/depth/tx_power (Phase 3a), so the wire and the GS machinery that produced those fields are dead. Two halves — drone (`drone/src/dynlink/`, C++) and GS (`gs/fpvdgs/dynlink/`, Python) — have **independent test suites** coupled only by the v3 byte layout (defined once below) and the coordinated deploy. Order: drone first (Part A, establishes v3 wire + removes drone HELLO), then GS (Part B), then the coordinated flag-day deploy (Part C).

**Tech Stack:** C++17 + doctest (drone); Python 3.13 + pytest (GS); nlohmann/json; CMake.

**Spec:** `docs/superpowers/specs/2026-06-07-phase3b-wire-shrink-hello-removal-design.md`. **Prereq: Phase 3a deployed + hardware-validated (met 2026-06-07).**

**Test commands:**
- Drone: `cd /home/gilankpam/Projects/drone/fpvd/drone && cmake --build build -j && ./build/fpvd_tests` (filter: `--test-case="*wire*"`). **NOT `ctest`.** Baseline: **300 passed**.
- GS: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/ -q` (one file: `… tests/unit/test_dl_wire.py -q`). Baseline: **291 passed, 1 skipped**.
- Git from repo root `/home/gilankpam/Projects/drone/fpvd`.

---

## The v3 Decision wire (THE coupling point — both sides must match byte-for-byte)

```
off  size  field
 0    4    magic    = 0x444C4B31 ('DLK1')   # unchanged
 4    1    version  = 3                       # was 2 — the compatibility gate
 5    1    flags
 6    4    sequence
10    1    mcs
11    4    crc32(bytes[0..10])
= 15 bytes on-wire  (11 payload + 4 CRC)
```
Dropped vs v2: `_pad(6..7)`, `timestamp_ms(12..15)`, `bandwidth(17)`, `tx_power_dBm(18)`, `k(19)`, `n(20)`, `depth(21)`, `bitrate_kbps(22..23)`, `fps(24)`, `_pad2(25..26)`. A v2 packet (version byte 2, 31 bytes) decoded by a v3 endpoint fails `BadVersion`/`Short`; a v3 packet (15 bytes) fails a v2 endpoint's size/CRC check — the hard flag-day.

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| **DRONE** | | |
| `drone/src/dynlink/runtime_config.hpp/.cpp` | modify | add `linkBandwidth`; drop `helloMtuBytes`/`helloFps` |
| `drone/src/dynlink/controller.cpp/.hpp` | modify | `d.bandwidth ← cfg.linkBandwidth`; remove HELLO send/recv + `generationId`; remove `hello_` |
| `drone/src/dynlink/wire.hpp/.cpp` | modify | v3 Decision (15B); delete Hello/HelloAck/Ping/Pong structs+funcs+constants; `peekKind`→Decision-only |
| `drone/src/dynlink/hello.hpp/.cpp` | **delete** | `HelloSm` |
| `drone/src/daemon.cpp/.hpp` | modify | drop `dlGenerationId_` + the `start()` generationId arg |
| `drone/CMakeLists.txt` | modify | drop `hello.cpp`, `test_dl_hello.cpp` |
| `drone/tests/unit/test_dl_wire.cpp` | modify | v3 Decision round-trip; delete Ping/Pong/Hello cases |
| `drone/tests/unit/test_dl_hello.cpp` | **delete** | |
| `drone/tests/unit/test_dl_runtime_config.cpp` | modify | assert `linkBandwidth`; drop hello fields |
| `drone/tests/integration/test_dl_controller.cpp` | modify | drop HELLO test + `generationId` args |
| **GS** | | |
| `gs/fpvdgs/dynlink/wire.py` | modify | v3 encode; delete Hello/HelloAck/Ping/Pong + constants |
| `gs/fpvdgs/dynlink/decision.py` | modify | shrink `Decision` to `{mcs}` (+ telemetry) |
| `gs/fpvdgs/dynlink/policy.py` | modify | gut to selector→`{mcs}`; remove sync-gate/bitrate/FEC/predictor/trailing/tx_power |
| `gs/fpvdgs/dynlink/controller.py` | modify | remove HELLO listener + `drone_config` |
| `gs/fpvdgs/dynlink/return_link.py` | modify | drop `send_hello_ack`/`send_ping` (keep `send`) |
| `gs/fpvdgs/dynlink/config_build.py` | modify | deprecate retired bitrate/FEC/predictor knobs |
| `gs/fpvdgs/dynlink/{drone_config,tunnel_listener,predictor,bitrate,dynamic_fec}.py` | **delete** | |
| `gs/tests/unit/test_dl_{bitrate,predictor,dynamic_fec,drone_config,policy_dynamic_fec_e2e,policy_trailing}.py` | **delete** | |
| `gs/tests/unit/test_dl_{wire,wire_contract,decision,policy_leading,controller,config_build,imports}.py` | modify | v3 + remove sync/HELLO refs |

Order: A1→A2→A3→A4 (drone), then B1→B2→B3→B4→B5 (GS), then C1 (deploy). Each task is one commit; the relevant suite stays green at every commit.

---

# Part A — Drone

## Task A1: Decouple `bandwidth` from the wire (`linkBandwidth` from config)

**Files:** Modify `drone/src/dynlink/runtime_config.hpp`, `runtime_config.cpp`, `controller.cpp`; Test `drone/tests/unit/test_dl_runtime_config.cpp`.

The drone reads `d.bandwidth` (the 20/40 radiotap value) from the wire to feed `applyLocalCompute` + `dispatchTxApply`. v3 drops bandwidth from the wire, so the drone must source it from `link.width` config. Add a `linkBandwidth` runtime field and set `d.bandwidth` from it right after decode.

- [ ] **Step 1: Write the failing test** — append to `drone/tests/unit/test_dl_runtime_config.cpp`:
```cpp
TEST_CASE("buildDlSnapshot maps link.width to linkBandwidth (radiotap value)") {
    fpvd::Config c{};
    c.link.width = 20;
    auto s20 = fpvd::dynlink::buildDlSnapshot(c, "wlan0");
    CHECK(s20.linkBandwidth == 20);
    c.link.width = 40;
    auto s40 = fpvd::dynlink::buildDlSnapshot(c, "wlan0");
    CHECK(s40.linkBandwidth == 40);
}
```
(`buildDlSnapshot` maps `c.link.width` through `modulationWidth()` from `dynlink/link_width.hpp` to the radiotap 20/40 value. Confirm `modulationWidth` is the existing helper that the controller's radio path uses; if the value is already 20/40 for a 20/40 input, the mapping is identity for these cases but keeps HT40-as-20 handling correct.)

- [ ] **Step 2: Run to verify it fails** — `cd /home/gilankpam/Projects/drone/fpvd/drone && cmake --build build -j 2>&1 | tail -5` → FAIL (`no member named 'linkBandwidth'`).

- [ ] **Step 3: Add the field** — in `drone/src/dynlink/runtime_config.hpp`, add to `struct DlRuntimeConfig` (near the other static link fields, e.g. after `bool ldpc;`):
```cpp
    uint8_t  linkBandwidth{20};   // radiotap 20/40 from link.width (wire no longer carries it)
```

- [ ] **Step 4: Map it** — in `drone/src/dynlink/runtime_config.cpp` `buildDlSnapshot`, near where `stbc`/`ldpc` are set, add:
```cpp
    s.linkBandwidth = static_cast<uint8_t>(modulationWidth(c.link.width));
```
Add `#include "dynlink/link_width.hpp"` at the top if not already included (check; `modulationWidth` lives there).

- [ ] **Step 5: Use it in the controller** — in `drone/src/dynlink/controller.cpp`, in the decision branch immediately after `decodeDecision(...)` succeeds and BEFORE `applyLocalCompute(cfg, d)` (which currently reads `d.bandwidth`), set:
```cpp
                        // v3 wire carries only {mcs}; bandwidth is static config.
                        d.bandwidth = cfg.linkBandwidth;
```
(This makes the drone use the config bandwidth even while the wire is still v2 — `d.bandwidth` decoded from the wire is overwritten. Harmless now; required once the wire drops bandwidth in A2.)

- [ ] **Step 6: Run to verify** — `cmake --build build -j && ./build/fpvd_tests --test-case="*linkBandwidth*"` → PASS; full suite `./build/fpvd_tests 2>&1 | tail -3` green (301 passed). Existing `test_dl_controller` decision cases use `d.bandwidth=20/40` matching `link.width` defaults, so applied radio bandwidth is unchanged.

- [ ] **Step 7: Commit**
```bash
git add drone/src/dynlink/runtime_config.hpp drone/src/dynlink/runtime_config.cpp drone/src/dynlink/controller.cpp drone/tests/unit/test_dl_runtime_config.cpp
git commit -m "feat(drone/dynlink): source bandwidth from link.width config (Phase 3b prep)"
```

---

## Task A2: Shrink the Decision wire to v3 (`{mcs}`)

**Files:** Modify `drone/src/dynlink/wire.hpp`, `wire.cpp`; Test `drone/tests/unit/test_dl_wire.cpp`.

Shrink `encodeDecision`/`decodeDecision` to the 15-byte v3 layout. The `Decision` struct keeps its fields (the drone fills `bandwidth` from config in A1 and computes `k/n/depth/bitrate/fps` via `applyLocalCompute`); they're simply no longer on the wire. **Do NOT touch Hello/Ping/Pong here** — that's A4.

- [ ] **Step 1: Update the wire round-trip test** — in `drone/tests/unit/test_dl_wire.cpp`, replace the Decision round-trip / golden test(s) with the v3 expectation. The v3 round-trip:
```cpp
TEST_CASE("v3 Decision encodes to 15 bytes and round-trips mcs/sequence/flags") {
    Decision d{};
    d.flags = 0; d.sequence = 0xAABBCCDD; d.mcs = 5;
    uint8_t buf[64];
    size_t n = encodeDecision(d, buf, sizeof(buf));
    CHECK(n == 15);
    CHECK(buf[4] == 3);            // version
    Decision out{};
    CHECK(decodeDecision(buf, n, out) == DecodeResult::Ok);
    CHECK(out.version == 3);
    CHECK(out.sequence == 0xAABBCCDD);
    CHECK(out.mcs == 5);
    CHECK(out.flags == 0);
}

TEST_CASE("v3 decode rejects a v2-sized/old-version buffer") {
    // 31-byte all-zero buffer (or a v2 packet) must not decode as v3.
    uint8_t buf[31] = {0};
    put_u32_test(buf, 0x444C4B31u);   // helper or inline: write the magic
    buf[4] = 2;                        // version 2
    Decision out{};
    DecodeResult r = decodeDecision(buf, sizeof(buf), out);
    CHECK((r == DecodeResult::BadVersion || r == DecodeResult::BadCrc));
}
```
(For the magic write in the rejection test, reuse whatever the file already does to lay down bytes — if there's no `put_u32` test helper, set `buf[0..3]` to `0x31,0x4B,0x4C,0x44` for big-endian `0x444C4B31`, or just craft the 4 magic bytes inline. The key assertion is that a `version=2` buffer does NOT return `Ok`.) Delete any existing v2 Decision golden/round-trip cases this replaces.

- [ ] **Step 2: Run to verify it fails** — `cmake --build build -j && ./build/fpvd_tests --test-case="*v3 Decision*"` → FAIL (encode still 31 bytes / version 2).

- [ ] **Step 3: Update the constants** — in `drone/src/dynlink/wire.hpp`:
```cpp
inline constexpr uint8_t  kWireVersion      = 3;
inline constexpr size_t   kWirePayloadSize  = 11;          // magic+ver+flags+seq+mcs
inline constexpr size_t   kWireOnWire       = 15;          // 11 payload + 4 CRC
```

- [ ] **Step 4: Rewrite encode/decode** — in `drone/src/dynlink/wire.cpp`, replace `encodeDecision` and `decodeDecision` bodies with the v3 layout:
```cpp
size_t encodeDecision(const Decision& d, uint8_t* buf, size_t buflen) {
    if (buflen < kWireOnWire) return 0;
    std::memset(buf, 0, kWireOnWire);
    put_u32(&buf[0], kWireMagic);        // [0..3]  magic
    buf[4] = kWireVersion;               // [4]     version = 3
    buf[5] = d.flags;                    // [5]     flags
    put_u32(&buf[6], d.sequence);        // [6..9]  sequence
    buf[10] = d.mcs;                     // [10]    mcs
    uint32_t c = crc32(buf, kWirePayloadSize);
    put_u32(&buf[kWirePayloadSize], c);  // [11..14] crc32
    return kWireOnWire;
}

DecodeResult decodeDecision(const uint8_t* buf, size_t len, Decision& d) {
    if (len < kWireOnWire) return DecodeResult::Short;
    uint32_t magic = get_u32(&buf[0]);
    if (magic != kWireMagic) return DecodeResult::BadMagic;
    uint8_t version = buf[4];
    if (version != kWireVersion) return DecodeResult::BadVersion;
    uint32_t crc_wire = get_u32(&buf[kWirePayloadSize]);
    uint32_t crc_calc = crc32(buf, kWirePayloadSize);
    if (crc_wire != crc_calc) return DecodeResult::BadCrc;
    d = {};
    d.magic    = magic;
    d.version  = version;
    d.flags    = buf[5];
    d.sequence = get_u32(&buf[6]);
    d.mcs      = buf[10];
    return DecodeResult::Ok;   // bandwidth/k/n/depth/bitrate/fps left default; filled by config + applyLocalCompute
}
```

- [ ] **Step 5: Run to verify** — `cmake --build build -j && ./build/fpvd_tests --test-case="*v3 Decision*,*v3 decode*"` → PASS. Then full suite `./build/fpvd_tests 2>&1 | tail -3` green (301). The controller integration tests still pass: they inject a Decision and assert `sawRadio(mcs,bw)`/`sawFec(k,n)` — `mcs` comes from the wire, `bw` from `cfg.linkBandwidth` (A1), `k/n` from `applyLocalCompute`. If an integration test crafts a raw v2 wire buffer by hand, update it to the v3 encoder (use `encodeDecision`). Fix any test that hard-codes the 31-byte size.

- [ ] **Step 6: Commit**
```bash
git add drone/src/dynlink/wire.hpp drone/src/dynlink/wire.cpp drone/tests/unit/test_dl_wire.cpp
git commit -m "feat(drone/dynlink): v3 Decision wire ({mcs}-only, 15 bytes)"
```

---

## Task A3: Remove the drone HELLO machinery + `generationId`

**Files:** Modify `drone/src/dynlink/controller.cpp`, `controller.hpp`, `daemon.cpp`, `daemon.hpp`.

Remove the HELLO send loop, the HelloAck receive arm, and the `generationId` plumbing. Leave `hello.cpp/hpp` + the wire Hello structs in place for now (they still compile); A4 deletes them. This keeps each commit green.

- [ ] **Step 1: Controller `start()` signature** — in `drone/src/dynlink/controller.hpp`, change `void start(const DlRuntimeConfig& snap, uint32_t generationId);` to `void start(const DlRuntimeConfig& snap);`. Remove the `std::atomic<uint32_t> generationId_{0};` member and the `std::optional<HelloSm> hello_;` member. Remove `#include "dynlink/hello.hpp"`.

- [ ] **Step 2: Controller body** — in `drone/src/dynlink/controller.cpp`, in `start(...)`: drop the `generationId` param, `generationId_.store(generationId);`, and `hello_.emplace(...)` + `hello_->setVanilla(...)`. In `run(...)`: remove the entire HELLO machinery — the `helloTimerFd` setup + its pollfd slot (`helloIdx`) + the `helloStateToPub` lambda + the initial hello status publish + the hot-reconcile `hello_->setMtuFps/setVanilla` block + the hello-timer fire branch + the `gsTunnelFd` setup/close (the GS-tunnel socket existed only to send HELLO) + the `helloTimerFd`/`gsTunnelFd` cleanup. In the receive switch, delete the `if (kind == PacketKind::HelloAck && hello_) { ... } else if (kind == PacketKind::Decision)` HelloAck arm so it becomes just `if (kind == PacketKind::Decision)`. Remove `status_.hello = ...` writes and the `HelloPub`/`helloStateToPub` usage. (The `DlStatus.hello` field + `HelloPub` enum in `runtime_config.hpp` can stay or be removed — if removing is clean, drop them; otherwise leave the field unset. Prefer removing `HelloPub hello{...}` from `DlStatus` + the enum if nothing else reads it — grep `status_.hello` / `.hello` first.)

- [ ] **Step 3: Daemon** — in `drone/src/daemon.cpp`, change the `dl_.start(buildDlSnapshot(effective_, radio_.iface), dlGenerationId_);` call to `dl_.start(buildDlSnapshot(effective_, radio_.iface));`. Remove the `dlGenerationId_(std::random_device{}())` initializer from the ctor init-list. In `drone/src/daemon.hpp`, remove the `uint32_t dlGenerationId_;` member. (Grep `dlGenerationId_` to confirm no other use.)

- [ ] **Step 4: Build + run** — `cd /home/gilankpam/Projects/drone/fpvd/drone && cmake --build build -j 2>&1 | tail -6`. Warnings about unused `hello.cpp` are fine (still compiled, removed in A4). Then update `drone/tests/integration/test_dl_controller.cpp`: remove the `generationId` argument from every `c.start(snap, ...)` call (→ `c.start(snap)`); delete the `"controller HELLO: sends DLHE…"` test case; remove any `snap.helloMtuBytes`/`snap.helloFps` assignments and `status().hello == HelloPub::...` assertions. Run `./build/fpvd_tests 2>&1 | tail -3` → green (the count drops by the removed HELLO test).

- [ ] **Step 5: Commit**
```bash
git add drone/src/dynlink/controller.cpp drone/src/dynlink/controller.hpp drone/src/daemon.cpp drone/src/daemon.hpp drone/tests/integration/test_dl_controller.cpp
git commit -m "feat(drone/dynlink): remove HELLO send/recv + generationId"
```

---

## Task A4: Delete `HelloSm` + the Hello/Ping/Pong wire + dead hello fields

**Files:** Delete `drone/src/dynlink/hello.hpp`, `hello.cpp`, `drone/tests/unit/test_dl_hello.cpp`; Modify `wire.hpp`, `wire.cpp`, `runtime_config.hpp`, `runtime_config.cpp`, `CMakeLists.txt`, `test_dl_wire.cpp`.

- [ ] **Step 1: Delete the files**
```bash
git rm drone/src/dynlink/hello.hpp drone/src/dynlink/hello.cpp drone/tests/unit/test_dl_hello.cpp
```

- [ ] **Step 2: Wire structs/funcs/constants** — in `drone/src/dynlink/wire.hpp`: delete the `Ping`, `Pong`, `Hello`, `HelloAck` structs; the `kPing*`, `kPong*`, `kHello*`, `kHelloAck*` constants and `kHelloFlagVanillaWfbNg`; the `encodePing/decodePing/encodePong/decodePong/encodeHello/decodeHello/encodeHelloAck/decodeHelloAck` declarations; and reduce `enum class PacketKind { Unknown, Decision, Ping, Pong, Hello, HelloAck };` to `enum class PacketKind { Unknown, Decision };`. In `drone/src/dynlink/wire.cpp`: delete the `encodePing/decodePing/encodePong/decodePong/encodeHello/decodeHello/encodeHelloAck/decodeHelloAck` definitions, and reduce `peekKind` to:
```cpp
PacketKind peekKind(const uint8_t* buf, size_t len) {
    if (len < 4) return PacketKind::Unknown;
    return get_u32(buf) == kWireMagic ? PacketKind::Decision : PacketKind::Unknown;
}
```

- [ ] **Step 3: Dead runtime fields** — in `drone/src/dynlink/runtime_config.hpp` remove `uint16_t helloMtuBytes;` and `uint16_t helloFps;` from `DlRuntimeConfig`. In `runtime_config.cpp` remove the `s.helloMtuBytes = …;` and `s.helloFps = …;` lines in `buildDlSnapshot`.

- [ ] **Step 4: CMake + wire tests** — in `drone/CMakeLists.txt`: remove `src/dynlink/hello.cpp` from `fpvd_core` `target_sources` and `tests/unit/test_dl_hello.cpp` from `fpvd_tests` `target_sources`. In `drone/tests/unit/test_dl_wire.cpp`: delete all `Ping`/`Pong`/`Hello`/`HelloAck` encode/decode test cases and the `peekKind` cases that referenced them (keep/adjust a `peekKind` test that asserts the Decision magic → `PacketKind::Decision` and a non-magic → `Unknown`). In `test_dl_runtime_config.cpp`: remove any `helloMtuBytes`/`helloFps` assertions.

- [ ] **Step 5: Build + run** — `cmake --build build -j 2>&1 | tail -6 && ./build/fpvd_tests 2>&1 | tail -3` → green, warning-clean. Grep to confirm nothing references the deleted symbols: `grep -rnE "HelloSm|encodeHello|decodeHello|encodePing|encodePong|kHelloMagic|helloMtuBytes|PacketKind::Hello|PacketKind::Ping" drone/src drone/tests` → no hits.

- [ ] **Step 6: Commit**
```bash
git add -A drone/
git commit -m "feat(drone/dynlink): delete HelloSm + Hello/Ping/Pong wire + dead hello fields"
```

---

# Part B — GS

## Task B1: Shrink the GS Decision wire to v3 + remove Hello/Ping/Pong from `wire.py`

**Files:** Modify `gs/fpvdgs/dynlink/wire.py`; Test `gs/tests/unit/test_dl_wire.py`, `gs/tests/unit/test_dl_wire_contract.py`.

The GS only ENCODES the Decision (and used to decode HELLO/PONG — now gone). Shrink `Encoder.encode`/`_encode_raw` to v3 and delete the HELLO/PING/PONG dataclasses, functions, and constants. The `Decision` dataclass still has the extra fields at this point (policy still sets them); `encode` simply stops reading them. decision.py + policy shrink in B2.

- [ ] **Step 1: Update the wire tests** — in `gs/tests/unit/test_dl_wire.py`, replace the Decision encode test(s) with v3:
```python
def test_v3_decision_encodes_15_bytes_mcs_only():
    from fpvdgs.dynlink.wire import Encoder, MAGIC, VERSION
    from fpvdgs.dynlink.decision import Decision
    import struct, binascii
    enc = Encoder(seq=7)
    d = Decision(timestamp=0.0, mcs=5, bandwidth=20, tx_power_dBm=20,
                 k=8, n=12, depth=1, bitrate_kbps=9999)
    raw = enc.encode(d, sequence=0xAABBCCDD)
    assert len(raw) == 15
    magic, ver, flags, seq, mcs = struct.unpack(">IBBIB", raw[:11])
    assert magic == MAGIC and ver == 3 and seq == 0xAABBCCDD and mcs == 5
    assert binascii.crc32(raw[:11]) & 0xFFFFFFFF == struct.unpack(">I", raw[11:15])[0]
```
In `gs/tests/unit/test_dl_wire_contract.py`: delete `test_hello_golden` and `test_hello_ack_golden`; update the `test_decision_golden`/`test_decision_*` golden byte expectations to the 15-byte v3 layout (recompute the golden bytes: `magic(>I=0x444C4B31) + version(B=3) + flags(B) + sequence(>I) + mcs(B)` then `crc32` of those 11 bytes as `>I`). If the golden is asserted as a hex literal, regenerate it for the chosen (sequence, mcs) — compute with the same `struct.pack(">IBBIB", 0x444C4B31, 3, flags, seq, mcs)` + crc.

- [ ] **Step 2: Run to verify it fails** — `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_wire.py::test_v3_decision_encodes_15_bytes_mcs_only -q` → FAIL (encode still 31 bytes).

- [ ] **Step 3: Implement v3 encode** — in `gs/fpvdgs/dynlink/wire.py`: set `VERSION = 3`, `PAYLOAD_SIZE = 11`, `ON_WIRE_SIZE = 15`. Rewrite `_encode_raw` (and `Encoder.encode`/the stateless `encode`) to pack only `{mcs}`:
```python
def _encode_raw(*, version, flags, sequence, mcs):
    payload = struct.pack(">IBBIB", MAGIC, version, flags, sequence, mcs)
    crc = _crc32(payload)
    return payload + struct.pack(">I", crc)
```
And `Encoder.encode`:
```python
    def encode(self, decision, *, timestamp_ms=None, sequence=None):
        if sequence is None:
            sequence = self.seq
            self.seq = (self.seq + 1) & 0xFFFFFFFF
        return _encode_raw(version=VERSION, flags=0,
                           sequence=sequence, mcs=int(decision.mcs))
```
(Drop `timestamp_ms` entirely — it's no longer on the wire. Keep the `timestamp_ms=None` kwarg for call-site compatibility but ignore it, OR remove it and update callers — grep `encode(` to check. Simplest: keep the kwarg, ignore it.)

- [ ] **Step 4: Delete HELLO/PING/PONG from `wire.py`** — remove the `Ping`, `Pong`, `Hello`, `HelloAck` dataclasses; the `encode_ping/decode_ping/encode_pong/decode_pong/encode_hello/decode_hello/encode_hello_ack/decode_hello_ack` functions; the `PING_*`, `PONG_*`, `HELLO_*`, `HELLO_ACK_*` constants and `HELLO_FLAG_VANILLA_WFB_NG`; and `peek_kind` (the GS no longer demuxes an ingress channel — confirm B3 removes its only caller, the tunnel listener). Keep `MAGIC`, `_crc32`, `Encoder`, `_encode_raw`, `encode`.

- [ ] **Step 5: Run to verify** — `.venv/bin/python -m pytest tests/unit/test_dl_wire.py tests/unit/test_dl_wire_contract.py -q` → PASS. Full suite will still RED on `policy.py`/`controller.py` imports of the removed `Hello`/`encode_hello_ack` — fixed in B2/B3. Run `.venv/bin/python -m pytest tests/ -q 2>&1 | tail -8` and confirm the only failures are import errors in policy/controller/their tests (the B2/B3 targets), not in wire.

- [ ] **Step 6: Commit (with B2/B3)** — the GS won't import cleanly until B2/B3 remove the `Hello`/`encode_hello_ack` references. **Defer the commit; proceed to B2.** (Mirrors the drone A2→A3 sequencing.)

---

## Task B2: Gut `Policy` to selector→`{mcs}` + shrink `decision.py`

**Files:** Modify `gs/fpvdgs/dynlink/policy.py`, `decision.py`; Test `gs/tests/unit/test_dl_policy_leading.py`; Delete `gs/tests/unit/test_dl_policy_trailing.py`, `gs/tests/unit/test_dl_policy_dynamic_fec_e2e.py`.

This is the core teardown. `Policy.tick()` keeps the Phase-2 selector (`LeadingSelector` + RSSI cold-start + reactive-demote signals) and emits a `{mcs}`-only Decision; everything that produced bitrate/k/n/depth/tx_power is removed, along with the sync-gate.

- [ ] **Step 1: Shrink the `Decision` dataclass** — in `gs/fpvdgs/dynlink/decision.py`, reduce to:
```python
@dataclass
class Decision:
    timestamp: float
    mcs: int
    reason: str = ""
    signals_snapshot: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)
```
(Drop `bandwidth`, `tx_power_dBm`, `k`, `n`, `depth`, `bitrate_kbps`, `knobs_changed`. Keep `reason` + `signals_snapshot` for telemetry/status.)

- [ ] **Step 2: Gut `policy.py`.** Delete (by name/block — line numbers from the map are hints, they drift):
  - The imports `from .bitrate import …`, `from .drone_config import …`, `from .dynamic_fec import …`, `from .predictor import …`.
  - Dataclasses `CooldownConfig`, `FECBounds`, `TrailingState`; the `TrailingLoop` class; `_ipi_ms_for_encoder`. Trim `SafeDefaults` to just `mcs`. Trim `PolicyConfig` to drop `cooldown`, `fec`, `bitrate`, `dynamic_fec`, `predictor`, `max_latency_ms`, `sustained_loss_windows`, `clean_windows_for_depth_stepdown` (keep what the selector needs: `safe`, `leading`, `gate`, `profile_selection`, `starvation_windows`, smoothing).
  - The `_safe_decision` method and the sync-gate guard in `tick()` (`if self.drone_config is not None and not self.drone_config.is_synced(): return self._safe_decision(...)`).
  - In `tick()`: the wire_target/`compute_k`/`compute_n`/`NEscalator`/`EmitGate`/`clamp_n_for_bitrate_floor` block, the `compute_bitrate_kbps` calls, the `trailing.tick(...)` call, the predictor/`fit_or_degrade` budget block, the mtu/fps-from-drone_config lines, and the `_compute_tx_power` usage for the emitted decision.
  - In `Policy.__init__`: drop the `drone_config` param + `self.drone_config`, and the init of `self.trailing`, `_n_escalator`, `_emit_gate`, `_tick_counter`, `_starvation_count` (keep the starvation hysteresis if the selector's `link_starved` demote still needs a sustained count — see below), and the bitrate/mtu init.

  KEEP: `LeadingLoopConfig` (trim deprecated fields if trivial, else leave), `GateConfig`, `ProfileSelectionConfig`, `LeadingState`, `LeadingSelector` (entire class), `coarse_mcs_for_rssi` + `_COLD_START_RSSI_DBM`, `PolicyState` (shrink to `mcs`), the `probe_status` plumbing, and the starvation hysteresis feeding `link_starved` into the selector's emergency demote.

- [ ] **Step 3: New `tick()` emit** — `Policy.tick()` should end by building the slim Decision from the selector output:
```python
        # Selector (Phase 2) is the only decision now: probe-promote + reactive demote.
        new_mcs, _tx, mcs_changed = self.leading.select(
            probe=self._probe_status() if self._probe_status else None,
            loss_rate=signals.residual_loss_w,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )
        self.state.mcs = new_mcs
        return Decision(
            timestamp=signals.timestamp,
            mcs=new_mcs,
            reason="; ".join(self.leading.reasons) if self.leading.reasons else "",
            signals_snapshot={
                "rssi": signals.rssi,
                "residual_loss": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "link_starved": sustained_starved,
                "mcs": new_mcs,
            },
        )
```
(Keep the RSSI cold-start seed block before `select(...)` exactly as Phase 2 left it. Adapt `sustained_starved`/`ts_ms`/`self.leading.reasons` to the real local names already in `tick()`. The `_tx`/`mcs_changed` are unused now — drop or keep as `_`.)

- [ ] **Step 4: Update/remove policy tests.**
  - `git rm gs/tests/unit/test_dl_policy_trailing.py gs/tests/unit/test_dl_policy_dynamic_fec_e2e.py` (both test deleted machinery).
  - In `gs/tests/unit/test_dl_policy_leading.py`: delete `test_policy_emits_safe_defaults_until_drone_synced` and `test_cold_start_seed_raises_mcs_on_first_synced_tick` (they use `DroneConfigState`/`Hello`/the sync-gate). Update `test_cold_start_seed_raises_mcs_on_first_synced_tick`'s intent into a no-sync-gate version if cold-start coverage is otherwise lost — e.g.:
```python
def test_cold_start_seed_raises_mcs_without_a_sync_gate():
    p = _make_policy()           # built WITHOUT drone_config now
    sig = _signals(rssi=-50)     # strong RSSI
    dec = p.tick(sig)
    assert dec.mcs >= 3          # seeded above the floor on the first tick
```
  (Adapt `_make_policy`/`_signals` to the file's real helpers; `_make_policy` must construct `Policy(...)` WITHOUT `drone_config`.) The pure `LeadingSelector` tests (most of the file) stay unchanged.

- [ ] **Step 5: Commit (with B1)** — now the GS imports clean. Run the suite first (see B3 — controller still imports `DroneConfigState`/`TunnelListener`; those are removed in B3). Defer the commit until B3 if `controller.py` still references removed symbols; otherwise, if you prefer, commit B1+B2 together now and B3 next. **Recommended: do B3, then commit B1+B2+B3 together** (the GS suite only goes green once the controller stops importing the removed modules).

---

## Task B3: Remove the GS HELLO listener + `drone_config` wiring

**Files:** Modify `gs/fpvdgs/dynlink/controller.py`, `return_link.py`.

- [ ] **Step 1: `controller.py`** — remove: the imports `from .drone_config import DroneConfigState`, `from .tunnel_listener import TunnelListener`, and `encode_hello_ack` from the `.wire` import; the `drone_cfg = DroneConfigState()` line; the `drone_config=drone_cfg` argument in the `Policy(build_policy_config(snap), profile, …)` construction; the `_on_hello` closure; the `TunnelListener(...)` construction + `await listener.start()` + the `listener.stop()` in the finally + the bind-failure `except`; the `gs_listen_addr`/`gs_listen_port` constructor params + `self._gs_listen`; and the `"hello"` key in the status dict + `self._set(hello=...)`. Keep `ReturnLink` (it sends decisions) and the `SignalAggregator → Policy → encode → ReturnLink` core + the probe wiring.

- [ ] **Step 2: `return_link.py`** — delete the `send_hello_ack` and `send_ping` methods. Keep `send` (the decision sender). Remove any now-unused imports (e.g. ping/hello-ack wire helpers).

- [ ] **Step 3: Run the GS suite** — `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/ -q 2>&1 | tail -10`. Remaining failures should only be: `test_dl_controller.py` (the HELLO/sync test `test_controller_forwards_probe_snapshot_to_policy` constructs the controller with a HELLO handshake) and `test_dl_imports.py` (MODULES list still includes deleted modules — but those modules still EXIST until B4, so imports pass; the controller test is the real one). Update `test_dl_controller.py`:
  - `test_controller_forwards_probe_snapshot_to_policy` drove the controller to SYNCED via a HELLO send to verify the probe reached `Policy.tick`. With the sync-gate gone, the policy ticks immediately — simplify the test to assert the probe callable is invoked WITHOUT the HELLO handshake (drop the `gs_listen_port`/HELLO-send setup; keep the repeating stats client that drives ticks; assert `seen["called"] >= 1`).
  - `test_emits_decision_packet_to_drone` (or similar): update the expected packet to the 15-byte v3 (decode via the wire `Encoder` or assert `len == 15` + `version == 3` + the mcs byte). Remove any HELLO-listener assertions.

- [ ] **Step 4: Run to verify** — `.venv/bin/python -m pytest tests/unit/test_dl_wire.py tests/unit/test_dl_policy_leading.py tests/unit/test_dl_controller.py -q` → PASS. The full suite still references the deleted modules only in `test_dl_imports.py` (handled in B4) and via the still-present module files — run `.venv/bin/python -m pytest tests/ -q 2>&1 | tail -6` and confirm green EXCEPT any test that imports a to-be-deleted module directly (those test files are deleted in B4).

- [ ] **Step 5: Commit (B1+B2+B3 together)**
```bash
git add gs/fpvdgs/dynlink/wire.py gs/fpvdgs/dynlink/decision.py gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/controller.py gs/fpvdgs/dynlink/return_link.py gs/tests/unit/
git commit -m "feat(gs/dynlink): v3 {mcs} wire; gut Policy to selector; remove HELLO + sync-gate"
```

---

## Task B4: Delete the dead GS modules + their tests

**Files:** Delete `gs/fpvdgs/dynlink/{drone_config,tunnel_listener,predictor,bitrate,dynamic_fec}.py` and `gs/tests/unit/test_dl_{bitrate,predictor,dynamic_fec,drone_config}.py`; Modify `gs/tests/unit/test_dl_imports.py`.

- [ ] **Step 1: Confirm no live importers remain** — `grep -rnE "from \.(drone_config|tunnel_listener|predictor|bitrate|dynamic_fec) import|import (drone_config|tunnel_listener|predictor|bitrate|dynamic_fec)" gs/fpvdgs/` → expected: NO hits in `fpvdgs/` (B2/B3 removed them). If any remain, fix them first.

- [ ] **Step 2: Delete**
```bash
git rm gs/fpvdgs/dynlink/drone_config.py gs/fpvdgs/dynlink/tunnel_listener.py gs/fpvdgs/dynlink/predictor.py gs/fpvdgs/dynlink/bitrate.py gs/fpvdgs/dynlink/dynamic_fec.py
git rm gs/tests/unit/test_dl_bitrate.py gs/tests/unit/test_dl_predictor.py gs/tests/unit/test_dl_dynamic_fec.py gs/tests/unit/test_dl_drone_config.py
```
(The `test_dl_policy_trailing.py`/`test_dl_policy_dynamic_fec_e2e.py` were already `git rm`'d in B2.)

- [ ] **Step 3: Update the imports guard** — in `gs/tests/unit/test_dl_imports.py`, remove `"predictor"`, `"dynamic_fec"`, `"bitrate"`, `"drone_config"`, `"tunnel_listener"` from the `MODULES` list. (Keep `return_link`, `wire`, `policy`, etc.)

- [ ] **Step 4: Run the full suite** — `.venv/bin/python -m pytest tests/ -q 2>&1 | tail -5` → green. `grep -rnE "drone_config|tunnel_listener|predictor|\bbitrate\b|dynamic_fec|NEscalator|TrailingLoop|fit_or_degrade|_safe_decision|is_synced|encode_hello|Hello\b" gs/fpvdgs/` → only legitimate residue (e.g. `bitrate` substrings in unrelated names) — no live references to the removed machinery.

- [ ] **Step 5: Commit**
```bash
git add -A gs/
git commit -m "feat(gs/dynlink): delete dead bitrate/FEC/predictor/drone-config/tunnel-listener modules"
```

---

## Task B5: Deprecate the retired GS config knobs

**Files:** Modify `gs/fpvdgs/dynlink/config_build.py`; Test `gs/tests/unit/test_dl_config_build.py`.

The bitrate/FEC/predictor knobs no longer exist on the GS (the drone owns them). A deployed `config.json` carrying them must still LOAD with a warning, matching the Phase-2 SNR-deprecation pattern.

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_dl_config_build.py`:
```python
def test_retired_bitrate_fec_knobs_parse_and_warn(caplog):
    from fpvdgs.dynlink.config_build import build_policy_config
    import logging
    with caplog.at_level(logging.WARNING):
        cfg = build_policy_config(_block({"tuning": {
            "policy": {"bitrate": {"utilization_factor": 0.7, "min_bitrate_kbps": 1000}},
            "fec": {"base_redundancy_ratio": 0.4, "max_n_escalation": 6},
            "video": {"per_packet_airtime_us": 80, "max_latency_ms": 100},
        }}))
    assert cfg is not None   # loads despite the retired knobs
    assert any("3a" in r.message or "drone" in r.message.lower()
               for r in caplog.records)
```
(Adapt `build_policy_config`/`_block` to the file's real entry point — match the existing config-build tests.)

- [ ] **Step 2: Run to verify it fails** — `.venv/bin/python -m pytest tests/unit/test_dl_config_build.py::test_retired_bitrate_fec_knobs_parse_and_warn -q` → FAIL (no warning, or build_policy_config still references the deleted `BitrateConfig`/`DynamicFecConfig`/`PredictorConfig`).

- [ ] **Step 3: Implement** — in `gs/fpvdgs/dynlink/config_build.py`: remove the imports + construction of `BitrateConfig`, `DynamicFecConfig`, `PredictorConfig`, `CooldownConfig`, `FECBounds` and the parsing of their knobs (the `policy.bitrate`, `fec.*`, `video.per_packet_airtime_us`/`max_latency_ms`, `cooldown.*`, `safe_video.k/n` blocks). Add a deprecation set + warn mirroring `_DEPRECATED_GATE_KEYS`:
```python
_DEPRECATED_PHASE3A_KEYS = {
    # bitrate/FEC/predictor knobs moved to the drone in Phase 3a.
    "utilization_factor", "min_bitrate_kbps", "max_bitrate_kbps",
    "base_redundancy_ratio", "max_redundancy_ratio", "blocks_per_frame",
    "depth_max", "n_loss_threshold", "n_loss_windows", "n_loss_step",
    "n_recover_windows", "n_recover_step", "max_n_escalation",
    "per_packet_airtime_us", "max_latency_ms",
}
```
and, after reading the relevant raw sub-dicts (`policy.bitrate`, `fec`, `video`), collect any present keys and warn once:
```python
    _retired = sorted(
        k for raw in (bitrate_raw, fec_raw, video_raw) for k in _DEPRECATED_PHASE3A_KEYS
        if k in (raw or {})
    )
    if _retired:
        log.warning("bitrate/FEC/predictor knobs are now drone-local (Phase 3a) and "
                    "ignored on the GS: %s", ", ".join(sorted(set(_retired))))
```
(Adapt `bitrate_raw`/`fec_raw`/`video_raw` to the real local names; `PolicyConfig` no longer takes the removed sub-configs — drop them from its construction. Keep the selector knobs: `gate.*`, `profile_selection.*`, `leading_loop` tx_power_min/max if the selector still reads them, `starvation_windows`, smoothing, `safe.mcs`.)

- [ ] **Step 4: Run to verify** — `.venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q` → PASS; full suite `.venv/bin/python -m pytest tests/ -q 2>&1 | tail -3` green (count lower than the 291 baseline by the deleted test files; that's expected). Update any existing config-build test that asserted on a now-removed config field (`cfg.bitrate`, `cfg.dynamic_fec`, `cfg.predictor`) — delete or repoint those assertions.

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_config_build.py
git commit -m "feat(gs/dynlink): deprecate retired bitrate/FEC/predictor config knobs"
```

---

# Part C — Coordinated deploy (flag-day)

## Task C1: On-hardware flag-day (needs live drone + GS)

**Files:** none (verification). **Deploy BOTH ends together** — v3 ⟷ v2 are incompatible, so a half-deploy drops to the drone's safe defaults (video survives) until both are v3.

- [ ] **Step 1:** Deploy the drone: `./deploy/drone/deploy.sh --host 192.168.10.152`. Then the GS: `./deploy/gs/deploy.sh --host 10.18.0.1`. (Order doesn't matter for correctness — a skewed pair falls to safe defaults — but deploy both promptly.)
- [ ] **Step 2:** Enable `dynamicLink` on both ends. Confirm the link runs: the GS emits a v3 `{mcs}` decision (no HELLO, no sync-gate — it ticks immediately), the drone applies its own bitrate/k/n (drone `/tmp/MSPOSD.msg` shows the drone-computed `(k,n)`/bitrate as in the Phase-3a smoke), video healthy across MCS changes, `waybeam`/`wfb_video_tx` PIDs unchanged (no runner bounce).
- [ ] **Step 3:** Verify the GS no longer has a HELLO listener and the wire is 15 bytes (e.g. the GS `/status` shows a decision; the drone receives + applies). Confirm a forced MCS change still drives the drone's recompute.
- [ ] **Step 4:** Skew test (optional): downgrade just one side to the prior v2 build (or disable one end), confirm the drone watchdog drops to safe defaults with **video intact**, then restore v3 — confirms the hard-flag-day fallback.
- [ ] **Step 5:** Disable `dynamicLink`; confirm clean teardown.

---

## Self-Review

**Spec coverage (`2026-06-07-phase3b-wire-shrink-hello-removal-design.md`):**
- §3 v3 wire `{mcs}` (15B, bandwidth from `link.width`) → A1 (linkBandwidth) + A2 (drone wire) + B1 (GS wire). ✓
- §4 HELLO + sync-gate removal → A3 (drone send/recv + generationId) + B2 (GS sync-gate) + B3 (GS listener). ✓
- §5 PING/PONG removal → A4 (drone) + B1 (GS). ✓
- §6 GS teardown (delete bitrate/predictor/dynamic_fec/drone_config/tunnel_listener; gut policy) → B2 + B4. ✓ `return_link.py` kept (decision send), only HelloAck/ping methods removed (B3). ✓ `signals.py` kept (selector inputs). ✓
- §7 drone teardown (hello.cpp, generationId, v3 decode) → A2/A3/A4. ✓
- §8 config deprecation → B5. ✓
- §9 flag-day → C1. ✓
- §10 testing (v3 round-trip + v2 rejection; policy emits {mcs} with no sync-gate; deleted-module tests removed; hardware) → A2/B1 (wire), B2 (policy), B4 (deletes), C1 (hardware). ✓

**Placeholder scan:** Deletion tasks specify targets by name + the map's line hints and end with "run suite green + grep for residue" — the verification is concrete (no failing-test-first applies to pure deletion). The "adapt to the real local names" notes in B2/B3/B5 are real seams (the implementer reads the current `tick()`/test helpers); the emitted Decision shape, the deprecation set, and the v3 byte layout are given in full. No TBDs.

**Type/name consistency:** v3 layout identical across A2 (C++ `>` put_u32 offsets) and B1 (Python `struct.pack(">IBBIB", …)`): magic/version=3/flags/sequence/mcs + crc, 11 payload + 4 CRC = 15. `linkBandwidth` (A1 producer) ↔ `d.bandwidth = cfg.linkBandwidth` (A1 consumer) ↔ wire-drops-bandwidth (A2). `Decision` slim shape (B2 decision.py) ↔ `Policy.tick` emit (B2) ↔ GS `Encoder.encode` reads only `decision.mcs` (B1). Deleted symbols (Hello/Ping/Pong/HelloSm/generationId/DroneConfigState/bitrate/dynamic_fec/predictor) each have a delete task AND a grep-for-residue check.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-07-phase3b-wire-shrink-hello-removal.md`.** Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task, spec + quality review between (Part A drone tasks, then Part B GS tasks; **STOP before C1** and hand the flag-day deploy to the human unless told to deploy).
2. **Inline Execution** — execute via executing-plans with checkpoints.

**Note:** B1 defers its commit; the GS suite only goes green after B3 (so B1+B2+B3 land as one commit). The drone half (A1–A4) and GS half (B1–B5) each keep their own suite green and could even be executed/reviewed as two sub-sequences. C1 is the coordinated flag-day deploy (do both ends together).
