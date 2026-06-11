# swfec Adoption Plan 1/3 — wfb-ng Fork (stats contract v3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the swfec fork (`~/Projects/poc/wfb-ng`, branch `swfec`) the deployable wfb-ng base by adding the GS stats contract (SESSION v3 fields, tolerant python parsing, swfec fec_type naming) it currently lacks.

**Architecture:** The C side (`rx.cpp`) becomes the single source of truth for the contract: every SESSION IPC line gains trailing `interleave_depth:contract_version` fields (`1:3`), emitted on-change AND once per stats window. The python layer (`wfb_ng/protocols.py`) parses 4–6 field SESSION lines tolerantly, dedups on-change notifications, names fec_type 2 `'swfec'`, and tolerates ≥11-field PKT lines. Spec: `fpvd/docs/superpowers/specs/2026-06-11-swfec-adoption-design.md`.

**Tech Stack:** C++ (Makefile build), Python 2/3-style Twisted (`twisted.trial` tests).

**Repo:** ALL work in this plan happens in `/home/gilankpam/Projects/poc/wfb-ng` on branch `swfec` (NOT the fpvd repo).

**Scope note:** the spec mentioned porting `wfb_ng/services.py` + `conf/master.cfg` plumbing from the interleav fork — inspection showed their entire diff there is interleaver-specific (`-X` flag injection, `interleave_depth` cfg keys), none of which we adopt. Only `protocols.py` needs changes; this plan reflects that.

---

### Task 1: Contract version constant + 6-field on-change SESSION lines

**Files:**
- Modify: `/home/gilankpam/Projects/poc/wfb-ng/src/wifibroadcast.hpp:191-194` (near the `WFB_FEC_*` defines)
- Modify: `/home/gilankpam/Projects/poc/wfb-ng/src/rx.cpp:715` (RS SESSION line) and `:744` (swfec SESSION line; line numbers approximate — locate by `grep -n 'SESSION' src/rx.cpp`)

- [ ] **Step 1: Add the contract constant**

In `src/wifibroadcast.hpp`, directly below the `#define WFB_FEC_SWFEC 0x2` line, add:

```c
// IPC stats contract version, emitted as SESSION trailing field #6.
// v3: WFB_FEC_SWFEC (2) exists; for swfec sessions the SESSION k/n slots
// carry overhead_pct/deadline_ms. Field #5 (interleave_depth) is always 1
// in this fork (no block interleaver). Bump on any stats-shape change —
// fpvdgs hard-fails on versions it doesn't know, by design.
#define WFB_IPC_CONTRACT_VERSION 3
```

- [ ] **Step 2: Extend the RS-session SESSION line (rx.cpp ~715)**

Replace:

```c
IPC_MSG("%" PRIu64 "\tSESSION\t%" PRIu64 ":%u:%d:%d\n", get_time_ms(), epoch, WFB_FEC_VDM_RS, fec_k, fec_n);
```

with:

```c
// Trailing fields #5 (interleave_depth, fixed 1 — no interleaver in this
// fork) and #6 (contract_version). 4-field-only parsers stay compatible.
IPC_MSG("%" PRIu64 "\tSESSION\t%" PRIu64 ":%u:%d:%d:%u:%u\n", get_time_ms(), epoch, WFB_FEC_VDM_RS, fec_k, fec_n,
        1u, (unsigned)WFB_IPC_CONTRACT_VERSION);
```

- [ ] **Step 3: Extend the swfec-session SESSION line (rx.cpp ~744) and persist k/n**

The swfec session-init path currently emits from `new_session_data` directly and never stores the values, but Task 2's periodic re-emission needs them. Replace:

```c
IPC_MSG("%" PRIu64 "\tSESSION\t%" PRIu64 ":%u:%d:%d\n", get_time_ms(), epoch, WFB_FEC_SWFEC, (int)new_session_data->k, (int)new_session_data->n);
```

with:

```c
fec_k = new_session_data->k;   // swfec: overhead_pct rides the k slot
fec_n = new_session_data->n;   // swfec: deadline_ms rides the n slot
IPC_MSG("%" PRIu64 "\tSESSION\t%" PRIu64 ":%u:%d:%d:%u:%u\n", get_time_ms(), epoch, WFB_FEC_SWFEC, fec_k, fec_n,
        1u, (unsigned)WFB_IPC_CONTRACT_VERSION);
```

(`fec_k`/`fec_n` are existing `Aggregator` members, `src/rx.hpp:234-235`.)

- [ ] **Step 4: Build**

Run: `cd /home/gilankpam/Projects/poc/wfb-ng && make wfb_rx`
Expected: clean compile, no warnings about format strings (6 args added for `%u:%u`).

- [ ] **Step 5: Commit**

```bash
git add src/wifibroadcast.hpp src/rx.cpp
git commit -m "swfec: SESSION contract v3 — trailing interleave_depth/contract_version fields"
```

---

### Task 2: Periodic SESSION re-emission in dump_stats

**Files:**
- Modify: `/home/gilankpam/Projects/poc/wfb-ng/src/rx.cpp` — `Aggregator::dump_stats` (locate via `grep -n 'void Aggregator::dump_stats' src/rx.cpp`; the swfec loss-accounting block added on this branch sits just before the `PKT` `IPC_MSG`)

- [ ] **Step 1: Emit SESSION once per stats window**

In `Aggregator::dump_stats`, immediately BEFORE the `IPC_MSG("%" PRIu64 "\tPKT\t..."` line (and after the existing swfec loss-accounting block), add:

```c
// Contract v3: re-emit SESSION once per stats window so a late-attached
// python parser learns the session without waiting for an on-change event.
// The python side dedups, so aggregators only see real changes.
if (fec_p != NULL || swfec_dec != NULL)
{
    IPC_MSG("%" PRIu64 "\tSESSION\t%" PRIu64 ":%u:%d:%d:%u:%u\n", ts, epoch,
            session_is_swfec ? WFB_FEC_SWFEC : WFB_FEC_VDM_RS, fec_k, fec_n,
            1u, (unsigned)WFB_IPC_CONTRACT_VERSION);
}
```

(`ts`, `epoch`, `fec_p`, `swfec_dec`, `session_is_swfec` are all in scope in `dump_stats`; `fec_p` is non-NULL only for an active RS session, `swfec_dec` only for an active swfec session.)

- [ ] **Step 2: Build**

Run: `make wfb_rx`
Expected: clean compile.

- [ ] **Step 3: Run the existing C test suites (regression)**

Run: `make fec_swfec_test && ./fec_swfec_test`
Expected: all swfec differential/fuzz tests PASS (byte-exact vectors unaffected — we touched only stats emission).

- [ ] **Step 4: Commit**

```bash
git add src/rx.cpp
git commit -m "swfec: re-emit SESSION once per stats window (contract v3)"
```

---

### Task 3: Python — tolerant SESSION/PKT parsing, swfec naming, on-change dedup

**Files:**
- Modify: `/home/gilankpam/Projects/poc/wfb-ng/wfb_ng/protocols.py:45` (`fec_types`), `:409-414` (PKT `k_tuple` block), `:429-438` (SESSION block)
- Create: `/home/gilankpam/Projects/poc/wfb-ng/wfb_ng/tests/test_session_contract.py`

- [ ] **Step 1: Write the failing tests**

Create `wfb_ng/tests/test_session_contract.py`:

```python
# Contract-v3 parsing tests for RXAntennaProtocol: tolerant SESSION
# (4..6 fields), swfec fec_type naming, on-change dedup, tolerant PKT.
from unittest.mock import MagicMock

from twisted.trial import unittest

from wfb_ng.protocols import RXAntennaProtocol, BadTelemetry


def make_proto():
    cb = MagicMock()
    p = RXAntennaProtocol(cb, 'video rx')
    return p, cb


class SessionContractTests(unittest.TestCase):
    def test_six_field_session_parsed(self):
        p, cb = make_proto()
        p.lineReceived(b'100\tSESSION\t7:2:50:30:1:3')
        cb.process_new_session.assert_called_once_with('video rx', dict(
            fec_type='swfec', fec_k=50, fec_n=30, epoch=7,
            interleave_depth=1, contract_version=3))

    def test_four_field_session_defaults(self):
        p, cb = make_proto()
        p.lineReceived(b'100\tSESSION\t7:1:8:12')
        cb.process_new_session.assert_called_once_with('video rx', dict(
            fec_type='VDM_RS', fec_k=8, fec_n=12, epoch=7,
            interleave_depth=1, contract_version=1))

    def test_session_reemission_deduped(self):
        p, cb = make_proto()
        p.lineReceived(b'100\tSESSION\t7:2:50:30:1:3')
        p.lineReceived(b'200\tSESSION\t7:2:50:30:1:3')   # periodic re-emit
        self.assertEqual(cb.process_new_session.call_count, 1)

    def test_session_change_notifies_again(self):
        p, cb = make_proto()
        p.lineReceived(b'100\tSESSION\t7:2:50:30:1:3')
        p.lineReceived(b'200\tSESSION\t7:2:80:30:1:3')   # overhead changed
        self.assertEqual(cb.process_new_session.call_count, 2)

    def test_short_session_rejected(self):
        p, cb = make_proto()
        self.assertRaises(BadTelemetry,
                          p.lineReceived, b'100\tSESSION\t7:1:8')

    def test_pkt_eleven_fields_ok(self):
        p, cb = make_proto()
        p.lineReceived(b'100\tPKT\t1:2:3:4:5:6:7:8:9:10:11')  # must not raise

    def test_pkt_extra_fields_tolerated(self):
        p, cb = make_proto()
        p.lineReceived(b'100\tPKT\t1:2:3:4:5:6:7:8:9:10:11:12:13:14')

    def test_pkt_short_rejected(self):
        p, cb = make_proto()
        self.assertRaises(BadTelemetry,
                          p.lineReceived, b'100\tPKT\t1:2:3:4:5:6:7:8:9:10')
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/gilankpam/Projects/poc/wfb-ng && PYTHONPATH=$(pwd) python3 -m twisted.trial wfb_ng.tests.test_session_contract`
Expected: FAIL — `test_six_field_session_parsed` errors (`too many values to unpack` from the strict 4-field parse), `test_pkt_extra_fields_tolerated` errors on the `assert len(counters) == len(k_tuple)`.

- [ ] **Step 3: Implement the python changes**

In `wfb_ng/protocols.py`:

(a) Line 45 — extend the FEC name map:

```python
fec_types = {1: 'VDM_RS', 2: 'swfec'}
```

(b) PKT block (~lines 409-414) — replace:

```python
                k_tuple = ('all', 'all_bytes', 'dec_err', 'session', 'data', 'uniq', 'fec_rec', 'lost', 'bad', 'out', 'out_bytes')
                counters = tuple(int(i) for i in cols[2].split(':'))
                assert len(counters) == len(k_tuple)
```

with:

```python
                k_tuple = ('all', 'all_bytes', 'dec_err', 'session', 'data', 'uniq', 'fec_rec', 'lost', 'bad', 'out', 'out_bytes')
                counters = tuple(int(i) for i in cols[2].split(':'))
                # Tolerate newer wfb_rx emitting extra trailing counters;
                # reject lines shorter than the stock 11-field layout.
                if len(counters) < len(k_tuple):
                    raise BadTelemetry()
                counters = counters[:len(k_tuple)]
```

(c) SESSION block (~lines 429-438) — replace:

```python
            elif cmd == 'SESSION':
                if len(cols) != 3:
                    raise BadTelemetry()

                epoch, fec_type, fec_k, fec_n = list(int(i) for i in cols[2].split(':'))
                self.session = dict(fec_type=fec_types.get(fec_type, 'Unknown'), fec_k=fec_k, fec_n=fec_n, epoch=epoch)
                log.msg('New session detected [%s]: FEC=%s K=%d, N=%d, epoch=%d' % (self.rx_id, fec_types.get(fec_type, 'Unknown'), fec_k, fec_n, epoch))

                if self.ant_stat_cb is not None:
                    self.ant_stat_cb.process_new_session(self.rx_id, self.session)
```

with:

```python
            elif cmd == 'SESSION':
                if len(cols) != 3:
                    raise BadTelemetry()

                # Contract v3 emits 6 fields (epoch:fec_type:k:n:
                # interleave_depth:contract_version); stock emitted 4.
                # Accept either; missing trailing fields default.
                # For fec_type 'swfec', k/n carry overhead_pct/deadline_ms.
                parts = list(int(i) for i in cols[2].split(':'))
                if len(parts) < 4:
                    raise BadTelemetry()
                epoch, fec_type, fec_k, fec_n = parts[:4]
                interleave_depth = parts[4] if len(parts) > 4 else 1
                contract_version = parts[5] if len(parts) > 5 else 1

                new_session = dict(fec_type=fec_types.get(fec_type, 'Unknown'),
                                   fec_k=fec_k, fec_n=fec_n, epoch=epoch,
                                   interleave_depth=interleave_depth,
                                   contract_version=contract_version)

                # SESSION arrives on-change AND once per stats window;
                # only log + notify aggregators on a real change.
                if new_session != self.session:
                    self.session = new_session
                    log.msg('New session detected [%s]: FEC=%s K=%d, N=%d, epoch=%d' % (self.rx_id, new_session['fec_type'], fec_k, fec_n, epoch))

                    if self.ant_stat_cb is not None:
                        self.ant_stat_cb.process_new_session(self.rx_id, self.session)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=$(pwd) python3 -m twisted.trial wfb_ng.tests.test_session_contract`
Expected: 8 tests PASS.

- [ ] **Step 5: Run the full python suite (regression)**

Run: `PYTHONPATH=$(pwd) python3 -m twisted.trial wfb_ng.tests`
Expected: all PASS (test_proxy, test_tuntap, test_twisted, test_txrx untouched).

- [ ] **Step 6: Commit**

```bash
git add wfb_ng/protocols.py wfb_ng/tests/test_session_contract.py
git commit -m "swfec: python stats contract v3 — tolerant SESSION/PKT parse, swfec naming, on-change dedup"
```

---

### Task 4: swfec input-size headroom guard

**Files:**
- Modify: `/home/gilankpam/Projects/poc/wfb-ng/src/tx.hpp:36-38` (the `SWFEC_MAX_INPUT` define)

- [ ] **Step 1: Add a compile-time guard**

Directly below the `#define SWFEC_MAX_INPUT (MAX_FEC_PAYLOAD - 12 - 2)` line in `src/tx.hpp`, add:

```c
// fpvd's waybeam venc emits RTP datagrams of <= 1400 B payload + 12 B RTP
// header; oversize inputs are silently dropped in swfec mode, so pin the
// headroom at compile time. See fpvd spec 2026-06-11-swfec-adoption-design.
static_assert(SWFEC_MAX_INPUT >= 1412, "swfec input headroom regression");
```

- [ ] **Step 2: Build everything that includes tx.hpp**

Run: `make wfb_tx`
Expected: clean compile (assert holds). If it FAILS, the spec's size assumption is wrong — STOP and report; do not weaken the assert.

- [ ] **Step 3: Commit**

```bash
git add src/tx.hpp
git commit -m "swfec: compile-time input headroom guard for fpvd's 1412B venc datagrams"
```

---

### Task 5: Full-suite verification

- [ ] **Step 1: Run the complete fork test target**

Run: `cd /home/gilankpam/Projects/poc/wfb-ng && make test`
Expected: `fec_test`, `libsodium_test`, `fec_swfec_test`, and `twisted.trial wfb_ng.tests` all PASS.

- [ ] **Step 2: Manual smoke (optional but recommended before deploy)**

Build both ends locally and loop a UDP stream through `wfb_tx -z -k 50 -n 30` → `wfb_rx` on loopback-injected pcap or the bench radios; confirm the `:8103`-style JSON (via `wfb-cli` or raw socket) shows `"fec_type": "swfec"`, `"fec_k": 50`, `"fec_n": 30`, `"contract_version": 3`.

- [ ] **Step 3: Push the branch**

```bash
git push origin swfec
```
