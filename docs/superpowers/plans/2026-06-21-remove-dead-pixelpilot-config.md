# Remove Dead PixelPilot Config Knobs — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove nine GS `pixelpilot` config knobs whose pixelpilot CLI args no longer exist, so the renderer stops emitting flags the current pixelpilot binary rejects.

**Architecture:** The pixelpilot config flows through three GS modules — `config_defaults.py` (default tree), `schema.py` (`_validate_pixelpilot`), and `pixelpilot.py` (`render_pixelpilot_argv`, which builds the child argv). Removal touches each plus its tests, then the two live docs. A deployed `config.json` still carrying old keys is safely warned-and-stripped by the existing tolerant loader — no migration code.

**Tech Stack:** Python ≥3.11, pytest. GS-only (no drone changes).

## Global Constraints

- Scope is GS (`gs/fpvdgs`) only — no drone-side changes.
- The full GS pytest suite must stay green **as a whole** at every commit: `cd gs && .venv/bin/python -m pytest tests/ -q`.
- Do NOT edit `gs/build/lib/**` (build artifacts) or historical docs under `docs/superpowers/specs|plans/` other than this plan.
- Fields removed (config key → dead CLI arg): `videoScale`→`--video-scale`, `rtpJitterMs`→`--rtp-jitter-ms`, `dvr.framerate`→`--dvr-framerate`, `dvr.mode`→`--dvr-mode`, `dvr.maxSizeMb`→`--dvr-max-size`, `dvr.reencCodec`→`--dvr-reenc-codec`, `dvr.reencBitrate`→`--dvr-reenc-bitrate`, `dvr.reencFps`→`--dvr-reenc-fps`, `dvr.reencResolution`→`--dvr-reenc-resolution`.
- Fields RETAINED: `pixelpilot.{bin,env,configPath,osdConfigPath,screenMode,codec,rtpPort,extraArgs,enabled}` and `pixelpilot.dvr.{dir,template,fmp4,sequencedFiles,osd}`.

---

### Task 1: Renderer — stop emitting dead flags

**Files:**
- Modify: `gs/fpvdgs/pixelpilot.py` (`render_pixelpilot_argv`, lines 13-48)
- Test: `gs/tests/unit/test_pixelpilot_render.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: `render_pixelpilot_argv(effective: dict) -> list[str]` — argv no longer contains `--video-scale`, `--rtp-jitter-ms`, `--dvr-framerate`, `--dvr-mode`, `--dvr-max-size`, `--dvr-reenc-codec`, `--dvr-reenc-bitrate`, `--dvr-reenc-fps`, `--dvr-reenc-resolution`. Retained flags unchanged: `--config --osd --osd-custom-message --osd-config --codec --screen-mode [--dvr-fmp4] [--dvr-sequenced-files] --dvr-template [--dvr-osd] -p` plus `extraArgs`.

- [ ] **Step 1: Update the tests to expect the new argv**

Replace the `DEFAULTS` dict and the `test_defaults_render_full_argset` / `test_knobs_reflected` tests in `gs/tests/unit/test_pixelpilot_render.py`, and add a dead-flag guard test. The `DEFAULTS` dict becomes:

```python
DEFAULTS = {"pixelpilot": {
    "bin": "/usr/bin/pixelpilot",
    "configPath": "/etc/pixelpilot.yaml",
    "osdConfigPath": "/etc/pixelpilot/osd.json",
    "screenMode": "1920x1080@60",
    "codec": "h265",
    "rtpPort": 5600,
    "dvr": {"dir": "/media/dvr",
            "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
            "fmp4": True, "sequencedFiles": True, "osd": False},
    "extraArgs": [],
}}
```

`test_defaults_render_full_argset` becomes:

```python
def test_defaults_render_full_argset():
    assert render_pixelpilot_argv(DEFAULTS) == [
        "/usr/bin/pixelpilot",
        "--config", "/etc/pixelpilot.yaml",
        "--osd", "--osd-custom-message",
        "--osd-config", "/etc/pixelpilot/osd.json",
        "--codec", "h265",
        "--screen-mode", "1920x1080@60",
        "--dvr-fmp4", "--dvr-sequenced-files",
        "--dvr-template", "/media/dvr/record_%Y-%m-%d_%H-%M-%S.mp4",
        "-p", "5600",
    ]
```

`test_knobs_reflected` becomes (only retained knobs):

```python
def test_knobs_reflected():
    cfg = {"pixelpilot": {"rtpPort": 5602, "codec": "h264",
                          "screenMode": "1280x720@60"}}
    argv = render_pixelpilot_argv(cfg)
    assert argv[argv.index("-p") + 1] == "5602"
    assert argv[argv.index("--codec") + 1] == "h264"
    assert argv[argv.index("--screen-mode") + 1] == "1280x720@60"
```

Add this new test:

```python
def test_dead_flags_not_emitted():
    argv = render_pixelpilot_argv(DEFAULTS)
    for dead in ("--video-scale", "--rtp-jitter-ms", "--dvr-framerate",
                 "--dvr-mode", "--dvr-max-size", "--dvr-reenc-codec",
                 "--dvr-reenc-bitrate", "--dvr-reenc-fps",
                 "--dvr-reenc-resolution"):
        assert dead not in argv
```

Leave `test_dvr_osd_flag_toggles`, `test_fmp4_and_sequenced_default_on_can_disable`, `test_extra_args_appended`, `test_missing_block_uses_defaults`, and the env tests unchanged.

- [ ] **Step 2: Run the render tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_pixelpilot_render.py -q`
Expected: FAIL — `test_defaults_render_full_argset`, `test_knobs_reflected`, and `test_dead_flags_not_emitted` fail because the renderer still emits the dead flags.

- [ ] **Step 3: Remove the dead flags from the renderer**

Replace `render_pixelpilot_argv` in `gs/fpvdgs/pixelpilot.py` with:

```python
def render_pixelpilot_argv(effective: dict) -> list[str]:
    pp = effective.get("pixelpilot", {})
    dvr = pp.get("dvr", {})
    dvr_dir = dvr.get("dir", "/media/dvr")
    dvr_template = dvr.get("template", "record_%Y-%m-%d_%H-%M-%S.mp4")
    argv = [
        pp.get("bin", "/usr/bin/pixelpilot"),
        "--config", pp.get("configPath", "/etc/pixelpilot.yaml"),
        "--osd", "--osd-custom-message",
        "--osd-config", pp.get("osdConfigPath", "/etc/pixelpilot/osd.json"),
        "--codec", pp.get("codec", "h265"),
        "--screen-mode", pp.get("screenMode", "1920x1080@60"),
    ]
    if dvr.get("fmp4", True):
        argv.append("--dvr-fmp4")
    if dvr.get("sequencedFiles", True):
        argv.append("--dvr-sequenced-files")
    argv += [
        "--dvr-template", os.path.join(dvr_dir, dvr_template),
    ]
    if dvr.get("osd", False):
        argv.append("--dvr-osd")
    argv += [
        "-p", str(pp.get("rtpPort", 5600)),
    ]
    argv += pp.get("extraArgs", [])
    return argv
```

Leave `render_pixelpilot_env` unchanged.

- [ ] **Step 4: Run the render tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_pixelpilot_render.py -q`
Expected: PASS (all tests in the file).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/pixelpilot.py gs/tests/unit/test_pixelpilot_render.py
git commit -m "gs(pixelpilot): stop emitting removed pixelpilot CLI flags"
```

---

### Task 2: Defaults + API fixtures — drop dead keys

**Files:**
- Modify: `gs/fpvdgs/config_defaults.py` (pixelpilot block, lines 80-94)
- Test: `gs/tests/unit/test_api.py` (`_api_with_pp` defaults, lines 203-205; `test_patch_config_accepts_pixelpilot`, lines 258-263)

**Interfaces:**
- Consumes: renderer from Task 1 (dead flags already gone).
- Produces: default `pixelpilot` tree without the nine removed keys; otherwise identical shape.

- [ ] **Step 1: Update the API test fixtures to drop dead keys**

In `gs/tests/unit/test_api.py`, change the `pixelpilot` default in `_api_with_pp` (currently lines 203-205) to:

```python
                "pixelpilot": {"enabled": True, "screenMode": "1920x1080@60",
                               "extraArgs": []}}
```

And change `test_patch_config_accepts_pixelpilot` (lines 258-263) to patch a retained field instead of `videoScale`:

```python
def test_patch_config_accepts_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    code, _ = api.handle("PATCH", "/gs/config", {},
                         json.dumps({"pixelpilot": {"screenMode": "1280x720@60"}}).encode())
    assert code == 200
    assert store.pending()["pixelpilot"]["screenMode"] == "1280x720@60"
```

- [ ] **Step 2: Run the API tests (they should still pass — fixtures are self-consistent)**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_api.py -q`
Expected: PASS. (These edits remove references to dead keys; the tolerant schema already ignores unknowns, so the suite stays green here regardless of the defaults edit.)

- [ ] **Step 3: Remove the dead keys from the default tree**

Replace the `pixelpilot` block in `gs/fpvdgs/config_defaults.py` (lines 80-94) with:

```python
        "pixelpilot": {
            "enabled": True, "bin": "/usr/bin/pixelpilot", "env": {},
            "configPath": "/etc/pixelpilot.yaml",
            "osdConfigPath": "/etc/pixelpilot/osd.json",
            "screenMode": "1920x1080@60", "codec": "h265",
            "rtpPort": 5600,
            "dvr": {
                "dir": "/media/dvr",
                "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
                "fmp4": True, "sequencedFiles": True, "osd": False,
            },
            "extraArgs": [],
        },
```

- [ ] **Step 4: Run the full GS suite to verify the default tree still validates**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS (whole suite). This confirms the trimmed default tree still passes `validate_effective` wherever the real defaults are exercised.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/config_defaults.py gs/tests/unit/test_api.py
git commit -m "gs(pixelpilot): drop removed knobs from default config tree"
```

---

### Task 3: Schema — remove validators for dead fields

**Files:**
- Modify: `gs/fpvdgs/schema.py` (`_validate_pixelpilot`, lines 229-265)
- Test: `gs/tests/unit/test_schema.py` (`test_validate_effective_accepts_pixelpilot_block`, lines 91-100; `test_validate_effective_rejects_bad_pixelpilot`, lines 103-122)

**Interfaces:**
- Consumes: nothing new.
- Produces: `_validate_pixelpilot(pp: dict) -> None` that no longer validates `videoScale`, `rtpJitterMs`, `dvr.framerate`, `dvr.maxSizeMb`, `dvr.reencBitrate`, `dvr.reencFps`, `dvr.mode`, `dvr.reencCodec`, `dvr.reencResolution`. Still validates `enabled`, `rtpPort`, the string keys `{screenMode,bin,configPath,osdConfigPath,codec}`, `dvr.{dir,template}`, `dvr.{fmp4,sequencedFiles,osd}`, `env`, `extraArgs`.

- [ ] **Step 1: Update the schema tests for the trimmed validator**

In `gs/tests/unit/test_schema.py`, replace `test_validate_effective_accepts_pixelpilot_block` (lines 91-100) with:

```python
def test_validate_effective_accepts_pixelpilot_block():
    cfg = {"link": {"channel": 132, "width": 40, "region": "US"},
           "pixelpilot": {"enabled": True,
                          "screenMode": "1920x1080@60",
                          "rtpPort": 5600,
                          "codec": "h265",
                          "env": {},
                          "dvr": {"dir": "/media/dvr"},
                          "extraArgs": []}}
    schema.validate_effective(cfg)  # no raise
```

And replace the `for bad in (...)` tuple in `test_validate_effective_rejects_bad_pixelpilot` (lines 105-122) with this tuple (drops the removed-field cases, adds a retained dvr-string case):

```python
    for bad in (
        {"enabled": "yes"},
        {"screenMode": ""},
        {"extraArgs": "not-a-list"},
        {"extraArgs": [1, 2]},
        {"rtpPort": 0},
        {"rtpPort": 70000},
        {"codec": ""},
        {"dvr": {"dir": ""}},
        {"dvr": {"osd": "yes"}},
        {"env": {"A": 1}},
        {"env": "x"},
    ):
```

- [ ] **Step 2: Run the schema tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -q`
Expected: FAIL — `test_validate_effective_rejects_bad_pixelpilot` fails on the new `{"dvr": {"dir": ""}}` case only if the validator already rejects it (it does, so this case passes), but the test as a whole still passes here; the real failing signal is none yet. If it PASSES already, that is fine — proceed to Step 3 (the validator removal is a no-op for behavior since removed validators only ever rejected the now-dropped cases). The trimmed accept-block test must also still pass.

> Note: this task removes validators, so the test edit mostly removes assertions rather than adding failing ones. Treat Step 2 as a baseline run; the binding check is Step 4 (full suite green after the validator is trimmed).

- [ ] **Step 3: Trim the validator**

Replace `_validate_pixelpilot` in `gs/fpvdgs/schema.py` (lines 229-265) with:

```python
def _validate_pixelpilot(pp: dict) -> None:
    if not isinstance(pp.get("enabled", True), bool):
        raise SchemaError("pixelpilot.enabled must be a bool")
    port = pp.get("rtpPort", 5600)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("pixelpilot.rtpPort must be an int in 1..65535")
    for key in ("screenMode", "bin", "configPath", "osdConfigPath", "codec"):
        val = pp.get(key)
        if val is not None and (not isinstance(val, str) or not val):
            raise SchemaError(f"pixelpilot.{key} must be a non-empty string")
    dvr = pp.get("dvr", {})
    if not isinstance(dvr, dict):
        raise SchemaError("pixelpilot.dvr must be an object")
    for key in ("dir", "template"):
        val = dvr.get(key)
        if val is not None and (not isinstance(val, str) or not val):
            raise SchemaError(f"pixelpilot.dvr.{key} must be a non-empty string")
    for key in ("fmp4", "sequencedFiles", "osd"):
        if key in dvr and not isinstance(dvr[key], bool):
            raise SchemaError(f"pixelpilot.dvr.{key} must be a bool")
    env = pp.get("env", {})
    if not isinstance(env, dict) or not all(
            isinstance(k, str) and isinstance(v, str) for k, v in env.items()):
        raise SchemaError("pixelpilot.env must be a map of string to string")
    extra = pp.get("extraArgs", [])
    if not isinstance(extra, list) or not all(isinstance(a, str) for a in extra):
        raise SchemaError("pixelpilot.extraArgs must be a list of strings")
```

- [ ] **Step 4: Run the schema tests + full suite to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -q && cd gs 2>/dev/null; .venv/bin/python -m pytest tests/ -q`
Expected: PASS (schema file and whole suite).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/schema.py gs/tests/unit/test_schema.py
git commit -m "gs(pixelpilot): remove schema validation for dropped knobs"
```

---

### Task 4: Docs — update live references

**Files:**
- Modify: `docs/api.md` (pixelpilot JSON example + key table + curl example, lines 1110-1169)
- Modify: `gs/README.md` (pixelpilot JSON example + key table, lines 175-226; verification curl, line 242)

**Interfaces:** none (documentation only).

- [ ] **Step 1: Update `docs/api.md`**

In the pixelpilot JSON example, remove the `videoScale`, `rtpJitterMs` top-level lines and the `framerate`, `mode`, `maxSizeMb`, `reencCodec`, `reencBitrate`, `reencFps`, `reencResolution` lines inside `dvr`, so the block reads:

```json
  "enabled": true,
  "bin": "/usr/bin/pixelpilot",
  "env": {},
  "configPath": "/etc/pixelpilot.yaml",
  "osdConfigPath": "/etc/pixelpilot/osd.json",
  "screenMode": "1920x1080@60",
  "codec": "h265",
  "rtpPort": 5600,
  "dvr": {
    "dir": "/media/dvr",
    "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
    "fmp4": true,
    "sequencedFiles": true,
    "osd": false
  },
  "extraArgs": []
}
```

Delete these rows from the key table (lines 1146, 1149-1150, 1156-1161): `videoScale`, `rtpJitterMs`, `dvr.framerate`, `dvr.mode`, `dvr.maxSizeMb`, `dvr.reencCodec`, `dvr.reencBitrate`, `dvr.reencFps`, `dvr.reencResolution`. Keep `dvr.dir`, `dvr.template`, `dvr.fmp4`, `dvr.sequencedFiles`, `dvr.osd`, `extraArgs`.

Update the curl example (lines 1165-1168) to use a retained field:

```bash
# Change display mode and apply (restarts PixelPilot only):
curl -X PATCH http://10.18.0.1:8080/gs/config \
  -H 'content-type: application/json' \
  -d '{"pixelpilot":{"screenMode":"1280x720@60"}}'
```

- [ ] **Step 2: Update `gs/README.md`**

Apply the identical JSON-example trim (lines 175-200) so the `dvr` block contains only `dir`, `template`, `fmp4`, `sequencedFiles`, `osd` and the top level drops `videoScale`/`rtpJitterMs`. Delete the same nine rows from the key table (lines 211, 214-215, 221-226). In the verification step (line 242), change the curl payload from `{"pixelpilot":{"videoScale":1.5}}` to `{"pixelpilot":{"screenMode":"1280x720@60"}}`.

- [ ] **Step 3: Verify no stale references remain in live docs**

Run: `grep -rn "videoScale\|rtpJitterMs\|dvr-framerate\|dvr-mode\|dvr-reenc\|maxSizeMb\|reencCodec\|reencBitrate\|reencFps\|reencResolution\|framerate" docs/api.md gs/README.md`
Expected: no output (zero matches).

- [ ] **Step 4: Run the full GS suite once more (docs change nothing, sanity check)**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS (whole suite).

- [ ] **Step 5: Commit**

```bash
git add docs/api.md gs/README.md
git commit -m "docs(pixelpilot): drop removed config knobs from api.md + README"
```

---

## Self-Review

**Spec coverage:** All nine removed fields are dropped from defaults (Task 2), schema (Task 3), and renderer (Task 1); tests updated in Tasks 1-3; docs updated in Task 4 (api.md + README only, historical specs untouched per constraint). Retained-field list matches the spec. Migration/safety (tolerant loader) requires no code, as the spec states. ✓

**Placeholder scan:** No TBD/TODO; every code step shows full replacement code; every test step shows the exact assertions. ✓ (Task 3 Step 2 is intentionally a baseline run — explained inline — because removing validators removes assertions rather than adding a failing one; the binding gate is the full-suite green in Step 4.)

**Type consistency:** `render_pixelpilot_argv`/`render_pixelpilot_env` signatures unchanged; `_validate_pixelpilot(pp: dict) -> None` unchanged. Field names (`screenMode`, `rtpPort`, `dvr.dir`, etc.) consistent across tasks and docs. ✓
