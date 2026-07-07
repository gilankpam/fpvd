"""Closed-loop channel sim harness (gs/tools/simulate_channel.py)."""

import importlib.util
import json
from pathlib import Path

TOOL = Path(__file__).resolve().parents[2] / "tools" / "simulate_channel.py"


def _load_tool():
    spec = importlib.util.spec_from_file_location("simulate_channel", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _write_log(path, n=50):
    with open(path, "w") as f:
        for i in range(n):
            f.write(
                json.dumps(
                    {
                        "ts": 1.0 + 0.1 * i,
                        "snr": 30.0,
                        "snr_ewma": 30.0,
                        "snr_knees": [None, None, 6.0, None, 16.5, 21.0, None, None],
                    }
                )
                + "\n"
            )


def test_fit_viability_interpolates_and_extends(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    _write_log(log)
    _, knees = mod.load_channel(str(log))
    v = mod.fit_viability(knees)
    assert v[2] == 6.0 and v[4] == 16.5 and v[5] == 21.0
    assert v[3] == 11.25  # interior gap: linear between rung 2 (6.0) and rung 4 (16.5)
    assert v[6] > v[5] and v[1] < v[2]  # extended past both learned ends
    assert all(x is not None for x in v)


def test_closed_loop_run_produces_metrics(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    _write_log(log)
    from fpvdgs.dynlink.policy import Policy, PolicyConfig
    from fpvdgs.dynlink.signals import Signals

    ticks, knees = mod.load_channel(str(log))
    v = mod.fit_viability(knees)
    m = mod.run(ticks, v, Policy, PolicyConfig, Signals)
    assert m["ticks"] == 50
    assert m["glitch_ticks"] == 0  # snr 30 sits above every threshold
    assert set(m) >= {"glitch_s", "mcs_changes", "changes_per_min", "mean_mcs", "time_at_mcs"}
