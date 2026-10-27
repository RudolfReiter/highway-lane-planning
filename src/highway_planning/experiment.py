"""Evaluate three deterministic traffic scenes and export reproducible plans."""
import argparse
import json
from pathlib import Path
from dataclasses import asdict
import numpy as np
from .planner import SCENARIOS, plan, DT, N, LONG_HORIZON, LANE_WIDTH, BUFFER, VREF


def jsonable(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [jsonable(v) for v in value]
    return value


def run():
    data = {"kind": "highway", "version": 1, "recreated": True, "seed": None,
            "config": {"dt": DT, "steps": N, "long_horizon_s": LONG_HORIZON, "lane_width_m": LANE_WIDTH,
                       "buffer_m": BUFFER, "reference_speed_mps": VREF},
            "strategies": [{"id": "coupled", "name": "Coupled long–short term"}, {"id": "short", "name": "Short-term first"}], "scenarios": []}
    for scenario in SCENARIOS:
        row = asdict(scenario)
        row["runs"] = [plan(scenario, mode) for mode in ("coupled", "short")]
        for result in row["runs"]:
            m = result["metrics"]
            print(f"{scenario.id:6s} / {result['id']:7s}: goal transition {m['goal_transition_s']}, speed RMSE {m['speed_rmse_mps']:.3f}, collision-free {m['collision_free']}")
        data["scenarios"].append(row)
    return jsonable(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("docs/data"))
    args = parser.parse_args()
    data = run()
    args.output.mkdir(parents=True, exist_ok=True)
    content = json.dumps(data, separators=(",", ":"), allow_nan=False)
    (args.output / "experiment.json").write_text(content + "\n")
    (args.output / "experiment.js").write_text("window.EXPERIMENT=" + content + ";\n")
    summary = [{"scenario": s["id"], "method": r["id"], **r["metrics"]} for s in data["scenarios"] for r in s["runs"]]
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__": main()
